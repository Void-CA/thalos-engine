//! [`AnalysisReport`] — the canonical container for analysis results (spec
//! `analysis-report-contract`).
//!
//! The report aggregates observations, actions, metrics and a summary, and is the
//! single output type for all analyzers. It enforces the separation between
//! diagnostic facts (observations) and remediation (actions), and between the
//! domain model and its renderers.
//!
//! # Invariants (enforced by [`AnalysisReport::validate`])
//!
//! - **Report artifact (I3)**: the report references the artifact it analyzes.
//! - **Acyclic causal graph (I4)**: `causes[]` is a directed acyclic graph over
//!   the report's observations; cycles and dangling references are rejected.
//! - **Causal direction (I4, feedback loop)**: plan observations
//!   (MotionPlan artifact) must not reference feedback observations
//!   (ExecutionSession artifact) in `causes[]` — causality flows
//!   feedback → plan only.
//! - **Actions reference observations (I5)**: every action's `target_observation`
//!   must exist in the report; observations never carry remediation.
//! - **Unique identities (I8)**: observation ids (and action ids) are unique
//!   within the report, so merging observations from independent analyzers never
//!   collides.
//!
//! `validate` checks ONLY structural invariants (design C1). It never computes
//! `quality_index`, never interprets attributes, and never judges whether an
//! observation is "correct" — that is the aggregator's responsibility.

use std::collections::{BTreeMap, HashMap, HashSet};

use serde::{Deserialize, Serialize};

use crate::analysis::action::{Action, ActionId};
use crate::analysis::observation::{ArtifactRef, Observation, ObservationId};
use crate::analysis::summary::AnalysisSummary;

/// [`AnalysisReport`] validation failure. `validate` reports the FIRST violation
/// it finds (fail-fast); each variant maps 1:1 to a structural invariant
/// (I4, I5, I8).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ReportError {
    /// Two observations share the same id (I8).
    #[error("duplicate observation id {0:?}")]
    DuplicateObservationId(ObservationId),
    /// An observation references an id that does not exist in the report (I4).
    #[error("observation {from:?} references unknown observation {target:?}")]
    DanglingReference {
        /// The observation holding the bad reference.
        from: ObservationId,
        /// The referenced id that does not exist.
        target: ObservationId,
    },
    /// `causes[]` contains a cycle (I4).
    #[error("causal cycle detected at observation {0:?}")]
    CycleDetected(ObservationId),
    /// A plan observation (MotionPlan artifact) references a feedback
    /// observation (ExecutionSession artifact) in `causes[]` (I4 direction:
    /// the causal graph flows feedback → plan only).
    #[error(
        "plan observation {from:?} must not reference feedback observation {target:?} in causes[]"
    )]
    DirectionViolation {
        /// The plan observation holding the invalid reference.
        from: ObservationId,
        /// The referenced feedback observation.
        target: ObservationId,
    },
    /// Two actions share the same id (I8).
    #[error("duplicate action id {0:?}")]
    DuplicateActionId(ActionId),
    /// An action targets an observation id that does not exist in the report (I5).
    #[error("action {action:?} references unknown observation {target:?}")]
    UnknownTargetObservation {
        /// The action holding the bad reference.
        action: ActionId,
        /// The targeted id that does not exist.
        target: ObservationId,
    },
}

/// The canonical container for analysis results (spec `analysis-report-contract`).
///
/// # Structure
///
/// - `artifact`: the artifact this report analyzes (I3).
/// - `observations`: diagnostic facts — immutable, machine-readable (I1-I3).
/// - `actions`: remediation steps referencing observations by id (I5).
/// - `metrics`: named numeric measurements of the analysis run itself. Type
///   chosen per design.md Interfaces (`BTreeMap<String, f64>`); the spec's
///   `metrics[]` notation is the JSON wire shape, projected by the DTO (I6).
/// - `summary`: derived quality view (I7, design C2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AnalysisReport {
    /// The artifact this report analyzes (I3).
    pub artifact: ArtifactRef,
    /// Analysis facts produced by analyzers.
    pub observations: Vec<Observation>,
    /// Remediation actions targeting observations by id (I5).
    pub actions: Vec<Action>,
    /// Named measurements of the analysis run (deterministic order).
    pub metrics: BTreeMap<String, f64>,
    /// Derived quality summary (I7).
    pub summary: AnalysisSummary,
    /// Stable identity of the robot the report analyzes (spec
    /// `analysis-report-contract`, `robot-identity`): the scene's existing
    /// identity (`metadata.id` for catalog robots, `urdf:<hash>` for URDF
    /// imports) — set by the caller from the scene snapshot, never synthesized
    /// from the chain. ADITIVO: `#[serde(default)]` — reports produced before
    /// this field deserialize to `None` without error.
    #[serde(default)]
    pub robot_id: Option<String>,
}

/// DFS node state for cycle detection (design C4: classic 3-color DFS).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VisitState {
    /// Node not entered yet.
    Unvisited,
    /// Node on the current DFS path — hitting it again means a cycle.
    Visiting,
    /// Node fully explored.
    Visited,
}

impl AnalysisReport {
    /// Validates the structural invariants of the report (I3, I4, I5, I8).
    ///
    /// Checks, in order:
    /// 1. observation ids are unique (I8);
    /// 2. every `causes`/`related` reference exists (I4 — no dangling references);
    /// 3. `causes[]` forms a directed acyclic graph (I4 — classic DFS, design C4);
    /// 4. action ids are unique and every `target_observation` exists (I5, I8).
    ///
    /// Deliberately NOT checked (design C1): `quality_index`, attribute
    /// interpretation, summary consistency with observations, or whether any
    /// observation is "correct". Those belong to the aggregator.
    pub fn validate(&self) -> Result<(), ReportError> {
        // 1. Unique observation ids (I8).
        let mut id_index: HashMap<ObservationId, usize> = HashMap::new();
        for (i, obs) in self.observations.iter().enumerate() {
            if id_index.insert(obs.id, i).is_some() {
                return Err(ReportError::DuplicateObservationId(obs.id));
            }
        }

        // 2. Dangling references in causes/related (I4).
        for obs in &self.observations {
            for target in obs.causes.iter().chain(obs.related.iter()) {
                if !id_index.contains_key(target) {
                    return Err(ReportError::DanglingReference {
                        from: obs.id,
                        target: *target,
                    });
                }
            }
        }

        // 2b. Causal direction (I4, feedback loop): plan observations MUST NOT
        // reference feedback observations in `causes[]`. The causal graph flows
        // feedback → plan only; `related[]` has no direction and is exempt.
        for obs in &self.observations {
            for target in &obs.causes {
                let target_obs = &self.observations[id_index[target]];
                if matches!(obs.artifact, ArtifactRef::MotionPlan(_))
                    && matches!(target_obs.artifact, ArtifactRef::ExecutionSession(_))
                {
                    return Err(ReportError::DirectionViolation {
                        from: obs.id,
                        target: *target,
                    });
                }
            }
        }

        // 3. Cycles on causes[] — classic 3-color DFS (design C4).
        let mut state = vec![VisitState::Unvisited; self.observations.len()];
        for i in 0..self.observations.len() {
            if state[i] == VisitState::Unvisited {
                self.visit(i, &id_index, &mut state)?;
            }
        }

        // 4. Actions: unique ids and existing targets (I5, I8).
        let mut action_ids: HashSet<ActionId> = HashSet::new();
        for action in &self.actions {
            if !action_ids.insert(action.id) {
                return Err(ReportError::DuplicateActionId(action.id));
            }
            if !id_index.contains_key(&action.target_observation) {
                return Err(ReportError::UnknownTargetObservation {
                    action: action.id,
                    target: action.target_observation,
                });
            }
        }

        Ok(())
    }

    /// Classic DFS over `causes[]` edges (design C4). `id_index` maps observation
    /// ids to indices; existence was proven in step 2, so indexing never misses.
    fn visit(
        &self,
        i: usize,
        id_index: &HashMap<ObservationId, usize>,
        state: &mut [VisitState],
    ) -> Result<(), ReportError> {
        state[i] = VisitState::Visiting;
        for target in &self.observations[i].causes {
            let j = id_index[target];
            match state[j] {
                VisitState::Visiting => {
                    return Err(ReportError::CycleDetected(self.observations[i].id));
                }
                VisitState::Unvisited => self.visit(j, id_index, state)?,
                VisitState::Visited => {}
            }
        }
        state[i] = VisitState::Visited;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{AnalysisReport, ReportError};
    use crate::analysis::action::{Action, ActionId, ActionImpact, ActionKind, ActionPriority};
    use crate::analysis::location::Location;
    use crate::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use crate::analysis::summary::{AnalysisSummary, Grade};
    use crate::ids::MotionPlanId;
    use std::collections::BTreeMap;

    fn observation(id: u32, causes: Vec<u32>, related: Vec<u32>) -> Observation {
        Observation {
            id: ObservationId(id),
            kind: ObservationKind::ResidualError,
            severity: Severity::Error,
            artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
            location: Location::Waypoint(0),
            attributes: BTreeMap::new(),
            causes: causes.into_iter().map(ObservationId).collect(),
            related: related.into_iter().map(ObservationId).collect(),
        }
    }

    /// Same as [`observation`], but anchored to an execution session — the
    /// feedback-domain counterpart used by the planning feedback loop.
    fn execution_observation(id: u32, causes: Vec<u32>, related: Vec<u32>) -> Observation {
        let mut obs = observation(id, causes, related);
        obs.artifact =
            ArtifactRef::ExecutionSession(crate::ids::ExecutionSessionId("es-1".to_string()));
        obs
    }

    fn action(id: u32, target: u32) -> Action {
        Action {
            id: ActionId(id),
            kind: ActionKind::Waypoint,
            target_observation: ObservationId(target),
            priority: ActionPriority::High,
            impact: ActionImpact::Medium,
            parameters: BTreeMap::new(),
        }
    }

    fn summary() -> AnalysisSummary {
        AnalysisSummary {
            quality_index: 0.85,
            observation_count: 0,
            severity_distribution: BTreeMap::new(),
            grade: Grade::Good,
        }
    }

    fn report(observations: Vec<Observation>, actions: Vec<Action>) -> AnalysisReport {
        AnalysisReport {
            artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
            observations,
            actions,
            metrics: BTreeMap::new(),
            summary: summary(),
            robot_id: None,
        }
    }

    #[test]
    fn valid_causal_chain_is_accepted() {
        // I4: A.causes=[B], B.causes=[C] → A→B→C is acyclic.
        let r = report(
            vec![
                observation(1, vec![2], vec![]),
                observation(2, vec![3], vec![]),
                observation(3, vec![], vec![]),
            ],
            vec![],
        );
        assert_eq!(r.validate(), Ok(()));
    }

    #[test]
    fn causal_cycle_is_rejected() {
        // I4 negative: A.causes=[B], B.causes=[A] → cycle A→B→A rejected.
        let r = report(
            vec![
                observation(1, vec![2], vec![]),
                observation(2, vec![1], vec![]),
            ],
            vec![],
        );
        let err = r.validate().expect_err("cycle A→B→A must be rejected");
        assert!(matches!(err, ReportError::CycleDetected(ObservationId(2))));
    }

    #[test]
    fn self_loop_is_rejected() {
        // I4 triangulation: a self-reference is a cycle of length one.
        let r = report(vec![observation(1, vec![1], vec![])], vec![]);
        let err = r.validate().expect_err("self-loop must be rejected");
        assert!(matches!(err, ReportError::CycleDetected(ObservationId(1))));
    }

    #[test]
    fn dangling_cause_reference_is_rejected() {
        // I4 negative: causes points at an id that does not exist in the report.
        let r = report(vec![observation(1, vec![99], vec![])], vec![]);
        let err = r.validate().expect_err("dangling cause must be rejected");
        assert!(matches!(
            err,
            ReportError::DanglingReference {
                from: ObservationId(1),
                target: ObservationId(99),
            }
        ));
    }

    #[test]
    fn dangling_related_reference_is_rejected() {
        // I4 triangulation: related[] has no direction, but references must still exist.
        let r = report(vec![observation(1, vec![], vec![99])], vec![]);
        let err = r.validate().expect_err("dangling related must be rejected");
        assert!(matches!(
            err,
            ReportError::DanglingReference {
                from: ObservationId(1),
                target: ObservationId(99),
            }
        ));
    }

    #[test]
    fn action_targeting_existing_observation_is_accepted() {
        // I5: an action references obs-1 by id and obs-1 remains unchanged.
        let r = report(vec![observation(1, vec![], vec![])], vec![action(10, 1)]);
        assert_eq!(r.validate(), Ok(()));
        assert_eq!(r.observations[0].id, ObservationId(1));
    }

    #[test]
    fn action_with_unknown_target_is_rejected() {
        // I5 negative: target_observation must reference an existing observation.
        let r = report(vec![observation(1, vec![], vec![])], vec![action(10, 99)]);
        let err = r.validate().expect_err("unknown target must be rejected");
        assert!(matches!(
            err,
            ReportError::UnknownTargetObservation {
                action: ActionId(10),
                target: ObservationId(99),
            }
        ));
    }

    #[test]
    fn empty_report_is_valid() {
        // Spec analysis-report-contract "Empty report": empty observations[] and
        // actions[], zero metrics, quality_index=1.0 — and validate() accepts it.
        let mut r = report(vec![], vec![]);
        r.summary.quality_index = 1.0;
        assert_eq!(r.validate(), Ok(()));
        assert!(r.observations.is_empty());
        assert!(r.actions.is_empty());
        assert!(r.metrics.is_empty());
        assert_eq!(r.summary.observation_count, 0);
        assert_eq!(r.summary.quality_index, 1.0);
    }

    #[test]
    fn duplicate_observation_ids_are_rejected() {
        // I8: observation ids are unique within a report, so merging analyzers
        // never collides.
        let r = report(
            vec![
                observation(1, vec![], vec![]),
                observation(1, vec![], vec![]),
            ],
            vec![],
        );
        let err = r
            .validate()
            .expect_err("duplicate observation ids must be rejected");
        assert!(matches!(
            err,
            ReportError::DuplicateObservationId(ObservationId(1))
        ));
    }

    #[test]
    fn duplicate_action_ids_are_rejected() {
        // I8: action ids are unique within a report.
        let r = report(
            vec![observation(1, vec![], vec![])],
            vec![action(10, 1), action(10, 1)],
        );
        let err = r
            .validate()
            .expect_err("duplicate action ids must be rejected");
        assert!(matches!(err, ReportError::DuplicateActionId(ActionId(10))));
    }

    #[test]
    fn report_serializes_with_all_four_sections() {
        // Spec analysis-report-contract "Complete report": the canonical output
        // carries observations[], actions[], metrics[] and summary (+ artifact, I3).
        let r = report(vec![observation(1, vec![], vec![])], vec![action(10, 1)]);
        let value = serde_json::to_value(&r).expect("serialize");
        let obj = value.as_object().expect("object");
        for section in ["artifact", "observations", "actions", "metrics", "summary"] {
            assert!(obj.contains_key(section), "report must carry `{section}`");
        }
        assert_eq!(obj["observations"].as_array().map(|a| a.len()), Some(1));
        assert_eq!(obj["actions"].as_array().map(|a| a.len()), Some(1));
    }

    // ── Spec analysis-report-contract "Legacy report without robot_id" ──────

    /// Old JSON (produced before this change) carries no `robot_id` — the new
    /// backend must deserialize it to `None` via `#[serde(default)]`, never
    /// fail.
    #[test]
    fn legacy_json_without_robot_id_deserializes_to_none() {
        let mut r = report(vec![], vec![]);
        r.robot_id = Some("icebot-42".to_string());
        let mut value: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&r).expect("serialize")).expect("json");
        value.as_object_mut().expect("object").remove("robot_id");
        let legacy = serde_json::to_string(&value).expect("json");

        let back: AnalysisReport =
            serde_json::from_str(&legacy).expect("legacy JSON must deserialize");
        assert_eq!(
            back.robot_id, None,
            "a report without robot_id must deserialize to None"
        );
    }

    /// Round-trip: `robot_id` set on the domain model survives the wire and
    /// comes back identical.
    #[test]
    fn robot_id_survives_serialization_round_trip() {
        let mut r = report(vec![], vec![]);
        r.robot_id = Some("icebot-42".to_string());
        let json = serde_json::to_string(&r).expect("serialize");
        let back: AnalysisReport = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(
            back.robot_id.as_deref(),
            Some("icebot-42"),
            "robot_id must survive the round trip"
        );
    }

    /// Spec robot-identity "Different robots → distinguishable reports": two
    /// reports from different robots differ in their `robot_id` wire value.
    #[test]
    fn different_robot_ids_produce_distinguishable_reports() {
        let mut a = report(vec![], vec![]);
        a.robot_id = Some("robot-a".to_string());
        let mut b = report(vec![], vec![]);
        b.robot_id = Some("robot-b".to_string());

        let ja = serde_json::to_value(&a).expect("serialize");
        let jb = serde_json::to_value(&b).expect("serialize");
        assert_eq!(ja["robot_id"], "robot-a");
        assert_eq!(jb["robot_id"], "robot-b");
        assert_ne!(ja, jb, "reports from different robots must differ");
    }

    #[test]
    fn structural_invariants_survive_serialization_round_trip() {
        // Stability test (user requirement): the structural invariants must survive
        // the wire. validate(report) → OK → serialize → deserialize →
        // validate(report') → OK.
        let r = report(
            vec![
                observation(1, vec![], vec![]),  // A
                observation(2, vec![1], vec![]), // B.causes=[A]
                observation(3, vec![2], vec![]), // C.causes=[B]  → A→B→C acyclic
            ],
            vec![action(10, 3)], // action targets C by id
        );
        assert_eq!(r.validate(), Ok(()), "source report must be valid");

        let json = serde_json::to_string(&r).expect("serialize");
        let back: AnalysisReport = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(
            back.validate(),
            Ok(()),
            "invariants must survive the round trip"
        );

        // The structural edges (I4 chain + I5 action reference) came through intact.
        assert_eq!(back.observations[1].causes, vec![ObservationId(1)]);
        assert_eq!(back.observations[2].causes, vec![ObservationId(2)]);
        assert_eq!(back.actions[0].target_observation, ObservationId(3));
    }

    #[test]
    fn feedback_observation_may_reference_plan_observation_in_causes() {
        // I4 direction (feedback loop): F.causes=[P] (feedback → plan) is valid.
        let r = report(
            vec![
                observation(1, vec![], vec![]),            // P — plan observation
                execution_observation(2, vec![1], vec![]), // F.causes=[P]
            ],
            vec![],
        );
        assert_eq!(r.validate(), Ok(()));
    }

    #[test]
    fn plan_observation_must_not_reference_feedback_in_causes() {
        // I4 negative: P.causes=[F] (plan → feedback) is rejected.
        let r = report(
            vec![
                observation(1, vec![2], vec![]),          // P.causes=[F]
                execution_observation(2, vec![], vec![]), // F
            ],
            vec![],
        );
        let err = r.validate().expect_err("P.causes=[F] must be rejected");
        assert!(matches!(
            err,
            ReportError::DirectionViolation {
                from: ObservationId(1),
                target: ObservationId(2),
            }
        ));
    }

    #[test]
    fn direction_rule_scoped_to_causes_not_related() {
        // related[] carries no causal direction — cross-layer related links
        // stay valid even though the same edge in causes[] would be rejected.
        let r = report(
            vec![
                observation(1, vec![], vec![2]),
                execution_observation(2, vec![], vec![]),
            ],
            vec![],
        );
        assert_eq!(r.validate(), Ok(()));
    }
}
