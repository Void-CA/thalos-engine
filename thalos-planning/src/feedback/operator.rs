//! Proposal type over the unified observation model.
//!
//! Defines the [`ActionProposal`] type that the advisor produces for
//! remediating observations. The `ObservationIntentionOperator` trait
//! contract was removed in the phase-7 deletion: remediation is decided
//! directly by the advisor over observations, and materialized by the
//! `ProposalMaterializer`s in [`crate::feedback::materializer`].
//!
//! ## Trait Contract (removed)
//!
//! The removed operator trait required `name()`, `applies_to()` and `apply()`
//! producing zero or more [`ActionProposal`]s — never mutations of the
//! observation, never plan modifications.

use std::collections::BTreeMap;

use thalos_core::analysis::action::{Action, ActionId, ActionImpact, ActionKind, ActionPriority};
use thalos_core::analysis::attribute_value::AttributeValue;
use thalos_core::analysis::observation::ObservationId;

// ============================================================================
// ActionProposal (PR 4b — new model)
// ============================================================================

/// Proposal for a remediation action over the observation model (spec I5).
///
/// This is the "operator modeled as action" intermediate type: an operator
/// produces [`ActionProposal`]s that reference an observation by id, WITHOUT
/// fabricating an [`ActionId`]. Assigning ids is the aggregator's job
/// (1..=n during report construction) — operators never hardcode them
/// (PR 4a gotcha).
///
/// The proposal exists because operators must not mutate observations (C4)
/// and must not claim an identity they do not own; it represents an
/// *intention*, not a plan modification (C3).
#[derive(Debug, Clone, PartialEq)]
pub struct ActionProposal {
    /// The remediation kind (e.g. [`ActionKind::SwitchMoveStrategy`]).
    pub kind: ActionKind,
    /// The observation this proposal remediates (I5).
    pub target_observation: ObservationId,
    /// Scheduling priority of the remediation.
    pub priority: ActionPriority,
    /// Expected impact on the artifact's quality.
    pub impact: ActionImpact,
    /// Typed parameters for the remediation (stable keys, D5).
    pub parameters: BTreeMap<String, AttributeValue>,
}

impl ActionProposal {
    /// Materializes the proposal into a full [`Action`] with a caller-owned id.
    ///
    /// The id is supplied by the consumer (the aggregator assigns 1..=n) —
    /// the proposal itself carries no identity, so operators cannot hardcode
    /// ids.
    pub fn materialize(&self, id: ActionId) -> Action {
        Action {
            id,
            kind: self.kind,
            target_observation: self.target_observation,
            priority: self.priority,
            impact: self.impact,
            parameters: self.parameters.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use thalos_core::analysis::action::{
        Action, ActionId, ActionImpact, ActionKind, ActionPriority,
    };
    use thalos_core::analysis::location::Location;
    use thalos_core::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use thalos_core::analysis::report::{AnalysisReport, ReportError};
    use thalos_core::analysis::summary::{AnalysisSummary, Grade};
    use thalos_core::ids::{ExecutionSessionId, MotionPlanId};

    /// An execution-domain observation (feedback loop vocabulary).
    fn execution_observation(id: u32, kind: ObservationKind, causes: Vec<u32>) -> Observation {
        Observation {
            id: ObservationId(id),
            kind,
            severity: Severity::Error,
            artifact: ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string())),
            location: Location::Timestamp(400),
            attributes: BTreeMap::new(),
            causes: causes.into_iter().map(ObservationId).collect(),
            related: Vec::new(),
        }
    }

    /// A plan-domain observation.
    fn plan_observation(id: u32, kind: ObservationKind) -> Observation {
        Observation {
            id: ObservationId(id),
            kind,
            severity: Severity::Warning,
            artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
            location: Location::Waypoint(0),
            attributes: BTreeMap::new(),
            causes: Vec::new(),
            related: Vec::new(),
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

    #[test]
    fn action_proposal_has_no_id_and_materializes_with_caller_id() {
        // ActionId gotcha (PR 4a): the operator must not fabricate ids — the
        // proposal carries none; the aggregator assigns 1..=n at materialization.
        let proposal = ActionProposal {
            kind: ActionKind::SwitchMoveStrategy,
            target_observation: ObservationId(3),
            priority: ActionPriority::High,
            impact: ActionImpact::High,
            parameters: BTreeMap::new(),
        };

        let action: Action = proposal.materialize(ActionId(9));
        assert_eq!(action.id, ActionId(9));
        assert_eq!(action.kind, ActionKind::SwitchMoveStrategy);
        assert_eq!(action.target_observation, ObservationId(3));
    }

    #[test]
    fn feedback_observation_may_be_caused_by_plan_observation() {
        // C5 / I4 direction: F.causes=[P] (feedback → plan) is accepted.
        let report = AnalysisReport {
            artifact: ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string())),
            observations: vec![
                plan_observation(1, ObservationKind::NearSingularity),
                execution_observation(2, ObservationKind::TrackingError, vec![1]),
            ],
            actions: Vec::new(),
            metrics: BTreeMap::new(),
            summary: summary(),
            robot_id: None,
        };
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn plan_observation_must_not_be_caused_by_feedback() {
        // C5 / I4 negative: P.causes=[F] (plan → feedback) is rejected.
        let mut report = AnalysisReport {
            artifact: ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string())),
            observations: vec![
                plan_observation(1, ObservationKind::NearSingularity),
                execution_observation(2, ObservationKind::TrackingError, vec![]),
            ],
            actions: Vec::new(),
            metrics: BTreeMap::new(),
            summary: summary(),
            robot_id: None,
        };
        report.observations[0].causes = vec![ObservationId(2)];

        let err = report
            .validate()
            .expect_err("P.causes=[F] must be rejected");
        assert!(matches!(
            err,
            ReportError::DirectionViolation {
                from: ObservationId(1),
                target: ObservationId(2),
            }
        ));
    }
}
