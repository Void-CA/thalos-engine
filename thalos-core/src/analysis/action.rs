//! Remediation actions attached to an
//! [`AnalysisReport`](crate::analysis::report::AnalysisReport).
//!
//! An [`Action`] is a remediation step that targets a single
//! [`Observation`](crate::analysis::observation::Observation) by id (spec I5).
//! Actions live at the report level, never inside observations: diagnosis and
//! remediation are separate concerns.
//!
//! # Invariants
//!
//! - **Actions reference observations (I5)**: `target_observation` points at an
//!   observation id present in the report; existence is enforced by
//!   [`AnalysisReport::validate`](crate::analysis::report::AnalysisReport::validate).
//! - **One-directional navigation (design C3)**: `Action → Observation` only.
//!   `Observation` carries no back-reference to actions, so the report graph has
//!   a single navigation direction.
//! - **Immutable facts (I5)**: actions never mutate observations — applying
//!   remediation is the aggregator/operator's job (later phases).
//!
//! The vocabulary enums are `#[non_exhaustive]`, following the observation model
//! pattern: new action kinds, priorities and impacts can be added without
//! breaking consumers.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::analysis::attribute_value::AttributeValue;
use crate::analysis::observation::ObservationId;

/// Stable identity of an action within a report (counter newtype over `u32`,
/// mirroring [`ObservationId`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ActionId(pub u32);

/// What kind of remediation the action prescribes.
///
/// Variants are grounded in the existing plan-advisor suggestions
/// (`SuggestionKind`) plus the feedback-loop `SwitchMoveStrategy`, so the Phase 3
/// and Phase 4 migrations map onto them directly. `#[non_exhaustive]`: new kinds
/// are added without breaking consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ActionKind {
    /// Propose a different IK solution.
    IkSolution,
    /// Adjust a velocity profile.
    Velocity,
    /// Modify a waypoint.
    Waypoint,
    /// Alter the path to avoid a collision.
    Collision,
    /// Adjust the trajectory near a singularity.
    Singularity,
    /// Improve the manipulability of a configuration.
    Manipulability,
    /// Relax or enforce a constraint.
    Constraint,
    /// Switch the motion strategy during execution (feedback loop).
    SwitchMoveStrategy,
}

/// Priority of an action, for scheduling remediation.
///
/// Three levels, mirroring the proven plan-advisor impact model.
/// `#[non_exhaustive]`: levels can be added without breaking consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ActionPriority {
    /// Low priority.
    Low,
    /// Medium priority.
    Medium,
    /// High priority.
    High,
}

/// Expected impact of an action on the artifact's quality.
///
/// Mirrors the plan advisor's proven three-level `Impact` model.
/// `#[non_exhaustive]`: levels can be added without breaking consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ActionImpact {
    /// Low expected impact.
    Low,
    /// Medium expected impact.
    Medium,
    /// High expected impact.
    High,
}

/// A remediation action targeting a single observation by id (spec I5).
///
/// # Invariants
///
/// - `target_observation` MUST reference an observation present in the report
///   (enforced by
///   [`AnalysisReport::validate`](crate::analysis::report::AnalysisReport::validate)).
/// - `parameters` are typed [`AttributeValue`]s, same policy as observation
///   `attributes` (D5): stable string keys, never presentation strings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Action {
    /// Stable identity within the report.
    pub id: ActionId,
    /// The remediation kind.
    pub kind: ActionKind,
    /// The observation this action remediates (I5).
    pub target_observation: ObservationId,
    /// Scheduling priority of the remediation.
    pub priority: ActionPriority,
    /// Expected impact on the artifact's quality.
    pub impact: ActionImpact,
    /// Typed parameters for the remediation (stable keys, D5).
    pub parameters: BTreeMap<String, AttributeValue>,
}

#[cfg(test)]
mod tests {
    use super::{Action, ActionId, ActionImpact, ActionKind, ActionPriority};
    use crate::analysis::attribute_value::AttributeValue;
    use serde_json::json;
    use std::collections::BTreeMap;

    fn action(id: u32, target: u32) -> Action {
        Action {
            id: ActionId(id),
            kind: ActionKind::Waypoint,
            target_observation: crate::analysis::observation::ObservationId(target),
            priority: ActionPriority::High,
            impact: ActionImpact::Medium,
            parameters: BTreeMap::new(),
        }
    }

    #[test]
    fn action_serializes_target_as_observation_id() {
        // I5: target_observation is the observation id on the wire, never an
        // embedded copy of the observation.
        let value = serde_json::to_value(action(10, 3)).expect("serialize");
        assert_eq!(value["target_observation"], json!(3));
    }

    #[test]
    fn action_id_is_transparent_counter() {
        let id = ActionId(42);
        assert_eq!(serde_json::to_value(id).expect("serialize"), json!(42));
        let back: ActionId = serde_json::from_value(json!(42)).expect("deserialize");
        assert_eq!(back, id);
    }

    #[test]
    fn action_kind_round_trip_all_variants() {
        let kinds = [
            ActionKind::IkSolution,
            ActionKind::Velocity,
            ActionKind::Waypoint,
            ActionKind::Collision,
            ActionKind::Singularity,
            ActionKind::Manipulability,
            ActionKind::Constraint,
            ActionKind::SwitchMoveStrategy,
        ];
        for kind in kinds {
            let json = serde_json::to_string(&kind).expect("serialize");
            let back: ActionKind = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(back, kind);
        }
    }

    #[test]
    fn action_priority_and_impact_round_trip() {
        for p in [
            ActionPriority::Low,
            ActionPriority::Medium,
            ActionPriority::High,
        ] {
            let json = serde_json::to_string(&p).expect("serialize");
            let back: ActionPriority = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(back, p);
        }
        for i in [ActionImpact::Low, ActionImpact::Medium, ActionImpact::High] {
            let json = serde_json::to_string(&i).expect("serialize");
            let back: ActionImpact = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(back, i);
        }
    }

    #[test]
    fn action_parameters_are_typed_attribute_values() {
        // D5: parameters use typed AttributeValue, never presentation strings.
        let mut a = action(1, 3);
        a.parameters.insert(
            "strategy".to_string(),
            AttributeValue::Text("switch_to_osc".to_string()),
        );
        let value = serde_json::to_value(&a).expect("serialize");
        assert_eq!(
            value["parameters"]["strategy"],
            json!({"Text": "switch_to_osc"})
        );
    }
}
