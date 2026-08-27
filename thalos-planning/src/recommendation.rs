//! Bridge type between analysis facts ([`Action`]) and planning commands
//! ([`ProgramEdit`]) (design D3).
//!
//! Materializers (PR2) convert action proposals into executable motion
//! segments and wrap them in a `Recommendation`; the apply pipeline (PR4/PR5)
//! stores its edit and inverse for O(1) undo.

use serde::{Deserialize, Serialize};
use thalos_core::analysis::action::Action;

use crate::program_edit::ProgramEdit;

/// Stable identity of a recommendation within an analysis report (counter
/// newtype over `u32`, mirroring `ActionId`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RecommendationId(pub u32);

/// Availability of a recommendation (design D8).
///
/// `Unavailable` marks recommendations whose materialization failed (e.g. no
/// IK solution) — they stay present in the output instead of being dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RecommendationStatus {
    /// The edit can be applied.
    Available,
    /// The edit cannot be materialized/applied (IK failure, etc.).
    Unavailable,
}

/// Why a recommendation is [`RecommendationStatus::Unavailable`] (design
/// ADR-2, spec recommendation-availability-contract "Availability Reason
/// Exposure"). Additive on the wire: serialized `snake_case`, projected only
/// when present so old clients without the field keep deserializing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UnavailabilityReason {
    /// Inverse kinematics did not converge for the edit's target (pose or
    /// position unreachable from the segment-start joints).
    IkFailed,
    /// The edited program cannot be recompiled (invalid goal, joint limit,
    /// collision, …).
    CompileFailed,
    /// The edited program compiles but re-analysis fails.
    PlanningFailed,
    /// The target configuration cannot be reached at all (e.g. a tool
    /// rotation a planar robot cannot realize).
    UnreachableConfiguration,
    /// The remediation does not apply to this proposal/target.
    NotApplicable,
    /// The target segment type is not supported by the remediation.
    Unsupported,
}

/// A bridge between an analysis [`Action`] and a planning [`ProgramEdit`]
/// (design D3).
///
/// `id` MUST be unique per analysis report (spec recommendation-model). The
/// `status` field is optional for wire compatibility — old clients that do
/// not know the field still deserialize (`#[serde(default)]`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Recommendation {
    /// Unique id within the analysis report.
    pub id: RecommendationId,
    /// The analysis fact this recommendation remediates.
    pub action: Action,
    /// The planning command that applies the remediation.
    pub edit: ProgramEdit,
    /// Availability; `None` means "not evaluated" and is omitted on the wire.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub status: Option<RecommendationStatus>,
    /// Why the recommendation is unavailable (design ADR-2). Additive:
    /// `None` — omitted on the wire — keeps old clients deserializing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<UnavailabilityReason>,
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use thalos_core::analysis::action::{
        Action, ActionId, ActionImpact, ActionKind, ActionPriority,
    };
    use thalos_core::analysis::observation::ObservationId;
    use thalos_core::ids::OperationId;
    use thalos_core::motion::segment::MotionSegment;

    use super::{Recommendation, RecommendationId, RecommendationStatus, UnavailabilityReason};
    use crate::program_edit::ProgramEdit;

    fn action(id: u32) -> Action {
        Action {
            id: ActionId(id),
            kind: ActionKind::Waypoint,
            target_observation: ObservationId(3),
            priority: ActionPriority::High,
            impact: ActionImpact::Medium,
            parameters: BTreeMap::new(),
        }
    }

    fn edit() -> ProgramEdit {
        ProgramEdit::MoveWaypoint {
            segment_index: 0,
            new_target: vec![4.0, 4.0],
            old_target: Some(vec![1.0, 1.0]),
        }
    }

    #[test]
    fn recommendation_construction_exposes_id_action_and_edit() {
        // Spec recommendation-model "Recommendation construction": id, action
        // and edit are accessible after construction.
        let rec = Recommendation {
            id: RecommendationId(7),
            action: action(1),
            edit: edit(),
            status: None,
            reason: None,
        };

        assert_eq!(rec.id, RecommendationId(7));
        assert_eq!(rec.action.kind, ActionKind::Waypoint);
        assert_eq!(rec.action.target_observation, ObservationId(3));
        assert!(matches!(
            rec.edit,
            ProgramEdit::MoveWaypoint {
                segment_index: 0,
                ..
            }
        ));
        assert_eq!(rec.status, None);
    }

    #[test]
    fn recommendation_status_serializes_as_snake_case() {
        // D8: status "unavailable" on the wire when IK fails.
        let json = serde_json::to_string(&RecommendationStatus::Unavailable).expect("serialize");
        assert_eq!(json, "\"unavailable\"");
        let json = serde_json::to_string(&RecommendationStatus::Available).expect("serialize");
        assert_eq!(json, "\"available\"");
    }

    #[test]
    fn recommendation_id_is_transparent_counter() {
        // Mirrors ActionId: transparent u32 on the wire.
        let id = RecommendationId(42);
        let json = serde_json::to_string(&id).expect("serialize");
        assert_eq!(json, "42");
        let back: RecommendationId = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, id);
    }

    #[test]
    fn recommendation_round_trips_through_json() {
        let rec = Recommendation {
            id: RecommendationId(9),
            action: action(2),
            edit: edit(),
            status: Some(RecommendationStatus::Unavailable),
            reason: None,
        };

        let json = serde_json::to_string(&rec).expect("serialize");
        let back: Recommendation = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(back, rec);
        assert_eq!(back.status, Some(RecommendationStatus::Unavailable));
    }

    #[test]
    fn recommendation_serializes_edit_variant_shape() {
        // The edit must cross the wire with its variant and payload (PR2 wire
        // contract consumes it), and a missing status defaults to None.
        let rec = Recommendation {
            id: RecommendationId(1),
            action: action(1),
            edit: edit(),
            status: None,
            reason: None,
        };

        let value = serde_json::to_value(&rec).expect("serialize");
        assert!(
            value["edit"]["MoveWaypoint"].is_object(),
            "edit variant on the wire"
        );
        assert_eq!(value["id"], serde_json::json!(1));
        assert!(
            value.get("status").is_none(),
            "None status is skipped on the wire"
        );
    }

    #[test]
    fn recommendation_deserializes_without_status_field() {
        // Wire compat (PR2): old JSON without `status` still deserializes.
        let rec = Recommendation {
            id: RecommendationId(2),
            action: action(3),
            edit: edit(),
            status: None,
            reason: None,
        };
        let mut value = serde_json::to_value(&rec).expect("serialize");
        value.as_object_mut().expect("object").remove("status");

        let back: Recommendation = serde_json::from_value(value).expect("deserialize");
        assert_eq!(back.status, None);
    }

    // ── T5 (M2): additive `reason` field (design ADR-2) ────────────────────
    //
    // Spec recommendation-availability-contract "Availability Reason
    // Exposure": the DTO MUST carry a reason when unavailable. Additive:
    // old JSON without `reason` deserializes to None; new JSON round-trips.

    #[test]
    fn unavailability_reason_serializes_as_snake_case() {
        // All six documented variants project snake_case on the wire.
        let cases = [
            (UnavailabilityReason::IkFailed, "ik_failed"),
            (UnavailabilityReason::CompileFailed, "compile_failed"),
            (UnavailabilityReason::PlanningFailed, "planning_failed"),
            (
                UnavailabilityReason::UnreachableConfiguration,
                "unreachable_configuration",
            ),
            (UnavailabilityReason::NotApplicable, "not_applicable"),
            (UnavailabilityReason::Unsupported, "unsupported"),
        ];
        for (reason, expected) in cases {
            let json = serde_json::to_string(&reason).expect("serialize");
            assert_eq!(json, format!("\"{expected}\""));
        }
    }

    #[test]
    fn recommendation_round_trips_reason_through_json() {
        // A new-client payload carrying the reason round-trips losslessly.
        let rec = Recommendation {
            id: RecommendationId(10),
            action: action(4),
            edit: edit(),
            status: Some(RecommendationStatus::Unavailable),
            reason: Some(UnavailabilityReason::IkFailed),
        };

        let json = serde_json::to_string(&rec).expect("serialize");
        let back: Recommendation = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(back, rec);
        assert_eq!(back.reason, Some(UnavailabilityReason::IkFailed));
    }

    #[test]
    fn recommendation_deserializes_without_reason_field() {
        // Old JSON without `reason` still deserializes (serde default → None),
        // and an explicit None is skipped on the wire (additive contract).
        let rec = Recommendation {
            id: RecommendationId(11),
            action: action(5),
            edit: edit(),
            status: Some(RecommendationStatus::Unavailable),
            reason: None,
        };
        let mut value = serde_json::to_value(&rec).expect("serialize");
        let obj = value.as_object_mut().expect("object");
        obj.remove("reason");

        let back: Recommendation = serde_json::from_value(value).expect("deserialize");
        assert_eq!(back.reason, None);
        assert_eq!(back.status, Some(RecommendationStatus::Unavailable));
        assert_eq!(back.id, RecommendationId(11));
    }

    // ── Helper guard: a recommendation edit must be a real planning edit ────

    #[test]
    fn recommendation_edit_applies_to_a_plan() {
        // The bridge is functional: the recommendation's edit transforms a
        // real program (used by preview/apply in PR3/PR4).
        let program = crate::motion::program::PlanningProgram::new(vec![MotionSegment::MoveJ {
            origin: OperationId("op-j".to_string()),
            target: vec![1.0, 1.0],
            max_velocity: Some(500.0),
            max_acceleration: Some(1000.0),
        }]);

        let result = edit().apply(&program).expect("edit must apply");
        assert!(matches!(
            &result.segments[0],
            MotionSegment::MoveJ { target, .. } if target == &vec![4.0, 4.0]
        ));
    }
}
