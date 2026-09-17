use std::collections::BTreeMap;

use thalos_core::analysis::attribute_value::AttributeValue;
use thalos_core::analysis::location::Location;
use thalos_core::analysis::observation::{
    ArtifactRef, Observation, ObservationId, ObservationKind, Severity as ObservationSeverity,
};
use thalos_core::ids::{OperationId, SemanticProgramId};

use crate::knowledge::KnowledgeProvider;
use crate::program::SemanticProgram;

mod level1;
mod level2;

/// Stable artifact anchor for validation observations (spec I3).
///
/// `SemanticProgram` carries no identity today (it is a bare sequence of
/// operations), so the validator anchors every observation to the
/// `SemanticProgram` artifact with this stable placeholder id. Supplying the
/// real program id is a follow-up when program identity lands on the model.
const PROGRAM_ARTIFACT_ID: &str = "semantic-program";

/// Anchor every validation observation to the program under validation (I3).
fn program_artifact() -> ArtifactRef {
    ArtifactRef::SemanticProgram(SemanticProgramId(PROGRAM_ARTIFACT_ID.to_string()))
}

/// Build a canonical validation observation (spec I1/I2/I3).
///
/// All semantic validation observations are anchored at the operation that
/// caused them (`Location::Operation(origin)`). The id is a placeholder — the
/// aggregator reassigns `1..=n` (closed decision). `attributes` carry typed
/// domain data only; presentation text is the renderer's responsibility (I1).
fn observation(
    kind: ObservationKind,
    severity: ObservationSeverity,
    origin: OperationId,
    attributes: BTreeMap<String, AttributeValue>,
) -> Observation {
    Observation {
        id: ObservationId(0),
        kind,
        severity,
        artifact: program_artifact(),
        location: Location::Operation(origin),
        attributes,
        causes: Vec::new(),
        related: Vec::new(),
    }
}

/// Run Level 1 validation (sequence rules, no provider needed).
///
/// Emits canonical [`Observation`]s — Place-without-Pick violations and other
/// sequence rules. Every observation is machine-readable (spec I2) and carries
/// no presentation text (spec I1).
pub fn validate(program: &SemanticProgram) -> Vec<Observation> {
    level1::validate_level1(program)
}

/// Run both Level 1 and Level 2 validation.
///
/// Level 2 requires a `KnowledgeProvider` to resolve resource references and
/// is skipped if Level 1 produced any error-severity observation.
pub fn validate_with_provider(
    program: &SemanticProgram,
    provider: &dyn KnowledgeProvider,
) -> Vec<Observation> {
    let mut observations = validate(program);
    if observations
        .iter()
        .any(|o| o.severity == ObservationSeverity::Error)
    {
        // Level 1 already has errors — skip Level 2 validation
        return observations;
    }
    observations.extend(level2::validate_level2(program, provider));
    observations
}

// ── Legacy validation model ────────────────────────────────────────────────
// REMOVED in the phase-6 deletion (tasks.md 6.1): the pre-migration
// severity/diagnostic/result vocabulary had zero remaining consumers since
// PR 5 (production validation emits `Observation`).

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use thalos_core::analysis::attribute_value::AttributeValue;
    use thalos_core::analysis::location::Location;
    use thalos_core::analysis::observation::{
        ArtifactRef, ObservationKind, Severity as ObservationSeverity,
    };
    use thalos_core::ids::SemanticProgramId;

    use crate::knowledge::{LoweringError, MockKnowledgeProvider};
    use crate::operation::*;
    use crate::resource::*;

    // ── Helper builders ──────────────────────────────────────────────────

    fn make_pick(origin: &str, object: &str, tool: Option<&str>) -> SemanticOperation {
        SemanticOperation::Pick(PickOp {
            origin: OperationId(origin.to_string()),
            object: ObjectId(object.to_string()),
            tool: tool.map(|t| ToolId(t.to_string())),
        })
    }

    fn make_place(origin: &str, object: &str, destination: &str) -> SemanticOperation {
        SemanticOperation::Place(PlaceOp {
            origin: OperationId(origin.to_string()),
            object: ObjectId(object.to_string()),
            destination: LocationId(destination.to_string()),
            tool: None,
        })
    }

    fn make_wait(origin: &str, duration: Duration) -> SemanticOperation {
        SemanticOperation::Wait(WaitOp {
            origin: OperationId(origin.to_string()),
            duration,
        })
    }

    fn make_home(origin: &str) -> SemanticOperation {
        SemanticOperation::Home(HomeOp {
            origin: OperationId(origin.to_string()),
        })
    }

    fn make_move_to(origin: &str, destination: &str) -> SemanticOperation {
        SemanticOperation::MoveTo(MoveToOp {
            origin: OperationId(origin.to_string()),
            destination: LocationId(destination.to_string()),
            tool: None,
        })
    }

    macro_rules! assert_observation_count {
        ($result:expr, $count:expr) => {
            assert_eq!(
                $result.len(),
                $count,
                "Expected {} observations, got {}: {:?}",
                $count,
                $result.len(),
                $result
            );
        };
    }

    // ── Place without Pick as Observation (spec semantic-validation) ───────

    #[test]
    fn place_without_any_pick_is_a_place_without_pick_observation() {
        // Spec semantic-validation "Validation error as Observation": a Place
        // without a preceding Pick yields an Observation with kind
        // PlaceWithoutPick, severity Error, artifact SemanticProgram, located
        // at the place's origin, and carrying the object_id as typed data.
        let program = SemanticProgram::new(vec![make_place("place-1", "bolt-1", "tray-1")]);
        let observations = validate(&program);
        assert_eq!(observations.len(), 1, "exactly one observation expected");
        assert_eq!(observations[0].kind, ObservationKind::PlaceWithoutPick);
        assert_eq!(observations[0].severity, ObservationSeverity::Error);
        assert_eq!(
            observations[0].artifact,
            ArtifactRef::SemanticProgram(SemanticProgramId("semantic-program".to_string()))
        );
        assert_eq!(
            observations[0].location,
            Location::Operation(OperationId("place-1".to_string()))
        );
        assert_eq!(
            observations[0].attributes["object_id"],
            AttributeValue::Text("bolt-1".to_string())
        );
        assert!(observations[0].causes.is_empty());
        assert!(observations[0].related.is_empty());
    }

    #[test]
    fn place_after_pick_of_different_object_reports_place_origin() {
        let program = SemanticProgram::new(vec![
            make_pick("pick-1", "bolt-1", None),
            make_place("place-2", "bolt-2", "tray-1"),
        ]);
        let observations = validate(&program);
        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].kind, ObservationKind::PlaceWithoutPick);
        assert_eq!(
            observations[0].location,
            Location::Operation(OperationId("place-2".to_string()))
        );
        assert_eq!(
            observations[0].attributes["object_id"],
            AttributeValue::Text("bolt-2".to_string())
        );
    }

    #[test]
    fn validation_observation_carries_no_presentation_text() {
        // I1 (spec semantic-validation "Domain Purity in Validation"): the
        // phenomenon is fully identifiable from kind + attributes; the
        // observation carries no message/help/localized text.
        let program = SemanticProgram::new(vec![make_place("place-1", "bolt-1", "tray-1")]);
        let observations = validate(&program);
        let json = serde_json::to_value(&observations[0]).expect("serialize");
        let obj = json.as_object().expect("object");
        for banned in ["message", "text", "icon", "label", "description", "help"] {
            assert!(
                !obj.contains_key(banned),
                "validation observation must not carry presentation field `{banned}`"
            );
        }
        // The object id survives ONLY as typed attribute data — never embedded
        // in a message string.
        assert_eq!(
            observations[0].attributes["object_id"],
            AttributeValue::Text("bolt-1".to_string())
        );
    }

    #[test]
    fn pick_then_place_of_same_object_valid() {
        let program = SemanticProgram::new(vec![
            make_pick("pick-1", "bolt-1", None),
            make_place("place-2", "bolt-1", "tray-1"),
        ]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    #[test]
    fn pick_then_place_after_other_ops_valid() {
        // Pick and Place don't need to be adjacent — other ops can be between
        let program = SemanticProgram::new(vec![
            make_pick("pick-1", "bolt-1", None),
            make_move_to("move-2", "table"),
            make_wait("wait-3", Duration::from_secs(1)),
            make_place("place-4", "bolt-1", "tray-1"),
        ]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    // ── Wait duration ─────────────────────────────────────────────────────

    #[test]
    fn wait_zero_duration_valid() {
        let program = SemanticProgram::new(vec![make_wait("wait-1", Duration::ZERO)]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    #[test]
    fn wait_positive_duration_valid() {
        let program = SemanticProgram::new(vec![make_wait("wait-2", Duration::from_secs(5))]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    // ── Home parameterless ────────────────────────────────────────────────

    #[test]
    fn home_alone_no_errors() {
        let program = SemanticProgram::new(vec![make_home("home-1")]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    // ── Valid sequences ───────────────────────────────────────────────────

    #[test]
    fn pick_alone_valid() {
        let program = SemanticProgram::new(vec![make_pick("pick-1", "bolt-1", None)]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    #[test]
    fn home_alone_valid() {
        let program = SemanticProgram::new(vec![make_home("home-1")]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    #[test]
    fn empty_program_valid() {
        let program = SemanticProgram::new(vec![]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    #[test]
    fn valid_full_sequence_no_errors() {
        let program = SemanticProgram::new(vec![
            make_pick("op-1", "bolt-1", None),
            make_place("op-2", "bolt-1", "tray-1"),
            make_move_to("op-3", "shelf-a"),
            make_wait("op-4", Duration::from_secs(2)),
            make_home("op-5"),
        ]);
        let observations = validate(&program);
        assert_observation_count!(observations, 0);
    }

    // ── Multiple errors ───────────────────────────────────────────────────

    #[test]
    fn multiple_place_without_pick_all_flagged() {
        let program = SemanticProgram::new(vec![
            make_place("p1", "bolt-1", "tray-1"),
            make_place("p2", "nut-2", "tray-2"),
        ]);
        let observations = validate(&program);
        assert_observation_count!(observations, 2);
        assert!(
            observations
                .iter()
                .all(|o| o.kind == ObservationKind::PlaceWithoutPick)
        );
    }

    // ── validate_with_provider Level 2 gate ───────────────────────────────

    #[test]
    fn level2_is_skipped_when_level1_has_errors() {
        // A Place without a Pick (Level 1 error) gates Level 2: even though the
        // provider would fail to resolve resources, only the Level 1
        // observation is emitted.
        let provider =
            MockKnowledgeProvider::new().with_home_pose(Err(LoweringError::MissingHomePose));
        let program = SemanticProgram::new(vec![make_place("place-1", "bolt-1", "tray-1")]);
        let observations = validate_with_provider(&program, &provider);
        assert_observation_count!(observations, 1);
        assert_eq!(observations[0].kind, ObservationKind::PlaceWithoutPick);
    }

    #[test]
    fn level2_runs_when_level1_is_clean() {
        // A program that passes Level 1 (Pick alone is valid) is still checked
        // against the provider: an unresolvable object surfaces as an
        // UnresolvableReference observation.
        let unknown = ObjectId("unknown".to_string());
        let provider = MockKnowledgeProvider::new().with_grasp_error(
            unknown.clone(),
            LoweringError::KnowledgeProvider("not found".into()),
        );
        let program = SemanticProgram::new(vec![make_pick("pick-1", "unknown", None)]);
        let observations = validate_with_provider(&program, &provider);
        assert_observation_count!(observations, 1);
        assert_eq!(observations[0].kind, ObservationKind::UnresolvableReference);
        assert_eq!(
            observations[0].attributes["object_id"],
            AttributeValue::Text("unknown".to_string())
        );
    }

    // ── Validation is read-only ───────────────────────────────────────────

    #[test]
    fn validation_is_read_only() {
        let ops = vec![
            make_pick("op-1", "bolt-1", None),
            make_place("op-2", "bolt-1", "tray-1"),
            make_home("op-3"),
        ];
        let program = SemanticProgram::new(ops);
        let len_before = program.operations.len();
        let _result = validate(&program);
        assert_eq!(
            program.operations.len(),
            len_before,
            "Validation must not mutate the program"
        );
    }
}
