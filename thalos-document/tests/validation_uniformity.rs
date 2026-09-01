//! Observation-model test — user contract (PR 5, spec C3).
//!
//! Semantic validation (`thalos_engine::semantic`) emits
//! `Vec<thalos_engine::core::analysis::observation::Observation>`, which is validatable
//! through `AnalysisReport::validate()` after aggregation. This test locks in
//! that report contract for the semantic validator.

use thalos_engine::core::analysis::aggregator::{Aggregator, DefaultAggregator};
use thalos_engine::core::analysis::observation::{ArtifactRef, Observation, ObservationKind, Severity};
use thalos_engine::core::analysis::scoring::DefaultScoringPolicy;
use thalos_engine::core::ids::{ObjectId, OperationId, SemanticProgramId};

use thalos_engine::semantic::operation::{PlaceOp, SemanticOperation};
use thalos_engine::semantic::program::SemanticProgram;
use thalos_engine::semantic::validation::validate;

/// A semantic program with a single Place and no preceding Pick — the
/// PlaceWithoutPick phenomenon of the semantic validator.
fn program_with_place_without_pick() -> SemanticProgram {
    SemanticProgram::new(vec![SemanticOperation::Place(PlaceOp {
        origin: OperationId("place-1".to_string()),
        object: ObjectId("bolt-1".to_string()),
        destination: thalos_engine::core::ids::LocationId("tray-1".to_string()),
        tool: None,
    })])
}

#[test]
fn semantic_validator_emits_observation_model() {
    // 1. The semantic validator produces the canonical `Observation` type.
    let semantic_observations: Vec<Observation> = validate(&program_with_place_without_pick());
    assert_eq!(
        semantic_observations.len(),
        1,
        "semantic validator must emit exactly one observation"
    );
    assert_eq!(
        semantic_observations[0].kind,
        ObservationKind::PlaceWithoutPick
    );

    // 2. Report-contract validity: the output aggregates through the canonical
    //    aggregator and passes `AnalysisReport::validate()`.
    let semantic_report = DefaultAggregator::new(DefaultScoringPolicy).aggregate(
        ArtifactRef::SemanticProgram(SemanticProgramId("semantic-program".to_string())),
        semantic_observations,
    );
    assert_eq!(semantic_report.validate(), Ok(()));
    assert_eq!(
        semantic_report.observations[0].kind,
        ObservationKind::PlaceWithoutPick
    );
    assert_eq!(semantic_report.observations[0].severity, Severity::Error);
}
