//! Canonical observation model — the shared analysis vocabulary (spec I1-I3).
//!
//! Every analyzer in Thalos emits [`Observation`]s: machine-readable,
//! artifact-anchored facts devoid of presentation. This is the single
//! observation language; the three legacy analysis vocabularies were migrated
//! onto it and removed (see the `analysis-model` change).
//!
//! # Invariants (from the specification)
//!
//! - **Facts, not presentation (I1)**: an observation has no message, icon, or
//!   UI directive. Presentation belongs to renderers.
//! - **Machine-readable identification (I2)**: `kind` + `artifact` + `location`
//!   identify the phenomenon without text parsing.
//! - **Artifact anchoring (I3)**: `artifact` is a required, non-optional
//!   [`ArtifactRef`]; a floating observation cannot be constructed.
//! - **Causal traceability (I4)**: `causes`/`related` reference other
//!   [`ObservationId`]s; cycles and dangling references are rejected at report
//!   level by `AnalysisReport::validate()` (Phase 1b).
//!
//! # Extensibility
//!
//! [`ObservationKind`], [`Severity`] and [`ArtifactRef`] are
//! `#[non_exhaustive]` so new phenomena can be added without breaking
//! downstream exhaustive matches — and without degrading machine-readability
//! through catch-all variants.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::analysis::attribute_value::AttributeValue;
use crate::analysis::location::Location;
use crate::ids::{
    ExecutionSessionId, MotionPlanId, RobotId, SceneId, SemanticProgramId, TaskDocumentId,
};

/// Stable identity of an observation within a report (closed decision:
/// a simple counter newtype over `u32`, NOT a UUID — no persistence or
/// cross-process sync exists that would justify one).
///
/// Assigned by the aggregator during report construction so that merging
/// observations from independent analyzers never collides (spec I8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ObservationId(pub u32);

/// The phenomenon an observation reports (spec I2: a phenomenon, never a
/// display classification).
///
/// `#[non_exhaustive]`: new phenomena are added without breaking consumers;
/// matches must include a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ObservationKind {
    /// The configuration is at or near a kinematic singularity.
    NearSingularity,
    /// A requested target pose is unreachable by the serial chain.
    UnreachableTarget,
    /// Residual tracking/kinematic error remains after solving.
    ResidualError,
    /// A joint position/velocity/effort limit was violated.
    JointLimitViolation,
    /// The plan brings the robot close to a collision.
    CollisionRisk,
    /// Executed motion deviates from the planned trajectory.
    RuntimeDeviation,
    /// Communication or execution latency exceeded its threshold.
    LatencySpike,
    /// Tracking error exceeded its threshold during execution.
    TrackingError,
    /// A `Place` exists without a preceding `Pick` (semantic validation).
    PlaceWithoutPick,
    /// A reference in the semantic program cannot be resolved.
    UnresolvableReference,
    /// A `Path` resource in a task document contains no point references
    /// (document validation, PR 5 vocabulary).
    EmptyPath,
    /// Average manipulability (Yoshikawa) over the trajectory is below
    /// the configured threshold (plan-level phenomenon, PR 3 vocabulary).
    LowManipulability,
    /// A full kinematic singularity: the Jacobian is (near-)rank-deficient and
    /// the trajectory cannot be executed at this configuration (PR 3
    /// vocabulary; distinct from [`ObservationKind::NearSingularity`] which is
    /// the Warning-grade precursor).
    Singularity,
    /// A waypoint is dangerously close to an obstacle without colliding —
    /// the Warning-grade precursor of [`ObservationKind::CollisionRisk`]
    /// (PR 3 vocabulary).
    CollisionNear,
    /// A trajectory constraint (velocity, acceleration, orientation, …) was
    /// violated — distinct from a joint-level [`ObservationKind::JointLimitViolation`]
    /// (PR 3 vocabulary).
    ConstraintViolation,
    /// A transient peak of the tracking error during execution — distinct from
    /// the sustained [`ObservationKind::TrackingError`] (PR 4 vocabulary;
    /// mirrors the legacy tracking-spike detection).
    TrackingSpike,
    /// A single joint deviated from its planned position beyond threshold during
    /// execution (PR 4 vocabulary; mirrors the legacy joint-deviation detection).
    JointDeviation,
    /// A single joint's velocity deviated beyond threshold during execution
    /// (PR 4 vocabulary; mirrors the legacy velocity-deviation detection).
    VelocityDeviation,
    /// An object was picked and never released by any later `Place`
    /// (semantic expert, B-lite: incomplete operation).
    PickWithoutPlace,
    /// A `Home` executed while the gripper still holds a picked object
    /// (semantic expert, B-lite: load carried into the home pose).
    HomeBeforePlace,
    /// A `Pick` of a second object occurred before the first was placed
    /// (semantic expert, B-lite: double-grasp risk).
    PickWhileHolding,
    /// Two consecutive `MoveTo` operations target the same destination
    /// (semantic expert, B-lite: redundancy).
    RedundantMoveTo,
    /// A `Place` is immediately followed by a `Pick` of the same object
    /// (semantic expert, B-lite: possible re-grasp loop).
    RePickAfterPlace,
    /// A `Wait` has a zero duration (semantic expert, B-lite: no-op).
    ZeroDurationWait,
    /// A non-empty program does not end with a `Home` operation
    /// (semantic expert, B-lite: never returns to the home pose).
    MissingFinalHome,
    /// `MoveTo` destinations oscillate A → B → A without an intervening
    /// `Pick`/`Place` (semantic expert, B-lite: inefficiency).
    ZigzagMoveTo,
}

/// Severity of an observation. Machine-readable; renderers map it to
/// presentation styles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Severity {
    /// Informational — no impact on the quality index.
    Info,
    /// Notable deviation worth attention.
    Warning,
    /// The artifact is invalid or unsafe.
    Error,
}

/// Typed reference to the artifact an observation belongs to (spec I3).
///
/// Each variant carries the artifact's typed identity. `#[non_exhaustive]`:
/// new artifact kinds can be added without breaking consumers.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ArtifactRef {
    /// The robot model being analyzed.
    Robot(RobotId),
    /// A scene being analyzed.
    Scene(SceneId),
    /// A semantic program under validation.
    SemanticProgram(SemanticProgramId),
    /// A task document under validation.
    TaskDocument(TaskDocumentId),
    /// A motion plan produced by planning.
    MotionPlan(MotionPlanId),
    /// An execution session recorded at runtime.
    ExecutionSession(ExecutionSessionId),
}

/// A single machine-readable fact produced by an analyzer.
///
/// # Invariants
///
/// - No text/presentation field (I1) — `kind` + `attributes` describe the
///   phenomenon fully.
/// - `kind` + `artifact` + `location` identify the phenomenon (I2).
/// - `artifact` is always set (I3).
/// - `causes`/`related` reference other observations; graph validity is
///   enforced at report level (I4).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Observation {
    /// Stable identity within the report, assigned by the aggregator.
    pub id: ObservationId,
    /// The phenomenon observed.
    pub kind: ObservationKind,
    /// Severity of the phenomenon.
    pub severity: Severity,
    /// The artifact this observation belongs to (required, I3).
    pub artifact: ArtifactRef,
    /// Where in the artifact the phenomenon is anchored (I2).
    pub location: Location,
    /// Typed, machine-readable attributes (D5). Keys are stable strings.
    pub attributes: BTreeMap<String, AttributeValue>,
    /// Observation ids this observation is caused by (I4, acyclic).
    pub causes: Vec<ObservationId>,
    /// Observation ids related to this one, without causal direction.
    pub related: Vec<ObservationId>,
}

#[cfg(test)]
mod tests {
    use super::{ArtifactRef, Observation, ObservationId, ObservationKind, Severity};
    use crate::analysis::attribute_value::AttributeValue;
    use crate::analysis::location::Location;
    use crate::ids::MotionPlanId;
    use serde_json::json;

    fn observation(kind: ObservationKind) -> Observation {
        Observation {
            id: ObservationId(1),
            kind,
            severity: Severity::Error,
            artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
            location: Location::Waypoint(5),
            attributes: Default::default(),
            causes: Vec::new(),
            related: Vec::new(),
        }
    }

    #[test]
    fn no_presentation_fields_in_serialized_observation() {
        // I1: observations carry facts, never display data.
        let json =
            serde_json::to_value(observation(ObservationKind::NearSingularity)).expect("serialize");
        let obj = json.as_object().expect("object");
        for banned in ["message", "text", "icon", "label", "description"] {
            assert!(
                !obj.contains_key(banned),
                "observation must not carry presentation field `{banned}`"
            );
        }
    }

    #[test]
    fn kind_artifact_location_identify_phenomenon() {
        // I2: the phenomenon is fully identifiable from machine-readable fields.
        let json =
            serde_json::to_value(observation(ObservationKind::NearSingularity)).expect("serialize");
        assert_eq!(json["kind"], json!("NearSingularity"));
        assert_eq!(json["severity"], json!("Error"));
        assert_eq!(json["artifact"], json!({"MotionPlan": "mp-1"}));
        assert_eq!(json["location"], json!({"Waypoint": 5}));
    }

    #[test]
    fn artifact_is_required_and_anchored() {
        // I3: every observation belongs to exactly one artifact. The `artifact`
        // field is a non-optional `ArtifactRef`, so construction without one
        // cannot compile; here we prove the anchor round-trips.
        let json =
            serde_json::to_value(observation(ObservationKind::ResidualError)).expect("serialize");
        assert_eq!(json["artifact"], json!({"MotionPlan": "mp-1"}));
    }

    #[test]
    fn observation_id_is_transparent_counter() {
        let id = ObservationId(7);
        assert_eq!(serde_json::to_value(id).expect("serialize"), json!(7));
        let back: ObservationId = serde_json::from_value(json!(7)).expect("deserialize");
        assert_eq!(back, id);
    }

    #[test]
    fn observation_kind_has_all_eighteen_phenomena_distinct() {
        let kinds = vec![
            ObservationKind::NearSingularity,
            ObservationKind::UnreachableTarget,
            ObservationKind::ResidualError,
            ObservationKind::JointLimitViolation,
            ObservationKind::CollisionRisk,
            ObservationKind::RuntimeDeviation,
            ObservationKind::LatencySpike,
            ObservationKind::TrackingError,
            ObservationKind::PlaceWithoutPick,
            ObservationKind::UnresolvableReference,
            // PR 5 vocabulary: document validation migrated from the legacy
            // document-validation vocabulary.
            ObservationKind::EmptyPath,
            // PR 3 vocabulary: plan-level phenomena migrated from the legacy
            // trajectory-analysis vocabulary.
            ObservationKind::LowManipulability,
            ObservationKind::Singularity,
            ObservationKind::CollisionNear,
            ObservationKind::ConstraintViolation,
            // PR 4 vocabulary: runtime phenomena migrated from ExecutionAnalyzer.
            ObservationKind::TrackingSpike,
            ObservationKind::JointDeviation,
            ObservationKind::VelocityDeviation,
        ];
        for (i, a) in kinds.iter().enumerate() {
            for (j, b) in kinds.iter().enumerate() {
                if i != j {
                    assert_ne!(a, b, "kinds {i} and {j} must be distinct");
                }
            }
        }
    }

    #[test]
    fn severity_round_trip() {
        for sev in [Severity::Info, Severity::Warning, Severity::Error] {
            let json = serde_json::to_string(&sev).expect("serialize");
            let back: Severity = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(back, sev);
        }
    }

    #[test]
    fn artifact_ref_round_trip_all_variants() {
        use crate::ids::{ExecutionSessionId, RobotId, SceneId, SemanticProgramId, TaskDocumentId};
        let refs = vec![
            ArtifactRef::Robot(RobotId("r1".to_string())),
            ArtifactRef::Scene(SceneId("s1".to_string())),
            ArtifactRef::SemanticProgram(SemanticProgramId("sp1".to_string())),
            ArtifactRef::TaskDocument(TaskDocumentId("td1".to_string())),
            ArtifactRef::MotionPlan(MotionPlanId("mp1".to_string())),
            ArtifactRef::ExecutionSession(ExecutionSessionId("es1".to_string())),
        ];
        for artifact in refs {
            let json = serde_json::to_string(&artifact).expect("serialize");
            let back: ArtifactRef = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(back, artifact);
        }
    }

    #[test]
    fn causes_and_related_carry_observation_ids() {
        let mut obs = observation(ObservationKind::TrackingError);
        obs.causes = vec![ObservationId(1), ObservationId(2)];
        obs.related = vec![ObservationId(3)];
        let json = serde_json::to_value(obs).expect("serialize");
        assert_eq!(json["causes"], json!([1, 2]));
        assert_eq!(json["related"], json!([3]));
    }

    #[test]
    fn attributes_are_typed_and_sorted() {
        let mut obs = observation(ObservationKind::TrackingError);
        obs.attributes
            .insert("value".to_string(), AttributeValue::Number(0.07));
        obs.attributes
            .insert("threshold".to_string(), AttributeValue::Number(0.05));
        let json = serde_json::to_value(obs).expect("serialize");
        // Values keep their typed shape on the wire (D5): externally tagged,
        // so a consumer sees `{"Number": 0.07}` — never a string.
        assert_eq!(json["attributes"]["value"], json!({"Number": 0.07}));
        assert_eq!(json["attributes"]["threshold"], json!({"Number": 0.05}));
    }
}
