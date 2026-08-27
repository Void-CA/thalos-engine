//! `SemanticExpert` — advisory program-level reasoning over a
//! `SemanticProgram` (design "B-lite — `SemanticExpert` module").
//!
//! Where `thalos-semantic`'s `validate` checks structural correctness, the
//! expert checks advisory quality: incomplete operations, load handling,
//! redundancy and inefficiency. It observes and evaluates only — the program
//! is never mutated. Rules are data-declared in a flat table (8 rules, frozen)
//! and applied with a single linear scan plus small maps (the `level1.rs`
//! pattern). No rule emits `Severity::Error` — expert findings never gate
//! compilation.

use std::collections::BTreeMap;

use thalos_core::analysis::attribute_value::AttributeValue;
use thalos_core::analysis::location::Location;
use thalos_core::analysis::observation::{
    ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
};
use thalos_core::ids::{OperationId, SemanticProgramId};

use thalos_semantic::operation::SemanticOperation;
use thalos_semantic::program::SemanticProgram;
use thalos_semantic::resource::LocationId;

/// Stable artifact anchor for expert observations (I3) — the SAME placeholder
/// id the semantic validator uses, so observations about one program stay
/// consistent.
const PROGRAM_ARTIFACT_ID: &str = "semantic-program";

fn program_artifact() -> ArtifactRef {
    ArtifactRef::SemanticProgram(SemanticProgramId(PROGRAM_ARTIFACT_ID.to_string()))
}

/// Build a canonical expert observation anchored at the offending operation.
fn observation(
    kind: ObservationKind,
    severity: Severity,
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

fn attr_text(key: &str, value: String) -> BTreeMap<String, AttributeValue> {
    let mut map = BTreeMap::new();
    map.insert(key.to_string(), AttributeValue::Text(value));
    map
}

/// A data-declared semantic rule (8 rules, frozen table — see design).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SemanticRule {
    /// Stable id, e.g. `"S01_pick_without_place"`.
    pub id: &'static str,
    /// The `ObservationKind` this rule emits.
    pub kind: ObservationKind,
    /// Advisory severity — `Warning` or `Info`, NEVER `Error`.
    pub severity: Severity,
    /// Human-readable description of the advisory condition.
    pub description: &'static str,
}

/// The frozen 8-rule table (distinct `ObservationKind` each).
pub const SEMANTIC_RULES: [SemanticRule; 8] = [
    SemanticRule {
        id: "S01_pick_without_place",
        kind: ObservationKind::PickWithoutPlace,
        severity: Severity::Warning,
        description: "Object picked, never released",
    },
    SemanticRule {
        id: "S02_home_before_place",
        kind: ObservationKind::HomeBeforePlace,
        severity: Severity::Warning,
        description: "Gripper carrying load into Home",
    },
    SemanticRule {
        id: "S03_pick_while_holding",
        kind: ObservationKind::PickWhileHolding,
        severity: Severity::Warning,
        description: "Pick of a second object before first Place",
    },
    SemanticRule {
        id: "S04_redundant_move_to",
        kind: ObservationKind::RedundantMoveTo,
        severity: Severity::Info,
        description: "Consecutive MoveTo to same destination",
    },
    SemanticRule {
        id: "S05_re_pick_after_place",
        kind: ObservationKind::RePickAfterPlace,
        severity: Severity::Info,
        description: "Place immediately followed by Pick of same object",
    },
    SemanticRule {
        id: "S06_zero_duration_wait",
        kind: ObservationKind::ZeroDurationWait,
        severity: Severity::Info,
        description: "Wait with zero duration (no-op)",
    },
    SemanticRule {
        id: "S07_missing_final_home",
        kind: ObservationKind::MissingFinalHome,
        severity: Severity::Warning,
        description: "Non-empty program does not end in Home",
    },
    SemanticRule {
        id: "S08_zigzag_move_to",
        kind: ObservationKind::ZigzagMoveTo,
        severity: Severity::Info,
        description: "Destination oscillation A->B->A without Pick/Place",
    },
];

/// Program-level advisory reasoning over a `SemanticProgram`.
pub struct SemanticExpert;

impl SemanticExpert {
    /// Linear scan over the program's operations (level1.rs pattern). Pure
    /// read: never mutates the program.
    pub fn analyze(program: &SemanticProgram) -> Vec<Observation> {
        let mut observations: Vec<Observation> = Vec::new();

        // Objects picked but not yet placed (object id, pick origin) — the
        // source for S01 (never placed) and the held-set for S02/S03.
        let mut pending_picks: Vec<(String, OperationId)> = Vec::new();
        // Most recently picked object (for the held_object_id attribute).
        let mut held: Vec<String> = Vec::new();
        // Destination of the previous MoveTo and the one before it.
        let mut prev_dest: Option<LocationId> = None;
        let mut prev2_dest: Option<LocationId> = None;
        // Whether a Pick/Place occurred since the last MoveTo (breaks S08).
        let mut object_op_since_last_move = false;
        // Most recent Place (object, origin) with no intervening op — S05 gate.
        let mut last_place: Option<(String, OperationId)> = None;

        for op in &program.operations {
            let origin = op_origin(op);
            match op {
                SemanticOperation::Pick(pick) => {
                    let object = pick.object.0.clone();
                    if last_place.as_ref().is_some_and(|(obj, _)| *obj == object) {
                        // S05: Place immediately followed by Pick of the same object.
                        observations.push(observation(
                            ObservationKind::RePickAfterPlace,
                            Severity::Info,
                            origin.clone(),
                            attr_text("object_id", object.clone()),
                        ));
                    }
                    if let Some(last_held) = held.last() {
                        // S03: Pick of a second object before the first is placed.
                        observations.push(observation(
                            ObservationKind::PickWhileHolding,
                            Severity::Warning,
                            origin.clone(),
                            {
                                let mut attrs = attr_text("object_id", object.clone());
                                attrs.insert(
                                    "held_object_id".to_string(),
                                    AttributeValue::Text(last_held.clone()),
                                );
                                attrs
                            },
                        ));
                    }
                    pending_picks.push((object.clone(), origin.clone()));
                    held.push(object);
                    last_place = None;
                    object_op_since_last_move = true;
                }
                SemanticOperation::Place(place) => {
                    let object = place.object.0.clone();
                    held.retain(|held| *held != object);
                    pending_picks.retain(|(pending, _)| *pending != object);
                    last_place = Some((object, origin.clone()));
                    object_op_since_last_move = true;
                }
                SemanticOperation::MoveTo(move_to) => {
                    let dest = move_to.destination.0.clone();
                    if prev_dest.as_ref().is_some_and(|d| d.0 == dest) && !object_op_since_last_move
                    {
                        // S04: consecutive MoveTo to the same destination.
                        observations.push(observation(
                            ObservationKind::RedundantMoveTo,
                            Severity::Info,
                            origin.clone(),
                            attr_text("destination", dest.clone()),
                        ));
                    } else if prev2_dest.as_ref().is_some_and(|d| d.0 == dest)
                        && !object_op_since_last_move
                    {
                        // S08: A -> B -> A without an intervening Pick/Place.
                        observations.push(observation(
                            ObservationKind::ZigzagMoveTo,
                            Severity::Info,
                            origin.clone(),
                            attr_text("destination", dest.clone()),
                        ));
                    }
                    prev2_dest = prev_dest.take();
                    prev_dest = Some(LocationId(dest));
                    last_place = None;
                    object_op_since_last_move = false;
                }
                SemanticOperation::Wait(wait) => {
                    if wait.duration.is_zero() {
                        // S06: zero-duration wait is a no-op.
                        observations.push(observation(
                            ObservationKind::ZeroDurationWait,
                            Severity::Info,
                            origin.clone(),
                            BTreeMap::new(),
                        ));
                    }
                    last_place = None;
                }
                SemanticOperation::Home(_home) => {
                    if let Some(last_held) = held.last() {
                        // S02: carrying a load into the home pose.
                        observations.push(observation(
                            ObservationKind::HomeBeforePlace,
                            Severity::Warning,
                            origin.clone(),
                            attr_text("object_id", last_held.clone()),
                        ));
                    }
                    last_place = None;
                }
            }
        }

        // S01: every object picked and never released.
        for (object, pick_origin) in &pending_picks {
            observations.push(observation(
                ObservationKind::PickWithoutPlace,
                Severity::Warning,
                pick_origin.clone(),
                attr_text("object_id", object.clone()),
            ));
        }

        // S07: a non-empty program must end with Home.
        if let Some(last) = program.operations.last() {
            if !matches!(last, SemanticOperation::Home(_)) {
                observations.push(observation(
                    ObservationKind::MissingFinalHome,
                    Severity::Warning,
                    op_origin(last),
                    BTreeMap::new(),
                ));
            }
        }

        observations
    }

    /// The rule table — count-inspectable and severity-inspectable.
    pub fn rules() -> &'static [SemanticRule] {
        &SEMANTIC_RULES
    }
}

/// The `OperationId` every operation carries for traceability.
fn op_origin(op: &SemanticOperation) -> OperationId {
    match op {
        SemanticOperation::Pick(p) => p.origin.clone(),
        SemanticOperation::Place(p) => p.origin.clone(),
        SemanticOperation::MoveTo(m) => m.origin.clone(),
        SemanticOperation::Wait(w) => w.origin.clone(),
        SemanticOperation::Home(h) => h.origin.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use thalos_core::ids::OperationId;
    use thalos_semantic::resource::ObjectId;

    fn pick(origin: &str, object: &str) -> SemanticOperation {
        SemanticOperation::Pick(thalos_semantic::operation::PickOp {
            origin: OperationId(origin.to_string()),
            object: ObjectId(object.to_string()),
            tool: None,
        })
    }

    fn place(origin: &str, object: &str, destination: &str) -> SemanticOperation {
        SemanticOperation::Place(thalos_semantic::operation::PlaceOp {
            origin: OperationId(origin.to_string()),
            object: ObjectId(object.to_string()),
            destination: LocationId(destination.to_string()),
            tool: None,
        })
    }

    fn move_to(origin: &str, destination: &str) -> SemanticOperation {
        SemanticOperation::MoveTo(thalos_semantic::operation::MoveToOp {
            origin: OperationId(origin.to_string()),
            destination: LocationId(destination.to_string()),
            tool: None,
        })
    }

    fn wait(origin: &str, duration: Duration) -> SemanticOperation {
        SemanticOperation::Wait(thalos_semantic::operation::WaitOp {
            origin: OperationId(origin.to_string()),
            duration,
        })
    }

    fn home(origin: &str) -> SemanticOperation {
        SemanticOperation::Home(thalos_semantic::operation::HomeOp {
            origin: OperationId(origin.to_string()),
        })
    }

    fn program(ops: Vec<SemanticOperation>) -> SemanticProgram {
        SemanticProgram::new(ops)
    }

    fn find(observations: &[Observation], kind: ObservationKind) -> &Observation {
        observations
            .iter()
            .find(|o| o.kind == kind)
            .unwrap_or_else(|| panic!("expected observation of kind {kind:?}"))
    }

    fn op_attr<'a>(observation: &'a Observation, key: &str) -> &'a str {
        match &observation.attributes[key] {
            AttributeValue::Text(text) => text,
            other => panic!("attribute {key} must be Text, got {other:?}"),
        }
    }

    // ── Per-rule fixtures (task 9.1) ────────────────────────────────────

    #[test]
    fn s01_picked_object_never_placed() {
        // Spec "Picked Object Never Placed": Pick(bolt-1) at op-1, no later
        // Place(bolt-1) → PickWithoutPlace at op-1, object_id=bolt-1, Warning.
        let observations = SemanticExpert::analyze(&program(vec![pick("op-1", "bolt-1")]));
        let obs = find(&observations, ObservationKind::PickWithoutPlace);
        assert_eq!(obs.severity, Severity::Warning);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-1".to_string()))
        );
        assert_eq!(op_attr(obs, "object_id"), "bolt-1");
        assert_eq!(
            obs.artifact,
            ArtifactRef::SemanticProgram(SemanticProgramId("semantic-program".to_string()))
        );
    }

    #[test]
    fn s02_home_before_place_of_held_object() {
        // Spec "Home Before Place of Held Object": Pick(bolt-1), MoveTo,
        // Home → HomeBeforePlace at the Home origin, object_id=bolt-1.
        let observations = SemanticExpert::analyze(&program(vec![
            pick("op-1", "bolt-1"),
            move_to("op-2", "shelf-a"),
            home("op-3"),
        ]));
        let obs = find(&observations, ObservationKind::HomeBeforePlace);
        assert_eq!(obs.severity, Severity::Warning);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-3".to_string()))
        );
        assert_eq!(op_attr(obs, "object_id"), "bolt-1");
    }

    #[test]
    fn s03_pick_while_holding_load() {
        // Spec "Second Pick Before First Place": Pick(bolt-1), Pick(nut-2),
        // Place(nut-2) → PickWhileHolding at the second Pick origin, with
        // object_id=nut-2 and held_object_id=bolt-1.
        let observations = SemanticExpert::analyze(&program(vec![
            pick("op-1", "bolt-1"),
            pick("op-2", "nut-2"),
            place("op-3", "nut-2", "tray-1"),
        ]));
        let obs = find(&observations, ObservationKind::PickWhileHolding);
        assert_eq!(obs.severity, Severity::Warning);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-2".to_string()))
        );
        assert_eq!(op_attr(obs, "object_id"), "nut-2");
        assert_eq!(op_attr(obs, "held_object_id"), "bolt-1");
    }

    #[test]
    fn s04_redundant_consecutive_move_to() {
        // Spec "Consecutive MoveTo to Same Destination": MoveTo(shelf-a),
        // MoveTo(shelf-a) → RedundantMoveTo at the second origin, destination.
        let observations = SemanticExpert::analyze(&program(vec![
            move_to("op-1", "shelf-a"),
            move_to("op-2", "shelf-a"),
        ]));
        let obs = find(&observations, ObservationKind::RedundantMoveTo);
        assert_eq!(obs.severity, Severity::Info);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-2".to_string()))
        );
        assert_eq!(op_attr(obs, "destination"), "shelf-a");
    }

    #[test]
    fn s05_re_pick_after_place() {
        // Spec "Place Then Pick of Same Object": Place(bolt-1) immediately
        // followed by Pick(bolt-1) → RePickAfterPlace at the Pick origin.
        let observations = SemanticExpert::analyze(&program(vec![
            place("op-1", "bolt-1", "tray-1"),
            pick("op-2", "bolt-1"),
        ]));
        let obs = find(&observations, ObservationKind::RePickAfterPlace);
        assert_eq!(obs.severity, Severity::Info);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-2".to_string()))
        );
        assert_eq!(op_attr(obs, "object_id"), "bolt-1");
    }

    #[test]
    fn s06_zero_duration_wait() {
        // Spec "Wait with Zero Duration": Wait(0s) → ZeroDurationWait at its
        // origin, Info.
        let observations = SemanticExpert::analyze(&program(vec![wait("op-1", Duration::ZERO)]));
        let obs = find(&observations, ObservationKind::ZeroDurationWait);
        assert_eq!(obs.severity, Severity::Info);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-1".to_string()))
        );
    }

    #[test]
    fn s07_missing_final_home() {
        // Spec "Program Does Not End With Home": final op Place → MissingFinalHome
        // anchored at the final operation's origin, Warning.
        let observations = SemanticExpert::analyze(&program(vec![
            pick("op-1", "bolt-1"),
            move_to("op-2", "tray-1"),
            place("op-3", "bolt-1", "tray-1"),
        ]));
        let obs = find(&observations, ObservationKind::MissingFinalHome);
        assert_eq!(obs.severity, Severity::Warning);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-3".to_string()))
        );
    }

    #[test]
    fn s08_zigzag_destinations() {
        // Spec "Oscillating Destinations": A -> B -> A without Pick/Place →
        // ZigzagMoveTo at the returning MoveTo origin, destination=shelf-a.
        let observations = SemanticExpert::analyze(&program(vec![
            move_to("op-1", "shelf-a"),
            move_to("op-2", "shelf-b"),
            move_to("op-3", "shelf-a"),
        ]));
        let obs = find(&observations, ObservationKind::ZigzagMoveTo);
        assert_eq!(obs.severity, Severity::Info);
        assert_eq!(
            obs.location,
            Location::Operation(OperationId("op-3".to_string()))
        );
        assert_eq!(op_attr(obs, "destination"), "shelf-a");
    }

    // ── Negative fixtures (task 9.2) ────────────────────────────────────

    #[test]
    fn clean_pick_place_home_emits_no_observations() {
        let observations = SemanticExpert::analyze(&program(vec![
            pick("op-1", "bolt-1"),
            move_to("op-2", "tray-1"),
            place("op-3", "bolt-1", "tray-1"),
            home("op-4"),
        ]));
        assert!(
            observations.is_empty(),
            "a clean pick->move->place->home program must be silent, got {observations:?}"
        );
    }

    #[test]
    fn object_transfer_between_moves_is_not_zigzag() {
        // Spec "Object Transfer Between Moves Is Not Zigzag": the returning
        // MoveTo(shelf-a) is preceded by a Pick/Place, so no ZigzagMoveTo —
        // and the full transfer program is silent.
        let observations = SemanticExpert::analyze(&program(vec![
            move_to("op-1", "shelf-a"),
            pick("op-2", "bolt-1"),
            move_to("op-3", "tray-1"),
            place("op-4", "bolt-1", "tray-1"),
            move_to("op-5", "shelf-a"),
            home("op-6"),
        ]));
        assert!(
            observations.is_empty(),
            "a Pick/Place between moves must suppress zigzag, got {observations:?}"
        );
    }

    #[test]
    fn program_unchanged_after_analyze() {
        let ops = vec![
            pick("op-1", "bolt-1"),
            place("op-2", "bolt-1", "tray-1"),
            home("op-3"),
        ];
        let program = program(ops);
        let before = program.clone();
        let _ = SemanticExpert::analyze(&program);
        assert_eq!(program, before, "analyze must not mutate the program");
    }

    // ── Rule base (task 9.3) ────────────────────────────────────────────

    #[test]
    fn rule_base_count_is_between_8_and_12() {
        let rules = SemanticExpert::rules();
        assert!(
            (8..=12).contains(&rules.len()),
            "rule count must be in [8, 12], got {}",
            rules.len()
        );
    }

    #[test]
    fn rule_base_kinds_are_distinct() {
        let rules = SemanticExpert::rules();
        let kinds: std::collections::HashSet<ObservationKind> =
            rules.iter().map(|r| r.kind).collect();
        assert_eq!(
            kinds.len(),
            rules.len(),
            "each rule maps to a distinct kind"
        );
    }

    #[test]
    fn no_rule_emits_error_severity() {
        // Advisory only: expert findings must never gate the 422 compile gate.
        for rule in SemanticExpert::rules() {
            assert_ne!(
                rule.severity,
                Severity::Error,
                "rule `{}` must not emit Error severity",
                rule.id
            );
        }
    }

    #[test]
    fn observations_carry_no_presentation_text() {
        // Spec "Observations Carry No Presentation Text": the wire form has no
        // message/text/label fields and attributes are typed data.
        let observations = SemanticExpert::analyze(&program(vec![pick("op-1", "bolt-1")]));
        let json = serde_json::to_value(&observations[0]).expect("serialize");
        let obj = json.as_object().expect("object");
        for banned in ["message", "text", "icon", "label", "description", "help"] {
            assert!(
                !obj.contains_key(banned),
                "expert observation must not carry presentation field `{banned}`"
            );
        }
        assert_eq!(
            observations[0].attributes["object_id"],
            AttributeValue::Text("bolt-1".to_string())
        );
    }
}
