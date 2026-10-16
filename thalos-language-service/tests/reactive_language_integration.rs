//! Reactive language integration — the `.thls` surface, end to end.
//!
//! Parses reactive `.thls`, resolves the process model, checks the compile
//! boundary (typed signal access, control-flow preservation) and the work
//! specialization, through the crate's public API.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::evaluator::CompileTimeValue;
use thalos_language_service::reactive::{
    ReactiveExpr, ReactiveStatement, SignalRef, SignalSchema, SignalType,
};
use thalos_language_service::reactive_process::{PEvent, compile_process, specialize_work};
use thalos_language_service::resolver::SemanticResolver;

const PALLETIZING: &str = r#"
use station::{ box_sensor as box, pallet }

process palletizing {
    state admitting : Bool = true
    state next_slot : Int = 0

    on rising(box.present) {
        if admitting {
            request place_box(box_id = box.id, slot = next_slot)
        }
    }
    on rising(pallet.full) {
        admitting = false
    }
    cycle {
        request maintain_station()
    }
}

work place_box(box_id : Int, slot : Int) {
    pick(box_id);
}

fn main() {
    start palletizing
}
"#;

const IF_ON_SIGNAL: &str = r#"
use station::{ box_sensor as box }

fn main() {
    if box.present {
        set_output("gripper", true);
    } else {
        set_output("gripper", false);
    }
}
"#;

const SPECIALIZE: &str = r#"
target slot_1 = joints(10deg, 20deg)
target slot_2 = joints(30deg, 40deg)

fn place(slot: Int) {
    if slot == 1 {
        movej(slot_1)
    } else {
        movej(slot_2)
    }
}

work place_box(box_id: Int, slot: Int) {
    place(slot);
}
"#;

fn schema() -> SignalSchema {
    SignalSchema::new()
        .with("box_sensor", "present", SignalType::Bool)
        .with("box_sensor", "id", SignalType::Int)
        .with("pallet", "full", SignalType::Bool)
}

#[test]
fn reactive_program_parses_and_resolves_the_process_model() {
    let ast = parse_source(PALLETIZING).expect("the reactive program must parse");
    let program = compile_process(&ast, &schema()).expect("the process must compile");

    assert_eq!(program.name, "palletizing");
    // `use station::{ box_sensor as box, pallet }` → two node aliases.
    assert_eq!(program.aliases.len(), 2);
    assert_eq!(program.states.len(), 2);
    assert_eq!(program.reactions.len(), 2);
    assert_eq!(program.works.len(), 1);
    assert_eq!(program.works[0].name, "place_box");
    assert_eq!(program.cycle.len(), 1);

    // The event resolves to the REAL node, not the alias.
    match &program.reactions[0].event {
        PEvent::Rising(SignalRef { module, channel }) => {
            assert_eq!(module, "box_sensor");
            assert_eq!(channel, "present");
        }
        other => panic!("expected rising(box.present), got {other:?}"),
    }
}

#[test]
fn a_reaction_that_actuates_is_rejected() {
    let src = r#"
use station::{ box_sensor as box }
process p {
    on rising(box.present) {
        set_output("gripper", true);
    }
}
fn main() { start p }
"#;
    let ast = parse_source(src).expect("parse");
    let err = compile_process(&ast, &schema()).expect_err("actuation in a reaction must fail");
    assert!(
        err.iter().any(|e| e.contains("reaction cannot actuate")),
        "got {err:?}"
    );
}

#[test]
fn an_unknown_node_is_rejected() {
    let src = r#"
use station::{ ghost as g }
process p { on rising(g.x) { request w() } }
work w() {}
fn main() { start p }
"#;
    let ast = parse_source(src).expect("parse");
    let err = compile_process(&ast, &schema()).expect_err("unknown node must fail");
    assert!(
        err.iter().any(|e| e.contains("unknown signal node 'ghost'")),
        "got {err:?}"
    );
}

#[test]
fn compile_reactive_preserves_control_flow_on_a_signal() {
    let ast = parse_source(IF_ON_SIGNAL).expect("parse");
    let reactive = SemanticCompiler::compile_reactive(&ast, &schema())
        .expect("a signal-conditional program must compile reactively");

    assert_eq!(reactive.statements.len(), 1, "one preserved If");
    match &reactive.statements[0] {
        ReactiveStatement::If {
            condition,
            then_branch,
            else_branch,
            ..
        } => {
            assert_eq!(then_branch.len(), 1);
            assert_eq!(else_branch.as_ref().map(Vec::len), Some(1));
            match condition {
                ReactiveExpr::Signal { module, channel, ty } => {
                    assert_eq!(module, "box_sensor");
                    assert_eq!(channel, "present");
                    assert_eq!(*ty, SignalType::Bool);
                }
                other => panic!("expected a signal condition, got {other:?}"),
            }
        }
        other => panic!("expected a preserved If, got {other:?}"),
    }
}

#[test]
fn specialize_work_binds_the_payload_through_the_pipeline() {
    let ast = parse_source(SPECIALIZE).expect("parse");

    let resolve = |slot: i64| {
        let specialized = specialize_work(
            &ast,
            "place_box",
            &[
                ("box_id".to_string(), CompileTimeValue::Int(12)),
                ("slot".to_string(), CompileTimeValue::Int(slot)),
            ],
        )
        .expect("specialize");
        let semantic = SemanticCompiler::compile(&specialized).expect("compile");
        SemanticResolver::resolve(&semantic).expect("resolve")
    };

    let one = resolve(1);
    let two = resolve(2);
    assert_eq!(one.statements.len(), 1);
    assert_ne!(
        one, two,
        "different payloads must yield different resolved programs"
    );

    // A missing argument is an explicit error, never a silent default.
    let err = specialize_work(&ast, "place_box", &[])
        .expect_err("missing args must fail");
    assert!(err.iter().any(|e| e.contains("missing argument")), "got {err:?}");
}
