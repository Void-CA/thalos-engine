//! Fase 1 conformance: annotation resolution, `Joints<Unknown>`, checker
//! integrity, effects and the validated-program gate.
//!
//! Normative source: `thalos-industrial/docs/lenguaje/sistema-tipos.md`.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::intelligence::analyze_intelligence;
use thalos_language_service::reactive::{SignalSchema, SignalType};
use thalos_language_service::evaluator::CompileTimeValue;
use thalos_language_service::reactive_process::{compile_process, PExpr};
use thalos_language_service::types::Type;

fn compile_ok(source: &str) {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect("must compile");
}

fn compile_err(source: &str) -> Vec<String> {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect_err("must fail")
}

fn assert_contains(errors: &[String], needle: &str) {
    assert!(
        errors.iter().any(|error| error.contains(needle)),
        "expected an error containing {needle:?}, got {errors:?}"
    );
}

fn schema() -> SignalSchema {
    SignalSchema::new().with("sensor", "present", SignalType::Bool)
}

#[test]
fn unannotated_parameter_is_unknown_not_position() {
    let intelligence = analyze_intelligence("fn f(x) {\n    wait(1s)\n}\n", 1);
    let hint = intelligence
        .hints
        .iter()
        .find(|hint| hint.name == "x")
        .expect("parameter hint");
    assert_eq!(hint.ty, Type::Unknown);
}

#[test]
fn unknown_parameter_cannot_be_used_as_a_concrete_target() {
    let errors = compile_err("fn f(x) {\n    movel(x)\n}\nfn main() {}\n");
    assert_contains(&errors, "movel expected a spatial target");
}

#[test]
fn unknown_annotations_are_errors_in_every_position() {
    assert_contains(
        &compile_err("fn main() {\n    let x: Foo = 1\n}\n"),
        "Unknown type annotation 'Foo'",
    );
    assert_contains(
        &compile_err("fn f(x: Foo) {}\nfn main() {}\n"),
        "Unknown type annotation 'Foo'",
    );
    assert_contains(
        &compile_err("fn f() -> Bar {}\nfn main() {}\n"),
        "Unknown type annotation 'Bar'",
    );
}

#[test]
fn const_annotations_are_validated() {
    assert_contains(
        &compile_err("const X: Length = 1\nfn main() {}\n"),
        "const 'X': expected Length, got Int",
    );
    compile_ok("const X: Length = 1m\nfn main() {}\n");
    // Numeric widening applies to const annotations too.
    compile_ok("const N: Float = 1\nfn main() {}\n");
}

#[test]
fn joints_unknown_is_compatible_at_boundaries() {
    compile_ok(
        r#"
fn f(j: Joints) { movej(j) }
fn main() { f(joints(0deg, 1deg)) }
"#,
    );
}

#[test]
fn joints_known_dimension_must_match() {
    let errors = compile_err(
        r#"
fn f(j: Joints<4>) { movej(j) }
fn main() { f(joints(0deg, 1deg)) }
"#,
    );
    assert_contains(&errors, "No matching overload for call 'f'");
}

#[test]
fn joints_dimension_zero_is_rejected() {
    let errors = compile_err("fn f(j: Joints<0>) {}\nfn main() {}\n");
    assert_contains(&errors, "Joints dimension must be greater than or equal to 1");
}

#[test]
fn joints_member_access_requires_a_known_dimension() {
    let errors = compile_err("fn f(j: Joints) {\n    let x = j.j3\n}\nfn main() {}\n");
    assert_contains(&errors, "Joints dimension is unknown");
}

#[test]
fn joints_member_access_checks_the_dimension() {
    let errors = compile_err("fn f(j: Joints<4>) {\n    let x = j.j5\n}\nfn main() {}\n");
    assert_contains(&errors, "Unknown member 'j5' on type Joints<4>");
}

#[test]
fn joints_offset_requires_a_known_dimension() {
    let errors = compile_err(
        "fn f(j: Joints) {\n    movej(j.offset(j3 = 1deg))\n}\nfn main() {}\n",
    );
    assert_contains(
        &errors,
        "offset requires a Joints receiver with a known dimension",
    );
}

#[test]
fn movec_requires_matching_spatial_targets() {
    let errors = compile_err(
        r#"
target A = position([0m, 0m, 0m])
target B = pose(position([0m, 0m, 0m]), euler(0deg, 0deg, 0deg))
fn main() { movec(A, B) }
"#,
    );
    assert_contains(&errors, "movec via and target must have the same type");

    let errors = compile_err(
        r#"
target J = joints(0deg, 0deg)
target A = position([0m, 0m, 0m])
fn main() { movec(J, A) }
"#,
    );
    assert_contains(&errors, "movec expected Position or Pose");
}

#[test]
fn reactive_statements_are_rejected_in_function_bodies() {
    assert_contains(
        &compile_err("fn main() {\n    x = 1\n}\n"),
        "only valid inside a process",
    );
    assert_contains(
        &compile_err("fn main() {\n    request foo()\n}\n"),
        "only valid inside a process",
    );
    assert_contains(
        &compile_err("fn f() {\n    start p\n}\nfn main() {}\n"),
        "start 'p' is only valid inside main",
    );
    assert_contains(
        &compile_err("fn main() {\n    start nope\n}\n"),
        "Unknown process 'nope'",
    );
    compile_ok("fn main() {\n    start p\n}\nprocess p {}\n");
}

#[test]
fn tail_expression_must_fit_the_declared_return_type() {
    assert_contains(
        &compile_err("fn main() {\n    1\n}\n"),
        "returns Int, but its return type is Unit",
    );
    assert_contains(
        &compile_err("fn f() -> Position {\n    [0m, 0m, 0m]\n}\nfn main() {}\n"),
        "expected Position, got Vector3",
    );
}

#[test]
fn diagnostics_render_types_with_display_not_debug() {
    let errors = compile_err("fn main() {\n    let x: Length = 1\n}\n");
    let joined = errors.join(" | ");
    assert!(joined.contains("expected Length, got Int"), "{errors:?}");
    assert!(!joined.contains("{ dimension"), "{errors:?}");
}

#[test]
fn validated_program_gate_never_returns_ir_on_failure() {
    let good = parse_source("fn main() {}\n").expect("parse");
    assert!(SemanticCompiler::compile_validated(&good).is_ok());

    let bad = parse_source("fn main() {\n    let x: Foo = 1\n}\n").expect("parse");
    assert!(SemanticCompiler::compile_validated(&bad).is_err());
}

#[test]
fn error_is_not_exposed_to_tooling() {
    let intelligence = analyze_intelligence("fn f(x: Foo) {}\n", 1);
    assert!(
        intelligence
            .symbols
            .iter()
            .all(|symbol| !symbol.ty.is_error()),
        "symbols must not expose Error: {:?}",
        intelligence.symbols
    );
    assert!(
        intelligence.hints.iter().all(|hint| !hint.ty.is_error()),
        "hints must not expose Error: {:?}",
        intelligence.hints
    );
}

#[test]
fn effects_reject_impure_non_unit_functions() {
    assert_contains(
        &compile_err("fn f() -> Position {\n    wait(1s)\n    position([0m, 0m, 0m])\n}\nfn main() {}\n"),
        "cannot have effects: timing",
    );
    assert_contains(
        &compile_err(
            "fn impure() {\n    wait(1s)\n}\nfn f() -> Position {\n    impure();\n    position([0m, 0m, 0m])\n}\nfn main() {}\n",
        ),
        "cannot have effects: timing",
    );
}

#[test]
fn effects_allow_pure_non_unit_functions_and_unit_effects() {
    compile_ok("fn f() -> Position {\n    position([0m, 0m, 0m])\n}\nfn main() {}\n");
    compile_ok("fn unit_with_wait() {\n    wait(1s)\n}\nfn main() {\n    unit_with_wait()\n}\n");
}

#[test]
fn reactive_state_annotations_and_assignments_are_validated() {
    let valid = r#"
use sensor
fn main() { start p }
process p {
    state armed: Bool = true
    on rising(sensor.present) { armed = false }
}
"#;
    let program = compile_process(&parse_source(valid).expect("parse"), &schema())
        .expect("valid reactive process");
    assert_eq!(program.states[0].ty, Type::Bool);

    assert_contains(
        &compile_process(
            &parse_source(
                r#"
use sensor
fn main() { start p }
process p { state armed: Bool = 1 }
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("must fail"),
        "state 'armed': expected Bool, got Int",
    );

    assert_contains(
        &compile_process(
            &parse_source(
                r#"
use sensor
fn main() { start p }
process p {
    state armed: Bool = true
    on rising(sensor.present) { armed = 1 }
}
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("must fail"),
        "state 'armed' expected Bool, got Int",
    );

    assert_contains(
        &compile_process(
            &parse_source(
                r#"
use sensor
fn main() { start p }
process p { state armed = sensor.present }
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("must fail"),
        "initializer must be a compile-time constant",
    );

    assert_contains(
        &compile_process(
            &parse_source(
                r#"
use sensor
fn main() { start p }
process p { state armed: Foo = true }
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("must fail"),
        "Unknown type annotation 'Foo'",
    );
}

#[test]
fn reactive_calls_respect_inferred_effects() {
    let errors = compile_process(
        &parse_source(
            r#"
use sensor
fn grab() { set_output("gripper", true); }
fn main() { start p }
process p {
    on rising(sensor.present) { grab(); }
}
"#,
        )
        .expect("parse"),
        &schema(),
    )
    .expect_err("physical call must be rejected in on");
    assert_contains(&errors, "physical operation 'grab' is not allowed in on");
}

#[test]
fn signal_reads_are_io_effects_even_through_use_aliases() {
    let schema = SignalSchema::new().with("sensor", "present", SignalType::Float);
    let ast = parse_source(
        r#"
use sensor as s
fn read() -> Float { s.present }
fn main() {}
"#,
    )
    .expect("parse");
    let errors = SemanticCompiler::compile_with_signals(&ast, &schema)
        .expect_err("reading a signal in a pure function must fail");
    assert_contains(&errors, "cannot have effects: io");
}

#[test]
fn member_access_on_a_value_stays_pure() {
    compile_ok("fn f(p: Position) -> Length { p.x }\nfn main() {}\n");
}

#[test]
fn state_numeric_widening_promotes_the_stored_value() {
    let program = compile_process(
        &parse_source(
            r#"
use sensor
fn main() { start p }
process p { state level: Float = 1 }
"#,
        )
        .expect("parse"),
        &schema(),
    )
    .expect("valid state");
    assert_eq!(program.states[0].ty, Type::Float);
    match &program.states[0].init {
        PExpr::Constant(CompileTimeValue::Float(value)) => assert_eq!(*value, 1.0),
        other => panic!("expected a promoted Float constant, got {other:?}"),
    }
}
