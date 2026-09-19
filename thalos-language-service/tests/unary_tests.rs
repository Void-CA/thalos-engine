//! Unary negation (`-expr`): additive inversion expressed as syntax over the
//! value types that actually have an additive inverse.
//!
//! `-10mm` remains a signed literal; `-side` is an explicit unary node. These
//! tests pin the type rules, the constant-folded value, and the motivating
//! program from the language discussion.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::model::MotionTarget;

const MM: f64 = 0.001;

fn compile_ok(source: &str) {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect("must compile");
}

fn compile_err(source: &str) -> Vec<String> {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect_err("must fail")
}

fn position_target(source: &str, name: &str) -> [f64; 3] {
    let ast = parse_source(source).expect("source must parse");
    let program = SemanticCompiler::compile(&ast).expect("must compile");
    let target = program
        .targets
        .iter()
        .find(|t| t.name == name)
        .unwrap_or_else(|| panic!("target '{name}' not found"));
    match &target.value {
        MotionTarget::Position(p) => [p.point.x, p.point.y, p.point.z],
        other => panic!("expected Position target, got {other:?}"),
    }
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn unary_negation_of_a_const_length_offsets_position() {
    let source = r#"
const S = 2mm
target P0 = position([100mm, 200mm, 300mm])
target P1 = offset(P0, x = -S)
fn main() { movel(P1) }
"#;
    let p1 = position_target(source, "P1");
    assert_close(p1[0], 100.0 * MM - 2.0 * MM);
    assert_close(p1[1], 200.0 * MM);
    assert_close(p1[2], 300.0 * MM);
}

#[test]
fn double_negation_restores_the_original_value() {
    let source = r#"
const S = 2mm
target P0 = position([100mm, 200mm, 300mm])
target P1 = offset(P0, x = --S)
fn main() { movel(P1) }
"#;
    let p1 = position_target(source, "P1");
    assert_close(p1[0], 100.0 * MM + 2.0 * MM);
}

#[test]
fn negating_a_vector3_is_component_wise() {
    compile_ok(
        r#"
const V = [1mm, 2mm, 3mm]
const N = -V
fn main() {}
"#,
    );
}

#[test]
fn negated_parameter_in_a_function_body_compiles() {
    // The motivating program: `-side` lowers to `SemanticExpr::Unary` because
    // the operand is only known at resolution time.
    compile_ok(
        r#"
target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)
target ptt = position([2.152, 0.783, 1.882])

fn draw_square(start_point : Position, side : Length) {
    movel(start_point.offset(x = side))
    movel(start_point.offset(y = side))
    movel(start_point.offset(x = -side))
    movel(start_point.offset(y = -side))
}

fn main() {
    movej(jtt)
    movel(ptt)
    movel(ptt.offset(x = -1))
    movel(ptt.offset(y = -0.5))
}
"#,
    );
}

#[test]
fn negating_a_position_is_rejected() {
    let errors =
        compile_err("target P0 = position([1mm, 2mm, 3mm])\nfn main() { movel(-P0) }");
    assert!(
        errors
            .iter()
            .any(|e| e.contains("Cannot negate value of type Position")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn negating_joints_is_rejected() {
    let errors = compile_err("target JTT = joints(1deg, 2deg, 3deg)\nfn main() { movej(-JTT) }");
    assert!(
        errors
            .iter()
            .any(|e| e.contains("Cannot negate value of type Joints")),
        "unexpected errors: {errors:?}"
    );
}
