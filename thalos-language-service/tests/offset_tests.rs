//! End-to-end `offset`: receiver-type-directed delta, result keeps the
//! receiver's type.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::model::MotionTarget;

const DEG: f64 = std::f64::consts::PI / 180.0;

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

fn joints_target(source: &str, name: &str) -> Vec<f64> {
    let ast = parse_source(source).expect("source must parse");
    let program = SemanticCompiler::compile(&ast).expect("must compile");
    let target = program
        .targets
        .iter()
        .find(|t| t.name == name)
        .unwrap_or_else(|| panic!("target '{name}' not found"));
    match &target.value {
        MotionTarget::Joints(config) => config.values.clone(),
        other => panic!("expected Joints target, got {other:?}"),
    }
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn named_position_offset_applies_selected_components() {
    let source = r#"
target P0 = position([100mm, 200mm, 300mm])
target P1 = offset(P0, x = -10mm, z = 5mm)
fn main() { movel(P1) }
"#;
    let p1 = position_target(source, "P1");
    assert_close(p1[0], 0.090);
    assert_close(p1[1], 0.200);
    assert_close(p1[2], 0.305);
}

#[test]
fn positional_position_offset_uses_a_full_vector_delta() {
    let source = r#"
target P0 = position([100mm, 200mm, 300mm])
target P1 = offset(P0, [10mm, 0mm, -5mm])
fn main() { movel(P1) }
"#;
    let p1 = position_target(source, "P1");
    assert_close(p1[0], 0.110);
    assert_close(p1[1], 0.200);
    assert_close(p1[2], 0.295);
}

#[test]
fn named_joint_offset_preserves_dimension_and_is_a_first_class_target() {
    // The canonical case: the offset result is a typed Joints target that can
    // itself be referenced by later instructions.
    let source = r#"
target JTT = joints(10deg, 20deg, 30deg)
target JTT2 = offset(JTT, j2 = 5deg)
fn main() { movej(JTT2) }
"#;
    let values = joints_target(source, "JTT2");
    assert_eq!(values.len(), 3);
    assert_close(values[0], 10.0 * DEG);
    assert_close(values[1], 25.0 * DEG);
    assert_close(values[2], 30.0 * DEG);
}

#[test]
fn positional_joint_offset_requires_the_full_delta() {
    let source = r#"
target JTT = joints(10deg, 20deg, 30deg)
target JTT2 = offset(JTT, 0deg, 5deg, 0deg)
fn main() { movej(JTT2) }
"#;
    let values = joints_target(source, "JTT2");
    assert_close(values[0], 10.0 * DEG);
    assert_close(values[1], 25.0 * DEG);
    assert_close(values[2], 30.0 * DEG);
}

#[test]
fn offset_result_flows_into_movej() {
    compile_ok(
        r#"
target JTT = joints(10deg, 20deg, 30deg)
fn main() { movej(offset(JTT, j2 = 5deg)) }
"#,
    );
}

#[test]
fn unknown_component_on_position_is_rejected() {
    let source =
        "target P0 = position([1mm, 2mm, 3mm])\nfn main() { movel(offset(P0, j2 = 5deg)) }";
    let errors = compile_err(source);
    assert!(
        errors.iter().any(|e| e.contains("Unknown offset component 'j2'")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn wrong_positional_joint_arity_is_rejected() {
    let source =
        "target JTT = joints(10deg, 20deg, 30deg)\nfn main() { movej(offset(JTT, 1deg)) }";
    let errors = compile_err(source);
    assert!(
        errors
            .iter()
            .any(|e| e.contains("expected 3 joint deltas, got 1")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn non_vector_delta_for_position_is_rejected() {
    let source = "target P0 = position([1mm, 2mm, 3mm])\nfn main() { movel(offset(P0, 1mm)) }";
    let errors = compile_err(source);
    assert!(
        errors.iter().any(|e| e.contains("expected Vector3")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn mixing_named_and_positional_delta_is_rejected() {
    let source = "target P0 = position([1mm, 2mm, 3mm])\nfn main() { movel(offset(P0, [1mm, 0mm, 0mm], x = 1mm)) }";
    let errors = compile_err(source);
    assert!(
        errors.iter().any(|e| e.contains("cannot mix named and positional")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn unknown_receiver_reports_only_the_root_cause() {
    let source =
        "target P0 = position([1mm, 2mm, 3mm])\nfn main() { movel(offset(UNKNOWN, x = 1mm)) }";
    let errors = compile_err(source);
    assert!(
        errors.iter().any(|e| e.contains("Unknown identifier 'UNKNOWN'")),
        "unexpected errors: {errors:?}"
    );
    assert!(
        !errors.iter().any(|e| e.contains("offset")),
        "the offset layer must not pile on after an unknown receiver: {errors:?}"
    );
}
