//! End-to-end named-argument binding: parse → compile → canonical targets.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::model::MotionTarget;

const DEG: f64 = std::f64::consts::PI / 180.0;

fn joint_values(source: &str, target_name: &str) -> Vec<f64> {
    let ast = parse_source(source).expect("source must parse");
    let program = SemanticCompiler::compile(&ast).expect("must compile");
    let target = program
        .targets
        .iter()
        .find(|t| t.name == target_name)
        .unwrap_or_else(|| panic!("target '{target_name}' not found"));
    match &target.value {
        MotionTarget::Joints(config) => config.values.clone(),
        other => panic!("expected Joints target, got {other:?}"),
    }
}

#[test]
fn named_joints_compile_to_canonical_order_with_zero_defaults() {
    let source = "target JTT = joints(j3 = 30deg, j1 = 10deg)\nfn main() { movej(JTT) }";
    let values = joint_values(source, "JTT");
    assert_eq!(values.len(), 3);
    assert!((values[0] - 10.0 * DEG).abs() < 1e-9, "j1 = {}", values[0]);
    assert!(values[1].abs() < 1e-9, "j2 default must be zero");
    assert!((values[2] - 30.0 * DEG).abs() < 1e-9, "j3 = {}", values[2]);
}

#[test]
fn positional_joints_still_work() {
    let source = "target JTT = joints(10deg, 20deg, 0deg)\nfn main() { movej(JTT) }";
    let values = joint_values(source, "JTT");
    assert_eq!(values.len(), 3);
    assert!((values[0] - 10.0 * DEG).abs() < 1e-9);
    assert!((values[1] - 20.0 * DEG).abs() < 1e-9);
    assert!(values[2].abs() < 1e-9);
}

#[test]
fn named_joints_are_usable_inside_function_bodies() {
    let source = "fn main() { movej(joints(j2 = 5deg)) }";
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect("must compile");
}

#[test]
fn duplicate_named_joint_is_a_compile_error() {
    let source = "target JTT = joints(j1 = 10deg, j1 = 20deg)\nfn main() { movej(JTT) }";
    let ast = parse_source(source).expect("source must parse");
    let errors = SemanticCompiler::compile(&ast).expect_err("must fail");
    assert!(
        errors.iter().any(|e| e.contains("Duplicate joint argument 'j1'")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn out_of_range_named_joint_is_a_compile_error() {
    let source = "target JTT = joints(j0 = 10deg)\nfn main() { movej(JTT) }";
    let ast = parse_source(source).expect("source must parse");
    let errors = SemanticCompiler::compile(&ast).expect_err("must fail");
    assert!(
        errors.iter().any(|e| e.contains("Unknown joint argument 'j0'")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn named_arguments_on_unsupported_callees_are_rejected() {
    let source = "target P = position(x = 1mm)\nfn main() { movej(P) }";
    let ast = parse_source(source).expect("source must parse");
    let errors = SemanticCompiler::compile(&ast).expect_err("must fail");
    assert!(
        errors.iter().any(|e| e.contains("Named arguments are not supported for 'position'")),
        "unexpected errors: {errors:?}"
    );
}
