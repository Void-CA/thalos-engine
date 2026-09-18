//! `.offset(...)` is pure syntax sugar for `offset(receiver, ...)`: it must
//! produce the exact same semantic result.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::model::MotionTarget;

const DEG: f64 = std::f64::consts::PI / 180.0;

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

fn compile_err(source: &str) -> Vec<String> {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect_err("must fail")
}

#[test]
fn member_offset_and_free_offset_are_equivalent_for_position() {
    let free = "target P0 = position([100mm, 200mm, 300mm])\ntarget P1 = offset(P0, x = -10mm, z = 5mm)\nfn main() { movel(P1) }";
    let member = "target P0 = position([100mm, 200mm, 300mm])\ntarget P1 = P0.offset(x = -10mm, z = 5mm)\nfn main() { movel(P1) }";

    let a = position_target(free, "P1");
    let b = position_target(member, "P1");
    for (x, y) in a.iter().zip(b.iter()) {
        assert!((x - y).abs() < 1e-12, "free {a:?} != member {b:?}");
    }
    // And the value is actually the offset one.
    assert!((b[0] - 0.090).abs() < 1e-9);
    assert!((b[2] - 0.305).abs() < 1e-9);
}

#[test]
fn member_offset_and_free_offset_are_equivalent_for_joints() {
    let free = "target JTT = joints(10deg, 20deg, 30deg)\ntarget JTT2 = offset(JTT, j2 = 5deg)\nfn main() { movej(JTT2) }";
    let member = "target JTT = joints(10deg, 20deg, 30deg)\ntarget JTT2 = JTT.offset(j2 = 5deg)\nfn main() { movej(JTT2) }";

    let a = joints_target(free, "JTT2");
    let b = joints_target(member, "JTT2");
    assert_eq!(a.len(), 3);
    assert_eq!(b.len(), 3);
    for (x, y) in a.iter().zip(b.iter()) {
        assert!((x - y).abs() < 1e-12, "free {a:?} != member {b:?}");
    }
    assert!((b[1] - 25.0 * DEG).abs() < 1e-9);
}

#[test]
fn member_offset_accepts_a_positional_vector_delta() {
    let source = "target P0 = position([100mm, 200mm, 300mm])\ntarget P1 = P0.offset([10mm, 0mm, 0mm])\nfn main() { movel(P1) }";
    let p1 = position_target(source, "P1");
    assert!((p1[0] - 0.110).abs() < 1e-9);
    assert!((p1[1] - 0.200).abs() < 1e-9);
}

#[test]
fn member_offset_in_a_function_body_flows_into_movej() {
    let source = "target JTT = joints(10deg, 20deg, 30deg)\nfn main() { movej(JTT.offset(j2 = 5deg)) }";
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect("must compile");
}

#[test]
fn unknown_member_operation_is_rejected() {
    let source = "target P0 = position([1mm, 2mm, 3mm])\nfn main() { movel(P0.foo(x = 1mm)) }";
    let errors = compile_err(source);
    assert!(
        errors.iter().any(|e| e.contains("Unknown member operation 'foo'")),
        "unexpected errors: {errors:?}"
    );
}
