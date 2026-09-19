//! Dimensional strictness (Fase 1): no implicit `Number -> dimension`
//! coercion, and geometric constructors demand their dimensional components.
//!
//! Normative source: `docs/language/thalos-dimensional-system.md`.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;

fn compile_ok(source: &str) {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect("must compile");
}

fn compile_err(source: &str) -> Vec<String> {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect_err("must fail")
}

fn assert_error_contains(errors: &[String], needle: &str) {
    assert!(
        errors.iter().any(|e| e.contains(needle)),
        "expected error containing {needle:?}, got {errors:?}"
    );
}

// ── Vector3 = [Length, Length, Length] ────────────────────────────────

#[test]
fn dimensional_position_components_are_accepted() {
    compile_ok(
        r#"
target P = position([1m, 2m, 3m])
fn main() { movel(P) }
"#,
    );
}

#[test]
fn bare_number_position_components_are_rejected() {
    let errors = compile_err("target P = position([1, 2, 3])\nfn main() { movel(P) }");
    assert_error_contains(&errors, "vector component expected Length, found Int");
}

#[test]
fn bare_number_vector_in_a_function_body_is_rejected() {
    let errors = compile_err("fn main() {\n    let v = [1, 2, 3]\n    movel(position(v))\n}");
    assert_error_contains(&errors, "vector component expected Length, found Int");
}

// ── offset deltas ─────────────────────────────────────────────────────

#[test]
fn offset_named_bare_number_is_rejected() {
    let errors =
        compile_err("target P = position([0mm, 0mm, 0mm])\nfn main() { movel(P.offset(x = 1)) }");
    assert_error_contains(&errors, "vector component expected Length, found Int");
}

#[test]
fn offset_named_length_is_accepted() {
    compile_ok("target P = position([0mm, 0mm, 0mm])\nfn main() { movel(P.offset(x = 1mm)) }");
}

#[test]
fn offset_joint_bare_number_is_rejected() {
    let errors = compile_err(
        "target J = joints(0deg, 5deg, 10deg)\nfn main() { movej(J.offset(j2 = 5)) }",
    );
    assert_error_contains(&errors, "offset joint delta expected Angle, found Int");
}

#[test]
fn offset_joint_angle_is_accepted() {
    compile_ok("target J = joints(0deg, 5deg, 10deg)\nfn main() { movej(J.offset(j2 = 5deg)) }");
}

// ── joints = Angle^n (no cartesian vector form) ───────────────────────

#[test]
fn joints_angles_are_accepted() {
    compile_ok("fn main() { movej(joints(0deg, 5deg, 10deg)) }");
}

#[test]
fn joints_bare_numbers_are_rejected() {
    let errors = compile_err("fn main() { movej(joints(1, 2, 3)) }");
    assert_error_contains(&errors, "joints component expected Angle, found Int");
}

#[test]
fn joints_vector_form_is_rejected() {
    let errors = compile_err("fn main() { movej(joints([1m, 2m, 3m])) }");
    assert_error_contains(&errors, "joints component expected Angle, found Vector3");
}
