//! Fase 2 conformance: constructors and members are driven by the registry.
//!
//! These tests exercise the user-visible consequences of the centralized
//! `ConstructorSpec`/`MemberSpec` dispatch, not the registry internals (those
//! live in `registry.rs`).

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::intelligence::analyze_intelligence;

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

#[test]
fn position_requires_a_vector3() {
    assert_contains(
        &compile_err("target P = position(1m)\nfn main() {}\n"),
        "No matching overload for constructor 'position'",
    );
    compile_ok("target P = position([1m, 2m, 3m])\nfn main() {}\n");
}

#[test]
fn pose_dispatches_both_declared_overloads() {
    compile_ok(
        r#"
target A = pose([1m, 2m, 3m], euler(0deg, 0deg, 0deg))
target B = pose(position([1m, 2m, 3m]), euler(0deg, 0deg, 0deg))
fn main() {}
"#,
    );
}

#[test]
fn euler_requires_three_angles() {
    assert_contains(
        &compile_err("target Q = euler(0, 0, 0)\nfn main() {}\n"),
        "No matching overload for constructor 'euler'",
    );
}

#[test]
fn quaternion_accepts_integer_literals_through_numeric_widening() {
    compile_ok("const Q = quaternion(1, 0, 0, 0)\nfn main() {}\n");
}

#[test]
fn joints_requires_at_least_one_angle() {
    assert_contains(
        &compile_err("fn main() {\n    movej(joints())\n}\n"),
        "joints requires at least one Angle",
    );
}

#[test]
fn offset_is_a_receiver_method() {
    compile_ok(
        "target P = position([0m, 0m, 0m])\nfn main() {\n    movel(P.offset(x = 1mm))\n}\n",
    );
}

#[test]
fn variadic_and_receiver_constructors_are_not_function_signatures() {
    let intelligence = analyze_intelligence("fn main() {}\n", 1);
    let names: Vec<&str> = intelligence
        .signatures
        .iter()
        .map(|signature| signature.name.as_str())
        .collect();
    assert!(names.contains(&"position"), "fixed constructors are builtins");
    assert!(!names.contains(&"joints"), "variadic is registry-only");
    assert!(!names.contains(&"offset"), "receiver method is registry-only");
}
