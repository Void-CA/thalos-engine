//! Fase 3 conformance: canonical PascalCase constructors, named arguments and
//! postfix chaining.
//!
//! Lowercase constructors remain as deprecated aliases; these tests use the
//! canonical surface and verify that the compatibility aliases still compile.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::intelligence::analyze_intelligence;
use thalos_language_service::DiagnosticSeverity;

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
fn canonical_constructors_compile() {
    compile_ok(
        r#"
target A = Position(x = 1m, y = 2m, z = 3m)
target B = Position([1m, 2m, 3m])
target C = Position(vector = [1m, 2m, 3m])
target D = Pose(position = A, orientation = Quaternion(w = 1, x = 0, y = 0, z = 0))
target E = Pose(position = [1m, 2m, 3m], orientation = Quaternion.euler(roll = 0deg, pitch = 0deg, yaw = 90deg))
target J = Joints(j1 = 0deg, j2 = 30deg, j3 = 45deg)
const V = Vector3(x = 1m, y = 2m, z = 3m)
fn main() {
    movej(J)
    movel(D)
    movel(E)
}
"#,
    );
}

#[test]
fn joints_named_arguments_default_omitted_joints() {
    compile_ok(
        r#"
target J = Joints(j1 = 0deg, j3 = 10deg)
fn main() { movej(J) }
"#,
    );
}

#[test]
fn postfix_chaining_reaches_value_members() {
    compile_ok(
        r#"
target P = Position(x = 0m, y = 0m, z = 0m)
fn shifted_x() -> Length { P.offset(x = 1mm).x }
const Q = Quaternion(w = 1, x = 0, y = 0, z = 0)
const POSE = Pose(position = P, orientation = Q)
fn pose_z() -> Length { POSE.position.z }
fn main() {}
"#,
    );
}

#[test]
fn lowercase_aliases_still_compile_during_the_deprecation_window() {
    compile_ok(
        r#"
target A = position([1m, 2m, 3m])
target B = pose(A, euler(0deg, 0deg, 0deg))
target J = joints(0deg, 0deg, 0deg)
fn main() {
    movej(J)
    movel(B)
}
"#,
    );
}

#[test]
fn named_arguments_are_validated_against_the_declared_names() {
    assert_contains(
        &compile_err("target A = Position(x = 1m, y = 2m)\nfn main() {}\n"),
        "Missing argument 'z' for 'Position'",
    );
    assert_contains(
        &compile_err("target A = Position(nope = 1m)\nfn main() {}\n"),
        "Unknown named arguments for 'Position'",
    );
    assert_contains(
        &compile_err("const Q = Quaternion.euler(rx = 0deg, ry = 0deg, rz = 0deg)\nfn main() {}\n"),
        "Unknown named arguments for 'Quaternion.euler'",
    );
}

#[test]
fn named_and_positional_arguments_cannot_be_mixed() {
    assert_contains(
        &compile_err("target A = Position(1m, y = 2m, z = 3m)\nfn main() {}\n"),
        "Position cannot mix named and positional arguments",
    );
}

#[test]
fn lowercase_constructors_emit_a_tooling_warning() {
    let legacy = analyze_intelligence("target P = position([1m, 2m, 3m])\nfn main() {}\n", 1);
    assert!(
        legacy.diagnostics.iter().any(|diagnostic| {
            diagnostic.severity == DiagnosticSeverity::Warning
                && diagnostic.code.as_deref() == Some("THL_LEGACY_CONSTRUCTOR")
        }),
        "expected a legacy-constructor warning: {:?}",
        legacy.diagnostics
    );

    let canonical = analyze_intelligence(
        "target P = Position(x = 1m, y = 2m, z = 3m)\nfn main() {}\n",
        1,
    );
    assert!(
        !canonical.diagnostics.iter().any(|diagnostic| {
            diagnostic.code.as_deref() == Some("THL_LEGACY_CONSTRUCTOR")
        }),
        "canonical constructors must not warn: {:?}",
        canonical.diagnostics
    );
}

#[test]
fn chaining_starts_from_a_constructor_call() {
    compile_ok(
        "fn f() -> Length { Position(x = 0m, y = 0m, z = 0m).offset(x = 1mm).x }\nfn main() {}\n",
    );
}
