//! Compiler-level deprecation warnings (Fase 1).
//!
//! The editor emits `THL_LEGACY_CONSTRUCTOR` with spans; the compiler now emits
//! the same code as a non-fatal warning so CLI/backend consumers do not depend
//! on the editor to surface deprecations.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;

#[test]
fn legacy_constructors_produce_non_fatal_warnings() {
    let source = r#"
target A = position([1m, 2m, 3m])
target B = pose(A, euler(0deg, 0deg, 0deg))
target J = joints(0deg, 0deg)
fn main() { movej(J) }
"#;
    let ast = parse_source(source).expect("parse");
    let report = SemanticCompiler::compile_with_warnings(&ast).expect("compile");

    let messages: Vec<&str> = report
        .warnings
        .iter()
        .map(|warning| warning.message.as_str())
        .collect();
    assert_eq!(report.warnings.len(), 4, "one warning per legacy call: {messages:?}");
    assert!(
        messages.iter().all(|message| !message.contains("THL_LEGACY")),
        "code lives in the structured field, not the message: {messages:?}"
    );
    assert!(messages.iter().any(|m| m.contains("'position'") && m.contains("'Position'")));
    assert!(messages.iter().any(|m| m.contains("'pose'") && m.contains("'Pose'")));
    assert!(messages.iter().any(|m| m.contains("'euler'") && m.contains("'Quaternion.euler'")));
    assert!(messages.iter().any(|m| m.contains("'joints'") && m.contains("'Joints'")));
    assert!(
        report.warnings.iter().all(|w| w.code == "THL_LEGACY_CONSTRUCTOR"),
        "unexpected warning codes"
    );

    // The legacy entry points still compile and discard warnings.
    assert!(SemanticCompiler::compile(&ast).is_ok());
}

#[test]
fn canonical_constructors_produce_no_warnings() {
    let source = r#"
target A = Position(x = 1m, y = 2m, z = 3m)
target B = Pose(position = A, orientation = Quaternion.euler(roll = 0deg, pitch = 0deg, yaw = 0deg))
target J = Joints(0deg, 0deg)
fn main() { movej(J) }
"#;
    let ast = parse_source(source).expect("parse");
    let report = SemanticCompiler::compile_with_warnings(&ast).expect("compile");
    assert!(
        report.warnings.is_empty(),
        "canonical constructors must not warn: {:?}",
        report.warnings
    );
}

#[test]
fn warnings_are_collected_in_consts_targets_and_function_bodies() {
    let source = r#"
const C = position([0m, 0m, 0m])
fn helper() { wait(1ms) }
target T = position([1m, 1m, 1m])
fn main() {
    let local = joints(0deg, 0deg)
    movej(local)
}
"#;
    let ast = parse_source(source).expect("parse");
    let report = SemanticCompiler::compile_with_warnings(&ast).expect("compile");
    let messages: Vec<&str> = report
        .warnings
        .iter()
        .map(|warning| warning.message.as_str())
        .collect();
    assert_eq!(
        report.warnings.len(),
        3,
        "const + target + function let: {messages:?}"
    );
    assert!(messages.iter().all(|m| m.contains("deprecated")));
}
