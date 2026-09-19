//! Fase 2 — dimensional algebra: unified `Number`, unit arithmetic, derived
//! `Speed`/`AngularSpeed`, and the shared value algebra used by the evaluator
//! and the resolver.
//!
//! Normative rules: `docs/language/thalos-dimensional-system.md`.

use thalos_lang::ast::BinaryOp;
use thalos_lang::parse_source;
use thalos_language_service::algebra::binary_value;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::evaluator::CompileTimeValue as V;
use thalos_language_service::model::{MotionTarget, ResolvedStatement};
use thalos_language_service::resolver::SemanticResolver;

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

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-12
}

// ── Value algebra (single source of truth) ────────────────────────────

#[test]
fn value_algebra_chain_length_duration_number() {
    // 100mm / 2s -> Speed(0.05)
    let speed = binary_value(BinaryOp::Div, &V::Length(0.1), &V::Duration(2.0)).unwrap();
    match speed {
        V::Speed(s) => assert!(close(s, 0.05), "got {s}"),
        other => panic!("expected Speed, got {other:?}"),
    }

    // Speed * 4 -> Speed(0.2)
    let scaled = binary_value(BinaryOp::Mul, &speed, &V::Int(4)).unwrap();
    match scaled {
        V::Speed(s) => assert!(close(s, 0.2), "got {s}"),
        other => panic!("expected Speed, got {other:?}"),
    }

    // (100mm / 2s) * 4 * 1s -> Length(0.2)
    let distance = binary_value(BinaryOp::Mul, &scaled, &V::Duration(1.0)).unwrap();
    match distance {
        V::Length(l) => assert!(close(l, 0.2), "got {l}"),
        other => panic!("expected Length, got {other:?}"),
    }

    // 100mm / 20mm -> Number (Float)
    let ratio = binary_value(BinaryOp::Div, &V::Length(0.1), &V::Length(0.02)).unwrap();
    match ratio {
        V::Float(f) => assert!(close(f, 5.0), "got {f}"),
        other => panic!("expected Float, got {other:?}"),
    }
}

#[test]
fn value_algebra_angular_speed() {
    let speed = binary_value(BinaryOp::Div, &V::Angle(1.0), &V::Duration(0.5)).unwrap();
    match speed {
        V::AngularSpeed(s) => assert!(close(s, 2.0), "got {s}"),
        other => panic!("expected AngularSpeed, got {other:?}"),
    }
    let angle = binary_value(BinaryOp::Mul, &speed, &V::Duration(0.25)).unwrap();
    match angle {
        V::Angle(a) => assert!(close(a, 0.5), "got {a}"),
        other => panic!("expected Angle, got {other:?}"),
    }
}

#[test]
fn value_algebra_number_promotion() {
    assert_eq!(
        binary_value(BinaryOp::Add, &V::Int(1), &V::Float(2.5)).unwrap(),
        V::Float(3.5)
    );
    assert_eq!(
        binary_value(BinaryOp::Add, &V::Int(1), &V::Int(2)).unwrap(),
        V::Int(3)
    );
    // Division always yields a Float: no integer division.
    assert_eq!(
        binary_value(BinaryOp::Div, &V::Int(3), &V::Int(2)).unwrap(),
        V::Float(1.5)
    );
}

// ── Type algebra via the compiler ─────────────────────────────────────

#[test]
fn length_over_duration_is_speed() {
    compile_ok("fn main() { let v : Speed = 100mm / 2s }");
}

#[test]
fn speed_times_duration_is_length() {
    compile_ok("fn main() { let d : Length = 100mm / 2s * 1s }");
}

#[test]
fn angle_over_duration_is_angular_speed() {
    compile_ok("fn main() { let w : AngularSpeed = 90deg / 2s }");
}

#[test]
fn length_over_length_is_number() {
    compile_ok("fn main() { let r : Float = 100mm / 20mm }");
}

#[test]
fn number_promotion_compiles() {
    compile_ok("fn main() { let n : Float = 1 + 2.5 }");
}

#[test]
fn int_argument_satisfies_float_parameter() {
    // Numeric widening: quaternion expects Float, Int is promoted.
    compile_ok("fn main() { let q = quaternion(1, 0, 0, 0) }");
}

#[test]
fn adding_length_and_angle_fails_in_checker() {
    let errors = compile_err("fn main() { let x = 100mm + 20deg }");
    assert_error_contains(&errors, "Invalid binary operation");
}

#[test]
fn multiplying_two_lengths_fails() {
    let errors = compile_err("fn main() { let x = 100mm * 20mm }");
    assert_error_contains(&errors, "Invalid binary operation");
}

#[test]
fn multiplying_two_durations_fails() {
    let errors = compile_err("fn main() { let x = 2s * 3s }");
    assert_error_contains(&errors, "Invalid binary operation");
}

#[test]
fn bare_number_does_not_satisfy_length() {
    // Widening is numeric only; it is never a dimensional coercion.
    let errors = compile_err("target P = position([0mm, 0mm, 0mm])\nfn main() { movel(P.offset(x = 1)) }");
    assert_error_contains(&errors, "vector component expected Length");
}

// ── Same semantics across contexts (const / function / parameter) ─────

#[test]
fn dimensional_value_survives_const_to_target() {
    compile_ok(
        r#"
const V = 100mm / 2s
const D = V * 1s
target P = position([D, 0mm, 0mm])
fn main() { movel(P) }
"#,
    );
}

#[test]
fn dimensional_value_in_a_function_body() {
    compile_ok(
        r#"
fn main() {
    let d = 100mm / 2s * 1s
    movel(position([d, 0mm, 0mm]))
}
"#,
    );
}

#[test]
fn wait_with_summed_durations_resolves() {
    let source = r#"
fn pause(t : Duration) {
    wait(t + 1s)
}

fn main() {
    pause(2s);
}
"#;
    let ast = parse_source(source).expect("parse");
    let sem = SemanticCompiler::compile(&ast).expect("compile");
    let resolved = SemanticResolver::resolve(&sem).expect("resolve");

    let seconds = resolved
        .statements
        .iter()
        .find_map(|stmt| match stmt {
            ResolvedStatement::Wait { seconds, .. } => Some(*seconds),
            _ => None,
        })
        .expect("a wait statement");
    assert!(close(seconds, 3.0), "expected 3s, got {seconds}");
}

#[test]
fn speed_times_duration_resolves_through_a_parameter() {
    let source = r#"
target START = position([1m, 0m, 0m])

fn move_for(p : Position, v : Speed, t : Duration) {
    movel(p.offset(x = v * t))
}

fn main() {
    move_for(START, 100mm / 2s, 1s);
}
"#;
    let ast = parse_source(source).expect("parse");
    let sem = SemanticCompiler::compile(&ast).expect("compile");
    let resolved = SemanticResolver::resolve(&sem).expect("resolve");

    let position = resolved
        .statements
        .iter()
        .find_map(|stmt| match stmt {
            ResolvedStatement::Motion(motion) => match &motion.target {
                MotionTarget::Position(p) => Some(p.point.x),
                _ => None,
            },
            _ => None,
        })
        .expect("a spatial motion");
    // 1m + (0.05 m/s * 1s) = 1.05m
    assert!(close(position, 1.05), "expected 1.05, got {position}");
}
