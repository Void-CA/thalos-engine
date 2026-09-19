//! `offset` on runtime receivers (parameters/locals) must resolve, not fail as
//! an "Unresolved function call". Regression for the `draw_square` program.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;
use thalos_language_service::model::{MotionTarget, ResolvedProgram, ResolvedStatement};
use thalos_language_service::resolver::SemanticResolver;

const MM: f64 = 0.001;

fn resolve_ok(source: &str) -> ResolvedProgram {
    let ast = parse_source(source).expect("source must parse");
    let sem = SemanticCompiler::compile(&ast).expect("must compile");
    SemanticResolver::resolve(&sem).expect("must resolve")
}

fn position_targets(resolved: &ResolvedProgram) -> Vec<[f64; 3]> {
    resolved
        .statements
        .iter()
        .filter_map(|stmt| match stmt {
            ResolvedStatement::Motion(motion) => match &motion.target {
                MotionTarget::Position(p) => Some([p.point.x, p.point.y, p.point.z]),
                _ => None,
            },
            _ => None,
        })
        .collect()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn offset_over_locals_in_a_called_function_resolves() {
    let source = r#"
target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)
target ptt = position([2.152m, 0.783m, 1.882m])

fn draw_square(start_point : Position, side : Length) {
    let p1 = start_point.offset(x = side);
    let p2 = p1.offset(y = side);
    let p3 = p2.offset(x = -side);

    movel(start_point);
    movel(p1);
    movel(p2);
    movel(p3);
    movel(start_point);
}

fn main() {
    movej(jtt);
    draw_square(ptt, 0.5m);
}
"#;

    let resolved = resolve_ok(source);
    // draw_square contributes 5 spatial moves (the movej is a Joints target).
    let positions = position_targets(&resolved);
    assert_eq!(positions.len(), 5);

    let expect = [
        [2.152, 0.783, 1.882],
        [2.652, 0.783, 1.882],
        [2.652, 1.283, 1.882],
        [2.152, 1.283, 1.882],
        [2.152, 0.783, 1.882],
    ];
    for (got, want) in positions.iter().zip(expect.iter()) {
        for axis in 0..3 {
            assert_close(got[axis], want[axis]);
        }
    }
}

#[test]
fn offset_with_positional_vector_delta_resolves_on_a_parameter() {
    let source = r#"
target START = position([100mm, 100mm, 0mm])

fn shift(p : Position) {
    movel(p.offset([10mm, 20mm, 0mm]));
}

fn main() {
    shift(START);
}
"#;

    let resolved = resolve_ok(source);
    let positions = position_targets(&resolved);
    assert_eq!(positions.len(), 1);
    assert_close(positions[0][0], 110.0 * MM);
    assert_close(positions[0][1], 120.0 * MM);
    assert_close(positions[0][2], 0.0);
}
