pub mod ast;
pub mod parser;
pub mod span;
pub mod units;

pub use ast::*;
pub use parser::parse_source;
pub use span::Span;
pub use units::*;

/// Programa inicial mostrado a todo proyecto nuevo en el editor.
///
/// Fuente única de verdad del starter en el lenguaje oficial `.thls`.
/// El editor debe importar esta constante (`thalos_engine::lang::DEFAULT_PROGRAM`)
/// en lugar de hardcodear su propia copia.
pub const DEFAULT_PROGRAM: &str = r#"const CLEARANCE = [0mm, 0mm, 150mm]

target PARK = joints(0deg, -30deg, -25deg, 0deg)
target PICK = position([1320mm, 140mm, 80mm])

fn main() {
    movej(PARK)
    movel(PICK)
    wait(500ms)
    movej(PARK)
}
"#;
