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
pub const DEFAULT_PROGRAM: &str = r#"
const APPROACH_HEIGHT = [0mm, 0mm, 150mm]

target home = joints(0deg, 0deg, 0deg, 0deg, 0deg, 0deg)

target pick = position([420mm, 180mm, 80mm])

fn above(p: Position) -> Position {
    p + APPROACH_HEIGHT
}

fn main() {
    movej(home)
    movej(pick)
    movel(above(pick))
    wait(200ms)
    movel(above(pick))
    movej(home)
}
"#;
