use serde::{Deserialize, Serialize};
use crate::ast::expression::{Arg, Expr};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Statement {
    Let {
        name: String,
        type_ann: Option<String>,
        value: Expr,
    },
    MoveJ {
        target: Expr,
    },
    MoveL {
        target: Expr,
    },
    MoveC {
        via: Expr,
        target: Expr,
    },
    Wait(Expr),
    SetOutput {
        output: String,
        value: Expr,
    },
    If {
        condition: Expr,
        then_branch: Vec<Statement>,
        else_branch: Option<Vec<Statement>>,
    },
    /// `request name(args)`: emit a work request. Does not admit it; the runtime
    /// arbitrates (see `docs/design/lenguaje-reactivo.md`).
    Request {
        name: String,
        args: Vec<Arg>,
    },
    /// `start name`: start a persistent process. Valid only in the contexts
    /// defined for starting processes (e.g. `main`), never inside a `work`.
    Start {
        process: String,
    },
    Expr(Expr),
}
