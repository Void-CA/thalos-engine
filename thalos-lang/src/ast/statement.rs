use serde::{Deserialize, Serialize};
use crate::ast::expression::Expr;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Statement {
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
    Expr(Expr),
}
