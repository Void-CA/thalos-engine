use serde::{Deserialize, Serialize};
use crate::ast::expression::Expr;

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
    Expr(Expr),
}
