use serde::{Deserialize, Serialize};
use crate::ast::expression::Expr;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TargetDecl {
    pub name: String,
    pub pose: Expr,
}
