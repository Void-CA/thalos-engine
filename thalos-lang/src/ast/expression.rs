use serde::{Deserialize, Serialize};
use crate::units::{AngleRadians, DurationSeconds, LengthMeters};

/// A single call argument.
///
/// Named arguments (`j2 = 5deg`, `x = -10mm`) carry a name; positional ones do
/// not. Names are resolved into canonical positional order by the binder before
/// any downstream stage sees the program.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Arg {
    pub name: Option<String>,
    pub value: Expr,
}

impl Arg {
    pub fn positional(value: Expr) -> Self {
        Self { name: None, value }
    }

    pub fn named(name: impl Into<String>, value: Expr) -> Self {
        Self {
            name: Some(name.into()),
            value,
        }
    }

    pub fn is_named(&self) -> bool {
        self.name.is_some()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Expr {
    Identifier(String),
    Number(f64),
    StringLiteral(String),
    Boolean(bool),
    Length(LengthMeters),
    Angle(AngleRadians),
    Duration(DurationSeconds),
    Vector3([Box<Expr>; 3]),
    Pose {
        position: Box<Expr>,
        orientation: Box<Expr>,
    },
    Call {
        callee: String,
        args: Vec<Arg>,
    },
    MemberCall {
        object: String,
        method: String,
        args: Vec<Arg>,
    },
    MemberAccess {
        object: String,
        member: String,
    },
    Binary {
        left: Box<Expr>,
        op: BinaryOp,
        right: Box<Expr>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BinaryOp {
    Add,
    Sub,
    Mul,
    Div,
    Gt,
    Lt,
    Gte,
    Lte,
    Eq,
    Neq,
}
