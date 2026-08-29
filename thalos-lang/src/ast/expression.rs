use serde::{Deserialize, Serialize};
use crate::units::{AngleRadians, DurationSeconds, LengthMeters};

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
        args: Vec<Expr>,
    },
    MemberCall {
        object: String,
        method: String,
        args: Vec<Expr>,
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
}
