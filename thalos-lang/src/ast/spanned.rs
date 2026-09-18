//! Spanned (tooling) mirror of the Thalos AST.
//!
//! This tree exists so the language service can map compiler facts back to
//! source locations without changing the semantic AST ([`crate::ast`]).
//!
//! Architecture (per the tooling-spans plan):
//!
//! ```text
//! source
//!   ↓  parse_source_spanned
//! SpannedProgram ──────────→ intelligence / source locations
//!   ↓  unspan
//! Program (semantic AST) ──→ checker / compiler / resolver / ...
//! ```
//!
//! ## Offset discipline
//!
//! Every [`Span`] in this module is expressed in **character offsets**, matching
//! chumsky's `Simple<char>` spans. The single char → byte conversion happens at
//! the `thalos-language-service` boundary (see `char_range_to_byte_span`); no
//! tooling API below that boundary exposes byte offsets and no consumer above it
//! deals with char offsets.

use serde::{Deserialize, Serialize};

use crate::ast::expression::{Arg, BinaryOp};
use crate::ast::item::{ConstDecl, FnDecl, Item, Param, UseDecl};
use crate::ast::program::Program;
use crate::ast::statement::Statement;
use crate::ast::target::TargetDecl;
use crate::ast::Expr;
use crate::span::Span;
use crate::units::{AngleRadians, DurationSeconds, LengthMeters};

/// A value paired with the source span it was parsed from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Spanned<T> {
    pub node: T,
    pub span: Span,
}

/// Root of the spanned tree. Mirrors [`Program`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpannedProgram {
    pub items: Vec<SpannedItem>,
}

/// Mirrors [`Item`] with source locations.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SpannedItem {
    Use { decl: UseDecl, span: Span },
    Const(SpannedConstDecl),
    Target(SpannedTargetDecl),
    Function(SpannedFnDecl),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpannedConstDecl {
    pub name: String,
    pub name_span: Span,
    pub type_ann: Option<String>,
    pub value: SpannedExpr,
    pub span: Span,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpannedTargetDecl {
    pub name: String,
    pub name_span: Span,
    pub pose: SpannedExpr,
    pub span: Span,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpannedFnDecl {
    pub name: String,
    pub name_span: Span,
    pub params: Vec<Spanned<Param>>,
    pub return_type: Option<String>,
    pub body: Vec<SpannedStatement>,
    pub tail_expr: Option<SpannedExpr>,
    pub span: Span,
}

/// Mirrors [`Statement`] with a source span per statement.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpannedStatement {
    pub kind: SpannedStatementKind,
    pub span: Span,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SpannedStatementKind {
    Let {
        name: String,
        name_span: Span,
        type_ann: Option<String>,
        value: SpannedExpr,
    },
    MoveJ {
        target: SpannedExpr,
    },
    MoveL {
        target: SpannedExpr,
    },
    MoveC {
        via: SpannedExpr,
        target: SpannedExpr,
    },
    Wait(SpannedExpr),
    SetOutput {
        output: String,
        output_span: Span,
        value: SpannedExpr,
    },
    If {
        condition: SpannedExpr,
        then_branch: Vec<SpannedStatement>,
        else_branch: Option<Vec<SpannedStatement>>,
    },
    Expr(SpannedExpr),
}

/// Mirrors [`Expr`] with a source span per expression node.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpannedExpr {
    pub kind: SpannedExprKind,
    pub span: Span,
}

/// A call argument with the source span of its name, when it is named.
///
/// The name span is preserved separately so diagnostics and tooling can point at
/// `j2` in `joints(j2 = 5deg)` rather than at the whole argument.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpannedArg {
    pub name: Option<String>,
    pub name_span: Option<Span>,
    pub value: SpannedExpr,
}

impl SpannedArg {
    pub fn unspan(self) -> Arg {
        Arg {
            name: self.name,
            value: self.value.unspan(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SpannedExprKind {
    Identifier(String),
    Number(f64),
    StringLiteral(String),
    Boolean(bool),
    Length(LengthMeters),
    Angle(AngleRadians),
    Duration(DurationSeconds),
    Vector3([Box<SpannedExpr>; 3]),
    Pose {
        position: Box<SpannedExpr>,
        orientation: Box<SpannedExpr>,
    },
    Call {
        callee: String,
        callee_span: Span,
        args: Vec<SpannedArg>,
    },
    MemberCall {
        object: String,
        method: String,
        args: Vec<SpannedArg>,
    },
    MemberAccess {
        object: String,
        member: String,
    },
    Binary {
        left: Box<SpannedExpr>,
        op: BinaryOp,
        right: Box<SpannedExpr>,
    },
}

impl SpannedProgram {
    /// Convert the spanned tree back into the semantic AST.
    ///
    /// The result is byte-for-byte, node-for-node identical to what
    /// `parse_source` produced before spans existed.
    pub fn unspan(self) -> Program {
        Program {
            items: self.items.into_iter().map(SpannedItem::unspan).collect(),
        }
    }
}

impl SpannedItem {
    pub fn unspan(self) -> Item {
        match self {
            SpannedItem::Use { decl, .. } => Item::Use(decl),
            SpannedItem::Const(c) => Item::Const(ConstDecl {
                name: c.name,
                type_ann: c.type_ann,
                value: c.value.unspan(),
            }),
            SpannedItem::Target(t) => Item::Target(TargetDecl {
                name: t.name,
                pose: t.pose.unspan(),
            }),
            SpannedItem::Function(f) => Item::Function(FnDecl {
                name: f.name,
                params: f.params.into_iter().map(|p| p.node).collect(),
                return_type: f.return_type,
                body: f.body.into_iter().map(SpannedStatement::unspan).collect(),
                tail_expr: f.tail_expr.map(|e| Box::new(e.unspan())),
            }),
        }
    }
}

impl SpannedStatement {
    pub fn unspan(self) -> Statement {
        match self.kind {
            SpannedStatementKind::Let {
                name,
                type_ann,
                value,
                ..
            } => Statement::Let {
                name,
                type_ann,
                value: value.unspan(),
            },
            SpannedStatementKind::MoveJ { target } => Statement::MoveJ {
                target: target.unspan(),
            },
            SpannedStatementKind::MoveL { target } => Statement::MoveL {
                target: target.unspan(),
            },
            SpannedStatementKind::MoveC { via, target } => Statement::MoveC {
                via: via.unspan(),
                target: target.unspan(),
            },
            SpannedStatementKind::Wait(expr) => Statement::Wait(expr.unspan()),
            SpannedStatementKind::SetOutput { output, value, .. } => Statement::SetOutput {
                output,
                value: value.unspan(),
            },
            SpannedStatementKind::If {
                condition,
                then_branch,
                else_branch,
            } => Statement::If {
                condition: condition.unspan(),
                then_branch: then_branch.into_iter().map(SpannedStatement::unspan).collect(),
                else_branch: else_branch.map(|branch| {
                    branch.into_iter().map(SpannedStatement::unspan).collect()
                }),
            },
            SpannedStatementKind::Expr(expr) => Statement::Expr(expr.unspan()),
        }
    }
}

impl SpannedExpr {
    pub fn unspan(self) -> Expr {
        match self.kind {
            SpannedExprKind::Identifier(name) => Expr::Identifier(name),
            SpannedExprKind::Number(n) => Expr::Number(n),
            SpannedExprKind::StringLiteral(s) => Expr::StringLiteral(s),
            SpannedExprKind::Boolean(b) => Expr::Boolean(b),
            SpannedExprKind::Length(v) => Expr::Length(v),
            SpannedExprKind::Angle(v) => Expr::Angle(v),
            SpannedExprKind::Duration(v) => Expr::Duration(v),
            SpannedExprKind::Vector3([x, y, z]) => Expr::Vector3([
                Box::new(x.unspan()),
                Box::new(y.unspan()),
                Box::new(z.unspan()),
            ]),
            SpannedExprKind::Pose {
                position,
                orientation,
            } => Expr::Pose {
                position: Box::new(position.unspan()),
                orientation: Box::new(orientation.unspan()),
            },
            SpannedExprKind::Call { callee, args, .. } => Expr::Call {
                callee,
                args: args.into_iter().map(SpannedArg::unspan).collect(),
            },
            SpannedExprKind::MemberCall {
                object,
                method,
                args,
            } => Expr::MemberCall {
                object,
                method,
                args: args.into_iter().map(SpannedArg::unspan).collect(),
            },
            SpannedExprKind::MemberAccess { object, member } => {
                Expr::MemberAccess { object, member }
            }
            SpannedExprKind::Binary { left, op, right } => Expr::Binary {
                left: Box::new(left.unspan()),
                op,
                right: Box::new(right.unspan()),
            },
        }
    }
}
