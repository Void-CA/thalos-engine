use serde::{Deserialize, Serialize};
use thalos_lang::ast::{BinaryOp, UnaryOp};
use thalos_lang::span::Span;
use crate::evaluator::{CompileTimeValue, Position, Pose};
use crate::types::Type;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CallSite {
    pub function: String,
    pub span: Option<Span>,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Provenance {
    pub source_name: Option<String>,
    pub span: Option<Span>,
    pub call_stack: Vec<CallSite>,
}

impl Provenance {
    pub fn new(source_name: Option<String>, span: Option<Span>) -> Self {
        Self {
            source_name,
            span,
            call_stack: Vec::new(),
        }
    }

    pub fn with_call_stack(mut self, call_stack: Vec<CallSite>) -> Self {
        self.call_stack = call_stack;
        self
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointConfiguration {
    pub values: Vec<f64>,
}

impl JointConfiguration {
    pub fn new(values: impl Into<Vec<f64>>) -> Self {
        Self {
            values: values.into(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MotionTarget {
    Position(Position),
    Pose(Pose),
    Joints(JointConfiguration),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResolvedTarget {
    pub name: String,
    pub value: MotionTarget,
    pub provenance: Provenance,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MotionKind {
    MoveJ,
    MoveL,
    MoveC { via: MotionTarget },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SemanticExpr {
    Constant(CompileTimeValue),
    ConstRef(String),
    LocalRef(String),
    ParameterRef(String),
    TargetRef(String),
    Binary {
        left: Box<SemanticExpr>,
        op: BinaryOp,
        right: Box<SemanticExpr>,
    },
    /// Prefix negation whose operand is only known at resolution time
    /// (`-side` where `side` is a parameter/local). Constant operands fold to
    /// `Constant` during lowering and never reach here.
    Unary {
        op: UnaryOp,
        operand: Box<SemanticExpr>,
    },
    Call {
        function: String,
        args: Vec<SemanticExpr>,
    },
    MemberCall {
        object: Box<SemanticExpr>,
        member: String,
        args: Vec<SemanticExpr>,
    },
    /// Read-only member access on a typed value (`PTT.x`, `JTT.j2`).
    ///
    /// Constant receivers fold to `Constant` before lowering; this variant
    /// carries accesses whose receiver is only known at resolution time
    /// (locals/parameters).
    MemberAccess {
        object: Box<SemanticExpr>,
        member: String,
    },
    ChannelAccess {
        module: String,
        channel: String,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SemanticMotion {
    pub kind: MotionKind,
    pub target: SemanticExpr,
    pub provenance: Provenance,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SemanticStatement {
    Let {
        name: String,
        value: SemanticExpr,
        provenance: Provenance,
    },
    Motion(SemanticMotion),
    Wait {
        duration: SemanticExpr,
        provenance: Provenance,
    },
    SetOutput {
        name: String,
        value: bool,
        provenance: Provenance,
    },
    Call {
        function: String,
        args: Vec<SemanticExpr>,
        provenance: Provenance,
    },
    If {
        condition: SemanticExpr,
        then_branch: Vec<SemanticStatement>,
        else_branch: Option<Vec<SemanticStatement>>,
        provenance: Provenance,
    },
    Expr(SemanticExpr),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SemanticFunction {
    pub name: String,
    pub params: Vec<String>,
    pub return_type: Type,
    pub body: Vec<SemanticStatement>,
    pub tail_expr: Option<SemanticExpr>,
    pub provenance: Provenance,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SemanticProgram {
    pub targets: Vec<ResolvedTarget>,
    pub functions: Vec<SemanticFunction>,
    pub entry_point: String,
}

// ── Concrete Resolved Output Types ──

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResolvedMotion {
    pub kind: MotionKind,
    pub target: MotionTarget,
    pub provenance: Provenance,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ResolvedStatement {
    Motion(ResolvedMotion),
    Wait {
        seconds: f64,
        provenance: Provenance,
    },
    SetOutput {
        name: String,
        value: bool,
        provenance: Provenance,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResolvedProgram {
    pub statements: Vec<ResolvedStatement>,
}
