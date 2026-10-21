use serde::{Deserialize, Serialize};
use crate::ast::expression::Expr;
use crate::ast::statement::Statement;
use crate::ast::target::TargetDecl;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Param {
    pub name: String,
    pub type_ann: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConstDecl {
    pub name: String,
    pub type_ann: Option<String>,
    pub value: Expr,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FnDecl {
    pub name: String,
    pub params: Vec<Param>,
    pub return_type: Option<String>,
    pub body: Vec<Statement>,
    pub tail_expr: Option<Box<Expr>>,
}

/// A `use` import tree (see the Industrial language reference, `thalos-lang.md` §11).
///
/// A tree is either a **leaf** (`prefix [as alias]`, no group) or a **group**
/// (`prefix::{ ... }`, with nested trees). `prefix` is the dotted/`::` path
/// segments leading to the name; for a group the leading segments are shared.
///
/// This is purely syntactic: it does not model Rust modules, visibility or
/// re-exports. Each leaf must resolve, against the station's signal interface,
/// to an interconnection node.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UseTree {
    /// Path segments before the leaf name (the group prefix for a group).
    pub prefix: Vec<String>,
    /// The leaf's local alias (`use a::b as c`).
    pub alias: Option<String>,
    /// Nested trees for a group (`prefix::{ ... }`); empty for a leaf.
    pub group: Vec<UseTree>,
}

impl UseTree {
    pub fn leaf(path: Vec<String>, alias: Option<String>) -> Self {
        Self {
            prefix: path,
            alias,
            group: Vec::new(),
        }
    }

    pub fn is_group(&self) -> bool {
        !self.group.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UseDecl {
    /// Simple module path, kept for backward compatibility (`use material_handling`).
    /// Empty when the declaration uses a group.
    pub path: String,
    pub tree: UseTree,
}

/// A `state name: T = init` declaration, inside a `process`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StateDecl {
    pub name: String,
    pub type_ann: Option<String>,
    pub value: Expr,
}

/// The trigger of an `on` reaction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Event {
    /// Rising edge of a signal: `rising(access)`.
    Rising(Expr),
    /// Falling edge of a signal: `falling(access)`.
    Falling(Expr),
    /// Level condition: the reaction body runs while the expression holds.
    Level(Expr),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OnDecl {
    pub event: Event,
    pub body: Vec<Statement>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CycleDecl {
    pub body: Vec<Statement>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ProcessItem {
    State(StateDecl),
    On(OnDecl),
    Cycle(CycleDecl),
}

/// A persistent process declaration (item-level; started with `start`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProcessDecl {
    pub name: String,
    pub items: Vec<ProcessItem>,
}

/// A declared unit of work (item-level; requested with `request`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkDecl {
    pub name: String,
    pub params: Vec<Param>,
    pub body: Vec<Statement>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Item {
    Use(UseDecl),
    Const(ConstDecl),
    Target(TargetDecl),
    Function(FnDecl),
    Process(ProcessDecl),
    Work(WorkDecl),
}
