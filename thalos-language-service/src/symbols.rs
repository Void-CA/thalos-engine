use serde::{Deserialize, Serialize};
use thalos_lang::span::Span;
use crate::types::Type;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SymbolKind {
    Target,
    Const,
    Function,
    Parameter,
    Variable,
    Imported,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Symbol {
    pub name: String,
    pub kind: SymbolKind,
    pub ty: Type,
    pub span: Option<Span>,
}

impl Symbol {
    pub fn new(name: impl Into<String>, kind: SymbolKind, ty: Type, span: Option<Span>) -> Self {
        Self {
            name: name.into(),
            kind,
            ty,
            span,
        }
    }
}
