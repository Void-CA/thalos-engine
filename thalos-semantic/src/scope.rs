use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use crate::symbols::Symbol;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ScopeKind {
    Builtin,
    Global,
    Function,
    Block,
}

#[derive(Debug, Clone, Default)]
pub struct Scope {
    pub kind: ScopeKind,
    pub symbols: HashMap<String, Vec<Symbol>>,
}

impl Scope {
    pub fn new(kind: ScopeKind) -> Self {
        Self {
            kind,
            symbols: HashMap::new(),
        }
    }

    pub fn insert(&mut self, symbol: Symbol) -> Result<(), String> {
        let entries = self.symbols.entry(symbol.name.clone()).or_default();
        // Check for exact duplicate in same scope if not overloaded function
        if !entries.is_empty() && entries.iter().any(|s| s.kind == symbol.kind && s.ty == symbol.ty) {
            return Err(format!("Symbol '{}' already declared in current scope", symbol.name));
        }
        entries.push(symbol);
        Ok(())
    }

    pub fn get(&self, name: &str) -> Option<&Vec<Symbol>> {
        self.symbols.get(name)
    }
}

impl Default for ScopeKind {
    fn default() -> Self {
        ScopeKind::Global
    }
}

#[derive(Debug, Clone)]
pub struct SymbolTable {
    scopes: Vec<Scope>,
}

impl Default for SymbolTable {
    fn default() -> Self {
        Self::new()
    }
}

impl SymbolTable {
    pub fn new() -> Self {
        Self {
            scopes: vec![Scope::new(ScopeKind::Builtin), Scope::new(ScopeKind::Global)],
        }
    }

    pub fn push_scope(&mut self, kind: ScopeKind) {
        self.scopes.push(Scope::new(kind));
    }

    pub fn pop_scope(&mut self) -> Option<Scope> {
        if self.scopes.len() > 2 {
            self.scopes.pop()
        } else {
            None // Protect builtin and global scopes
        }
    }

    pub fn declare(&mut self, symbol: Symbol) -> Result<(), String> {
        if let Some(scope) = self.scopes.last_mut() {
            scope.insert(symbol)
        } else {
            Err("No active scope to declare symbol".to_string())
        }
    }

    pub fn declare_builtin(&mut self, symbol: Symbol) -> Result<(), String> {
        if let Some(builtin_scope) = self.scopes.first_mut() {
            builtin_scope.insert(symbol)
        } else {
            Err("Builtin scope not found".to_string())
        }
    }

    pub fn lookup(&self, name: &str) -> Option<&Vec<Symbol>> {
        for scope in self.scopes.iter().rev() {
            if let Some(symbols) = scope.get(name) {
                return Some(symbols);
            }
        }
        None
    }
}
