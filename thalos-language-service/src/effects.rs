//! Declarative effect model for statements and functions.
//!
//! Purity is not a list of per-statement exceptions: every statement declares a
//! set of [`Effect`]s, and a function's effects are the union of its body's
//! effects (transitively through user-function calls). Contexts then decide which
//! effects they allow.

use std::collections::{BTreeSet, HashMap};

use serde::{Deserialize, Serialize};

use thalos_lang::ast::{Expr, Item, Program, Statement, UseTree};

use crate::reactive::SignalSchema;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Effect {
    /// No observable effect (the empty/default set).
    Pure,
    /// Assignment to process state.
    StateMutation,
    /// `wait`.
    Timing,
    /// `movej` / `movel` / `movec`.
    Motion,
    /// `set_output` and signal reads.
    Io,
    /// `request` / `start`.
    ProcessControl,
}

/// A set of effects. An empty set (or `{Pure}`) means the statement/function is
/// pure; concrete effects remove the `Pure` marker.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectSet(BTreeSet<Effect>);

impl EffectSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn pure() -> Self {
        let mut set = Self::default();
        set.0.insert(Effect::Pure);
        set
    }

    pub fn insert(&mut self, effect: Effect) {
        self.0.remove(&Effect::Pure);
        self.0.insert(effect);
    }

    pub fn union_with(&mut self, other: &EffectSet) {
        for effect in &other.0 {
            if *effect != Effect::Pure {
                self.insert(*effect);
            }
        }
    }

    pub fn contains(&self, effect: Effect) -> bool {
        self.0.contains(&effect)
    }

    pub fn is_pure(&self) -> bool {
        self.0.is_empty() || self.0 == BTreeSet::from([Effect::Pure])
    }

    /// Effects that involve the physical world or external I/O. Reactive
    /// coordination contexts (`on`/`cycle`) must not have these.
    pub fn has_physical(&self) -> bool {
        self.contains(Effect::Motion) || self.contains(Effect::Timing) || self.contains(Effect::Io)
    }

    pub fn names(&self) -> Vec<&'static str> {
        self.0
            .iter()
            .filter(|effect| **effect != Effect::Pure)
            .map(|effect| match effect {
                Effect::Pure => "pure",
                Effect::StateMutation => "state mutation",
                Effect::Timing => "timing",
                Effect::Motion => "motion",
                Effect::Io => "io",
                Effect::ProcessControl => "process control",
            })
            .collect()
    }
}

/// Effects of a statement, given the already-inferred effects of user functions.
pub fn statement_effects(
    statement: &Statement,
    signals: &SignalSchema,
    aliases: &HashMap<String, String>,
    function_effects: &HashMap<String, EffectSet>,
) -> EffectSet {
    let mut effects = EffectSet::new();
    match statement {
        Statement::Let { value, .. } => {
            effects.union_with(&expression_effects(value, signals, aliases, function_effects))
        }
        Statement::MoveJ { .. } | Statement::MoveL { .. } | Statement::MoveC { .. } => {
            effects.insert(Effect::Motion)
        }
        Statement::Wait(_) => effects.insert(Effect::Timing),
        Statement::SetOutput { value, .. } => {
            effects.insert(Effect::Io);
            effects.union_with(&expression_effects(value, signals, aliases, function_effects));
        }
        Statement::If {
            condition,
            then_branch,
            else_branch,
        } => {
            effects.union_with(&expression_effects(condition, signals, aliases, function_effects));
            for statement in then_branch {
                effects.union_with(&statement_effects(statement, signals, aliases, function_effects));
            }
            if let Some(branch) = else_branch {
                for statement in branch {
                    effects.union_with(&statement_effects(statement, signals, aliases, function_effects));
                }
            }
        }
        Statement::Request { args, .. } => {
            effects.insert(Effect::ProcessControl);
            for arg in args {
                effects.union_with(&expression_effects(&arg.value, signals, aliases, function_effects));
            }
        }
        Statement::Start { .. } => effects.insert(Effect::ProcessControl),
        Statement::Assign { value, .. } => {
            effects.insert(Effect::StateMutation);
            effects.union_with(&expression_effects(value, signals, aliases, function_effects));
        }
        Statement::Expr(expr) => {
            effects.union_with(&expression_effects(expr, signals, aliases, function_effects))
        }
    }
    effects
}

/// Effects of an expression. Only signal reads and calls can introduce effects;
/// literals, constructors and arithmetic are pure.
pub fn expression_effects(
    expr: &Expr,
    signals: &SignalSchema,
    aliases: &HashMap<String, String>,
    function_effects: &HashMap<String, EffectSet>,
) -> EffectSet {
    let mut effects = EffectSet::new();
    match expr {
        Expr::Identifier(_)
        | Expr::Number(_)
        | Expr::StringLiteral(_)
        | Expr::Boolean(_)
        | Expr::Length(_)
        | Expr::Angle(_)
        | Expr::Duration(_) => {}
        Expr::Vector3(components) => {
            for component in components {
                effects.union_with(&expression_effects(component, signals, aliases, function_effects));
            }
        }
        Expr::Binary { left, right, .. } => {
            effects.union_with(&expression_effects(left, signals, aliases, function_effects));
            effects.union_with(&expression_effects(right, signals, aliases, function_effects));
        }
        Expr::Unary { operand, .. } => {
            effects.union_with(&expression_effects(operand, signals, aliases, function_effects));
        }
        Expr::Call { callee, args } => {
            match callee.as_str() {
                "set_output" => effects.insert(Effect::Io),
                "movej" | "movel" | "movec" => effects.insert(Effect::Motion),
                "wait" => effects.insert(Effect::Timing),
                _ => {
                    if let Some(function) = function_effects.get(callee) {
                        effects.union_with(function);
                    }
                }
            }
            for arg in args {
                effects.union_with(&expression_effects(&arg.value, signals, aliases, function_effects));
            }
        }
        Expr::MemberCall { object, args, .. } => {
            effects.union_with(&expression_effects(object, signals, aliases, function_effects));
            for arg in args {
                effects.union_with(&expression_effects(&arg.value, signals, aliases, function_effects));
            }
        }
        Expr::MemberAccess { object, .. } => {
            if let Expr::Identifier(name) = object.as_ref() {
                let module = aliases.get(name).map(String::as_str).unwrap_or(name);
                if signals.knows_module(module) {
                    effects.insert(Effect::Io);
                }
            } else {
                effects.union_with(&expression_effects(
                    object,
                    signals,
                    aliases,
                    function_effects,
                ));
            }
        }
    }
    effects
}

/// Infer the effects of every user function by fixed-point iteration over the
/// call graph. Recursion is not part of the language; the fixed point still
/// converges because effect sets only grow.
pub fn analyze_function_effects(program: &Program, signals: &SignalSchema) -> HashMap<String, EffectSet> {
    let aliases = collect_use_aliases(program);
    let mut effects: HashMap<String, EffectSet> = program
        .items
        .iter()
        .filter_map(|item| match item {
            Item::Function(function) => Some((function.name.clone(), EffectSet::new())),
            _ => None,
        })
        .collect();

    // At most one propagation step per function; monotonic growth guarantees
    // convergence in that bound.
    let bound = effects.len().max(1);
    for _ in 0..=bound {
        let mut changed = false;
        for item in &program.items {
            let Item::Function(function) = item else {
                continue;
            };
            let mut inferred = EffectSet::new();
            for statement in &function.body {
                inferred.union_with(&statement_effects(statement, signals, &aliases, &effects));
            }
            if let Some(tail) = &function.tail_expr {
                inferred.union_with(&expression_effects(tail, signals, &aliases, &effects));
            }
            let previous = effects.get(&function.name).cloned().unwrap_or_default();
            if inferred != previous {
                effects.insert(function.name.clone(), inferred);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    effects
}

fn collect_use_aliases(program: &Program) -> HashMap<String, String> {
    let mut aliases = HashMap::new();
    for item in &program.items {
        if let Item::Use(use_decl) = item {
            collect_alias_tree(&use_decl.tree, &mut aliases);
        }
    }
    aliases
}

fn collect_alias_tree(tree: &UseTree, aliases: &mut HashMap<String, String>) {
    if tree.is_group() {
        for child in &tree.group {
            collect_alias_tree(child, aliases);
        }
        return;
    }
    if let Some(node) = tree.prefix.last() {
        let local = tree.alias.clone().unwrap_or_else(|| node.clone());
        aliases.insert(local, node.clone());
    }
}

/// Human-readable effect names for diagnostics.
pub fn describe_effects(effects: &EffectSet) -> String {
    let names = effects.names();
    if names.is_empty() {
        "none".to_string()
    } else {
        names.join(", ")
    }
}
