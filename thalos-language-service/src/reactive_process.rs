//! Reactive **process** model — semantics (compile) and runtime evaluation.
//!
//! This is Task 2 + Task 3 of the reactive surface, over the AST parsed in Task 1:
//!
//! - **Task 2 (semantics):** [`compile_process`] resolves `use` aliases against a
//!   [`SignalSchema`] (validating nodes and channels), validates **contexts**
//!   (`start` only in `main`, `request`/state writes only inside a `process`),
//!   resolves events (`rising`/`falling`/level), and rejects reactions that try to
//!   actuate (reactions only update state and/or request work).
//! - **Task 3 (IR + runtime):** [`ProcessProgram`] is the resolved IR;
//!   [`ProcessRuntime`] evaluates it tick by tick — edge detection for events,
//!   state updates, and work requests — yielding [`Effect`]s.
//!
//! Out of scope here: admission/arbitration of works, work lifecycle, execution of
//! physical operations. Turning an [`Effect::Request`] into an admitted unit is the
//! runtime's job.

use std::collections::{BTreeMap, HashMap, HashSet};

use serde::{Deserialize, Serialize};

use thalos_core::execution::session::{ObservationBundle, eval_channel_access};
use thalos_lang::ast::{
    Arg, BinaryOp, Expr as AstExpr, FnDecl, Item, Program, Statement as AstStatement, UnaryOp,
    UseTree,
};

use crate::compiler::prepare_symbols;
use crate::evaluator::{CompileTimeValue, EvalResult, Evaluator};
use crate::reactive::{SignalRef, SignalSchema, SignalType};
use crate::reactive_eval::RuntimeValue;

// ───────────────────────────────────────────────────────────────────────────
// IR
// ───────────────────────────────────────────────────────────────────────────

/// A resolved `use` alias: the local name a program uses for an interconnection
/// node.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignalAlias {
    pub local: String,
    pub node: String,
}

/// A process-model expression.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PExpr {
    Constant(CompileTimeValue),
    Signal {
        module: String,
        channel: String,
        ty: SignalType,
    },
    /// A reference to a process state or a work parameter (by name).
    Ref(String),
    Binary {
        left: Box<PExpr>,
        op: BinaryOp,
        right: Box<PExpr>,
    },
    Unary {
        op: UnaryOp,
        operand: Box<PExpr>,
    },
}

/// A process-model statement (coordination level; physical operations are calls).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PStmt {
    /// `state = value`
    SetState { name: String, value: PExpr },
    /// `request work(args)` (optionally `as token`)
    Request {
        work: String,
        args: Vec<(Option<String>, PExpr)>,
        token: Option<String>,
    },
    SetOutput { name: String, value: bool },
    /// A call to a task function inside a `cycle`/`work` body.
    Call { function: String, args: Vec<PExpr> },
    If {
        condition: PExpr,
        then_branch: Vec<PStmt>,
        else_branch: Option<Vec<PStmt>>,
    },
}

/// The trigger of a reaction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PEvent {
    Rising(SignalRef),
    Falling(SignalRef),
    Level(PExpr),
    /// A terminal (or `busy`) outcome of a request occurrence token.
    Token { token: String, event: TokenEvent },
}

/// A token occurrence outcome observable by a reactive process.
///
/// `Admitted` and `Busy` are not terminal: they do not free the token.
/// `Rejected`, `Completed`, `Failed` and `Unconfirmed` are terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TokenEvent {
    Admitted,
    Rejected,
    Completed,
    Failed,
    Unconfirmed,
    Busy,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PState {
    pub name: String,
    pub init: PExpr,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PReaction {
    pub event: PEvent,
    pub body: Vec<PStmt>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PWork {
    pub name: String,
    pub params: Vec<String>,
    pub body: Vec<PStmt>,
}

/// A resolved persistent process.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProcessProgram {
    pub name: String,
    pub aliases: Vec<SignalAlias>,
    /// Request occurrence tokens declared in this process (unique).
    pub tokens: Vec<String>,
    pub states: Vec<PState>,
    pub reactions: Vec<PReaction>,
    pub cycle: Vec<PStmt>,
    pub works: Vec<PWork>,
}

// ───────────────────────────────────────────────────────────────────────────
// Compile (Task 2)
// ───────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Scope {
    Reaction,
    Cycle,
    Work,
}

struct Ctx<'a> {
    evaluator: &'a Evaluator<'a>,
    aliases: &'a BTreeMap<String, String>,
    schema: &'a SignalSchema,
    states: &'a HashSet<String>,
    tokens: &'a HashSet<String>,
    locals: &'a HashSet<String>,
    functions: &'a HashMap<String, &'a FnDecl>,
}

impl Ctx<'_> {
    fn expr(&self, expr: &AstExpr) -> Result<PExpr, String> {
        if let EvalResult::Value(value) = self.evaluator.eval_expr(expr) {
            return Ok(PExpr::Constant(value));
        }
        match expr {
            AstExpr::Identifier(id) => {
                if self.states.contains(id) || self.locals.contains(id) {
                    Ok(PExpr::Ref(id.clone()))
                } else {
                    Err(format!("unknown identifier '{id}'"))
                }
            }
            AstExpr::MemberAccess { object, member } => self.signal(object, member),
            AstExpr::Binary { left, op, right } => Ok(PExpr::Binary {
                left: Box::new(self.expr(left)?),
                op: *op,
                right: Box::new(self.expr(right)?),
            }),
            AstExpr::Unary { op, operand } => Ok(PExpr::Unary {
                op: *op,
                operand: Box::new(self.expr(operand)?),
            }),
            other => Err(format!("unsupported expression: {other:?}")),
        }
    }

    fn signal(&self, object: &str, member: &str) -> Result<PExpr, String> {
        let node = self
            .aliases
            .get(object)
            .ok_or_else(|| format!("'{object}' is not imported (missing `use`)"))?;
        match self.schema.type_of(node, member) {
            Some(ty) => Ok(PExpr::Signal {
                module: node.clone(),
                channel: member.to_string(),
                ty,
            }),
            None => Err(format!("node '{node}' has no signal '{member}'")),
        }
    }

    fn stmts(&self, stmts: &[AstStatement], scope: Scope) -> Result<Vec<PStmt>, String> {
        let mut out = Vec::new();
        for stmt in stmts {
            match stmt {
                AstStatement::Assign { name, value } => {
                    if !self.states.contains(name) {
                        return Err(format!("unknown state '{name}'"));
                    }
                    out.push(PStmt::SetState {
                        name: name.clone(),
                        value: self.expr(value)?,
                    });
                }
                AstStatement::Request { name, args, token } => {
                    if scope == Scope::Work && token.is_some() {
                        return Err(
                            "`request ... as token` is only allowed in `on`/`cycle`".to_string(),
                        );
                    }
                    let mut compiled = Vec::with_capacity(args.len());
                    for arg in args {
                        compiled.push((arg.name.clone(), self.expr(&arg.value)?));
                    }
                    out.push(PStmt::Request {
                        work: name.clone(),
                        args: compiled,
                        token: token.clone(),
                    });
                }
                AstStatement::If {
                    condition,
                    then_branch,
                    else_branch,
                } => out.push(PStmt::If {
                    condition: self.expr(condition)?,
                    then_branch: self.stmts(then_branch, scope)?,
                    else_branch: match else_branch {
                        Some(branch) => Some(self.stmts(branch, scope)?),
                        None => None,
                    },
                }),
                AstStatement::SetOutput { output, value } => {
                    require_work_scope(scope, "set_output")?;
                    let v = match self.evaluator.eval_expr(value) {
                        EvalResult::Value(CompileTimeValue::Bool(b)) => b,
                        _ => return Err("set_output value must be a constant bool".to_string()),
                    };
                    out.push(PStmt::SetOutput {
                        name: output.clone(),
                        value: v,
                    });
                }
                AstStatement::MoveJ { .. } => require_work_scope(scope, "movej")?,
                AstStatement::MoveL { .. } => require_work_scope(scope, "movel")?,
                AstStatement::MoveC { .. } => require_work_scope(scope, "movec")?,
                AstStatement::Wait(_) => require_work_scope(scope, "wait")?,
                AstStatement::Start { .. } => {
                    return Err("`start` is only allowed in `main`".to_string());
                }
                AstStatement::Expr(AstExpr::Call { callee, args }) => {
                    if scope == Scope::Work {
                        let mut compiled = Vec::with_capacity(args.len());
                        for arg in args {
                            compiled.push(self.expr(&arg.value)?);
                        }
                        out.push(PStmt::Call {
                            function: callee.clone(),
                            args: compiled,
                        });
                    } else {
                        // Resolution first: an unknown call is a semantic error,
                        // independent of whether effect analysis runs.
                        if callee == "set_output" {
                            return Err(format!(
                                "physical operation 'set_output' is not allowed in {}",
                                scope_name(scope)
                            ));
                        }
                        if !self.functions.contains_key(callee) {
                            return Err(format!("unknown function '{callee}'"));
                        }
                        // Any call is rejected; the message distinguishes physical
                        // from pure, but neither is accepted as a statement.
                        if self.fn_has_physical_effects(callee, &mut HashSet::new())? {
                            return Err(format!(
                                "physical operation '{callee}' is not allowed in {}",
                                scope_name(scope)
                            ));
                        }
                        return Err(format!(
                            "function calls are not supported in {} (use an expression or `request work`)",
                            scope_name(scope)
                        ));
                    }
                }
                AstStatement::Expr(_) => {
                    if scope != Scope::Work {
                        return Err(format!(
                            "expression statements are not supported in {}",
                            scope_name(scope)
                        ));
                    }
                }
                AstStatement::Let { .. } => {
                    if scope != Scope::Work {
                        return Err(format!(
                            "local bindings (`let`) are not supported in {}",
                            scope_name(scope)
                        ));
                    }
                }
            }
        }
        Ok(out)
    }

    /// Whether a function produces physical/IO effects, directly or transitively.
    ///
    /// `visited` guards against call cycles. `set_output` is recognized in both AST
    /// forms: the dedicated [`AstStatement::SetOutput`] and the
    /// `Expr::Call { callee: "set_output" }` form used when the output name is a
    /// string literal.
    fn fn_has_physical_effects(
        &self,
        name: &str,
        visited: &mut HashSet<String>,
    ) -> Result<bool, String> {
        if !visited.insert(name.to_string()) {
            return Ok(false);
        }
        let Some(function) = self.functions.get(name) else {
            return Err(format!("unknown function '{name}'"));
        };
        for stmt in &function.body {
            if self.stmt_has_physical(stmt, visited)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn stmt_has_physical(
        &self,
        stmt: &AstStatement,
        visited: &mut HashSet<String>,
    ) -> Result<bool, String> {
        Ok(match stmt {
            AstStatement::MoveJ { .. }
            | AstStatement::MoveL { .. }
            | AstStatement::MoveC { .. }
            | AstStatement::Wait(_)
            | AstStatement::SetOutput { .. }
            | AstStatement::Start { .. } => true,
            AstStatement::If {
                then_branch,
                else_branch,
                ..
            } => {
                for s in then_branch {
                    if self.stmt_has_physical(s, visited)? {
                        return Ok(true);
                    }
                }
                if let Some(branch) = else_branch {
                    for s in branch {
                        if self.stmt_has_physical(s, visited)? {
                            return Ok(true);
                        }
                    }
                }
                false
            }
            AstStatement::Expr(AstExpr::Call { callee, .. }) => {
                if callee == "set_output" {
                    true
                } else if self.functions.contains_key(callee) {
                    self.fn_has_physical_effects(callee, visited)?
                } else {
                    return Err(format!("unknown function '{callee}'"));
                }
            }
            _ => false,
        })
    }

    fn event(&self, event: &thalos_lang::ast::Event) -> Result<PEvent, String> {
        use thalos_lang::ast::Event as AstEvent;
        match event {
            AstEvent::Rising(e) => Ok(PEvent::Rising(self.edge_signal(e)?)),
            AstEvent::Falling(e) => Ok(PEvent::Falling(self.edge_signal(e)?)),
            AstEvent::Level(e) => match token_event(e, self.tokens)? {
                Some((token, event)) => Ok(PEvent::Token { token, event }),
                None => Ok(PEvent::Level(self.expr(e)?)),
            },
        }
    }

    fn edge_signal(&self, expr: &AstExpr) -> Result<SignalRef, String> {
        match expr {
            AstExpr::MemberAccess { object, member } => {
                let node = self
                    .aliases
                    .get(object)
                    .ok_or_else(|| format!("'{object}' is not imported (missing `use`)"))?;
                match self.schema.type_of(node, member) {
                    Some(SignalType::Bool) => Ok(SignalRef {
                        module: node.clone(),
                        channel: member.clone(),
                    }),
                    Some(other) => Err(format!(
                        "`rising`/`falling` needs a Bool signal, '{node}.{member}' is {other:?}"
                    )),
                    None => Err(format!("node '{node}' has no signal '{member}'")),
                }
            }
            other => Err(format!("event expects a signal, got {other:?}")),
        }
    }
}

fn scope_name(scope: Scope) -> &'static str {
    match scope {
        Scope::Reaction => "on",
        Scope::Cycle => "cycle",
        Scope::Work => "work",
    }
}

/// Reject a physical operation outside a `work` body.
fn require_work_scope(scope: Scope, operation: &str) -> Result<(), String> {
    if scope == Scope::Work {
        Ok(())
    } else {
        Err(format!(
            "physical operation '{operation}' is not allowed in {}",
            scope_name(scope)
        ))
    }
}

/// Resolve `use` trees into a local-name → node map, validating each leaf against
/// the schema.
fn collect_aliases(
    tree: &UseTree,
    out: &mut BTreeMap<String, String>,
    schema: &SignalSchema,
    errors: &mut Vec<String>,
) {
    if tree.is_group() {
        for child in &tree.group {
            collect_aliases(child, out, schema, errors);
        }
        return;
    }
    let Some(node) = tree.prefix.last().cloned() else {
        errors.push("empty `use` path".to_string());
        return;
    };
    if !schema.knows_module(&node) {
        errors.push(format!("unknown signal node '{node}'"));
        return;
    }
    let local = tree.alias.clone().unwrap_or_else(|| node.clone());
    out.insert(local, node);
}

fn find_started_process(program: &Program) -> Option<String> {
    for item in &program.items {
        if let Item::Function(f) = item
            && f.name == "main"
        {
            for stmt in &f.body {
                if let AstStatement::Start { process } = stmt {
                    return Some(process.clone());
                }
            }
        }
    }
    None
}

fn collect_process_tokens(
    process: &thalos_lang::ast::ProcessDecl,
    tokens: &mut HashSet<String>,
    errors: &mut Vec<String>,
) {
    for item in &process.items {
        match item {
            thalos_lang::ast::ProcessItem::On(on) => {
                collect_statement_tokens(&on.body, tokens, errors)
            }
            thalos_lang::ast::ProcessItem::Cycle(cycle) => {
                collect_statement_tokens(&cycle.body, tokens, errors)
            }
            thalos_lang::ast::ProcessItem::State(_) => {}
        }
    }
}

fn collect_statement_tokens(
    stmts: &[AstStatement],
    tokens: &mut HashSet<String>,
    errors: &mut Vec<String>,
) {
    for stmt in stmts {
        match stmt {
            AstStatement::Request {
                token: Some(token), ..
            } => {
                // A token is a process-scoped slot: several request sites may
                // name the same token. Only one occurrence may own it at a time.
                tokens.insert(token.clone());
            }
            AstStatement::If {
                then_branch,
                else_branch,
                ..
            } => {
                collect_statement_tokens(then_branch, tokens, errors);
                if let Some(else_branch) = else_branch {
                    collect_statement_tokens(else_branch, tokens, errors);
                }
            }
            _ => {}
        }
    }
}

/// Resolve `token.event` in an `on` header. Returns `None` when the member
/// access is not a declared token (it may still be a signal level condition).
fn token_event(
    expr: &AstExpr,
    tokens: &HashSet<String>,
) -> Result<Option<(String, TokenEvent)>, String> {
    let AstExpr::MemberAccess { object, member } = expr else {
        return Ok(None);
    };
    if !tokens.contains(object) {
        return Ok(None);
    }
    let event = match member.as_str() {
        "admitted" => TokenEvent::Admitted,
        "rejected" => TokenEvent::Rejected,
        "completed" => TokenEvent::Completed,
        "failed" => TokenEvent::Failed,
        "unconfirmed" => TokenEvent::Unconfirmed,
        "busy" => TokenEvent::Busy,
        other => return Err(format!("token '{object}' has no event '{other}'")),
    };
    Ok(Some((object.clone(), event)))
}

/// Compile a reactive program into a [`ProcessProgram`], validated against
/// `schema` (Task 2).
pub fn compile_process(ast: &Program, schema: &SignalSchema) -> Result<ProcessProgram, Vec<String>> {
    let (ast, bind_errors) = crate::binder::normalize_program(ast);
    if !bind_errors.is_empty() {
        return Err(bind_errors);
    }

    let prep = prepare_symbols(&ast);
    let mut errors = prep.errors.clone();

    let mut aliases = BTreeMap::new();
    for item in &ast.items {
        if let Item::Use(u) = item {
            collect_aliases(&u.tree, &mut aliases, schema, &mut errors);
        }
    }

    let Some(process_name) = find_started_process(&ast) else {
        errors.push("no `start <process>` found in `main`".to_string());
        return Err(errors);
    };
    let process = ast.items.iter().find_map(|item| match item {
        Item::Process(p) if p.name == process_name => Some(p),
        _ => None,
    });
    let Some(process) = process else {
        errors.push(format!("process '{process_name}' not found"));
        return Err(errors);
    };

    let states: HashSet<String> = process
        .items
        .iter()
        .filter_map(|item| match item {
            thalos_lang::ast::ProcessItem::State(s) => Some(s.name.clone()),
            _ => None,
        })
        .collect();
    // Request occurrence tokens: declared at `request ... as token` sites in
    // on/cycle, unique per process, and never colliding with state/aliases.
    let mut tokens = HashSet::new();
    collect_process_tokens(process, &mut tokens, &mut errors);
    for token in &tokens {
        if states.contains(token) {
            errors.push(format!("token '{token}' collides with a state"));
        }
        if aliases.contains_key(token) {
            errors.push(format!("token '{token}' collides with a signal alias"));
        }
    }
    let no_locals: HashSet<String> = HashSet::new();

    // Function declarations, for effect analysis and call resolution.
    let functions: HashMap<String, &FnDecl> = ast
        .items
        .iter()
        .filter_map(|item| match item {
            Item::Function(f) => Some((f.name.clone(), f)),
            _ => None,
        })
        .collect();

    let evaluator = Evaluator::with_target_values(&prep.table, &prep.target_values);
    let ctx = Ctx {
        evaluator: &evaluator,
        aliases: &aliases,
        schema,
        states: &states,
        tokens: &tokens,
        locals: &no_locals,
        functions: &functions,
    };

    let mut compiled_states = Vec::new();
    let mut reactions = Vec::new();
    let mut cycle = Vec::new();
    for item in &process.items {
        match item {
            thalos_lang::ast::ProcessItem::State(s) => match ctx.expr(&s.value) {
                Ok(init) => compiled_states.push(PState {
                    name: s.name.clone(),
                    init,
                }),
                Err(e) => errors.push(format!("state '{}': {e}", s.name)),
            },
            thalos_lang::ast::ProcessItem::On(on) => match ctx.event(&on.event) {
                Ok(event) => match ctx.stmts(&on.body, Scope::Reaction) {
                    Ok(body) => reactions.push(PReaction { event, body }),
                    Err(e) => errors.push(format!("on-body: {e}")),
                },
                Err(e) => errors.push(format!("on-event: {e}")),
            },
            thalos_lang::ast::ProcessItem::Cycle(c) => match ctx.stmts(&c.body, Scope::Cycle) {
                Ok(body) => cycle = body,
                Err(e) => errors.push(format!("cycle-body: {e}")),
            },
        }
    }

    let mut works = Vec::new();
    for item in &ast.items {
        if let Item::Work(w) = item {
            let params: HashSet<String> = w.params.iter().map(|p| p.name.clone()).collect();
            let work_ctx = Ctx {
                evaluator: &evaluator,
                aliases: &aliases,
                schema,
                states: &states,
                tokens: &tokens,
                locals: &params,
                functions: &functions,
            };
            match work_ctx.stmts(&w.body, Scope::Work) {
                Ok(body) => works.push(PWork {
                    name: w.name.clone(),
                    params: w.params.iter().map(|p| p.name.clone()).collect(),
                    body,
                }),
                Err(e) => errors.push(format!("work '{}': {e}", w.name)),
            }
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    let mut token_names: Vec<String> = tokens.iter().cloned().collect();
    token_names.sort();

    Ok(ProcessProgram {
        name: process_name,
        aliases: aliases
            .into_iter()
            .map(|(local, node)| SignalAlias { local, node })
            .collect(),
        tokens: token_names,
        states: compiled_states,
        reactions,
        cycle,
        works,
    })
}

// ───────────────────────────────────────────────────────────────────────────
// Work specialization (compile boundary: AST → Program)
// ───────────────────────────────────────────────────────────────────────────

/// Build a runnable [`Program`] for a concrete work request: a copy of `program`
/// whose entry (`main`) is the **work body** with its parameters substituted by
/// the concrete `args`.
///
/// This reuses the compiler's semantic rules end to end — no `.thls` text is
/// generated or interpolated: the caller compiles the returned AST through the
/// canonical `compile` → `resolve` pipeline and then plans it.
///
/// Parameters are bound **by name** (the request carries named args). Every
/// declared parameter must be provided.
pub fn specialize_work(
    program: &Program,
    work_name: &str,
    args: &[(String, CompileTimeValue)],
) -> Result<Program, Vec<String>> {
    let work = program.items.iter().find_map(|item| match item {
        Item::Work(w) if w.name == work_name => Some(w),
        _ => None,
    });
    let Some(work) = work else {
        return Err(vec![format!("work '{work_name}' not found")]);
    };

    let mut bindings: std::collections::HashMap<&str, AstExpr> = std::collections::HashMap::new();
    for (name, value) in args {
        match value_to_expr(value) {
            Ok(expr) => {
                bindings.insert(name.as_str(), expr);
            }
            Err(e) => return Err(vec![format!("arg '{name}': {e}")]),
        }
    }
    for param in &work.params {
        if !bindings.contains_key(param.name.as_str()) {
            return Err(vec![format!(
                "work '{work_name}' missing argument '{}'",
                param.name
            )]);
        }
    }

    let mut body = Vec::with_capacity(work.body.len());
    for stmt in &work.body {
        match substitute_statement(stmt, &bindings) {
            Ok(s) => body.push(s),
            Err(e) => return Err(vec![e]),
        }
    }

    let entry = FnDecl {
        name: "main".to_string(),
        params: Vec::new(),
        return_type: None,
        body,
        tail_expr: None,
    };

    let mut items: Vec<Item> = program
        .items
        .iter()
        .filter(|item| !matches!(item, Item::Function(f) if f.name == "main"))
        .cloned()
        .collect();
    items.push(Item::Function(entry));

    Ok(Program { items })
}

fn value_to_expr(value: &CompileTimeValue) -> Result<AstExpr, String> {
    match value {
        CompileTimeValue::Bool(b) => Ok(AstExpr::Boolean(*b)),
        CompileTimeValue::Int(i) => Ok(AstExpr::Number(*i as f64)),
        CompileTimeValue::Float(f) => Ok(AstExpr::Number(*f)),
        other => Err(format!("unsupported work argument value: {other:?}")),
    }
}

fn substitute_statement(
    stmt: &AstStatement,
    bindings: &std::collections::HashMap<&str, AstExpr>,
) -> Result<AstStatement, String> {
    Ok(match stmt {
        AstStatement::Let {
            name,
            type_ann,
            value,
        } => AstStatement::Let {
            name: name.clone(),
            type_ann: type_ann.clone(),
            value: substitute_expr(value, bindings)?,
        },
        AstStatement::MoveJ { target } => AstStatement::MoveJ {
            target: substitute_expr(target, bindings)?,
        },
        AstStatement::MoveL { target } => AstStatement::MoveL {
            target: substitute_expr(target, bindings)?,
        },
        AstStatement::MoveC { via, target } => AstStatement::MoveC {
            via: substitute_expr(via, bindings)?,
            target: substitute_expr(target, bindings)?,
        },
        AstStatement::Wait(expr) => AstStatement::Wait(substitute_expr(expr, bindings)?),
        AstStatement::SetOutput { output, value } => AstStatement::SetOutput {
            output: output.clone(),
            value: substitute_expr(value, bindings)?,
        },
        AstStatement::If {
            condition,
            then_branch,
            else_branch,
        } => AstStatement::If {
            condition: substitute_expr(condition, bindings)?,
            then_branch: then_branch
                .iter()
                .map(|s| substitute_statement(s, bindings))
                .collect::<Result<_, _>>()?,
            else_branch: match else_branch {
                Some(branch) => Some(
                    branch
                        .iter()
                        .map(|s| substitute_statement(s, bindings))
                        .collect::<Result<_, _>>()?,
                ),
                None => None,
            },
        },
        AstStatement::Request { name, args, token } => AstStatement::Request {
            name: name.clone(),
            token: token.clone(),
            args: args
                .iter()
                .map(|a| substitute_arg(a, bindings))
                .collect::<Result<_, _>>()?,
        },
        AstStatement::Start { process } => AstStatement::Start {
            process: process.clone(),
        },
        AstStatement::Assign { name, value } => AstStatement::Assign {
            name: name.clone(),
            value: substitute_expr(value, bindings)?,
        },
        AstStatement::Expr(expr) => AstStatement::Expr(substitute_expr(expr, bindings)?),
    })
}

fn substitute_arg(
    arg: &Arg,
    bindings: &std::collections::HashMap<&str, AstExpr>,
) -> Result<Arg, String> {
    Ok(Arg {
        name: arg.name.clone(),
        value: substitute_expr(&arg.value, bindings)?,
    })
}

fn substitute_expr(
    expr: &AstExpr,
    bindings: &std::collections::HashMap<&str, AstExpr>,
) -> Result<AstExpr, String> {
    Ok(match expr {
        AstExpr::Identifier(id) => match bindings.get(id.as_str()) {
            Some(replacement) => replacement.clone(),
            None => AstExpr::Identifier(id.clone()),
        },
        AstExpr::Number(n) => AstExpr::Number(*n),
        AstExpr::StringLiteral(s) => AstExpr::StringLiteral(s.clone()),
        AstExpr::Boolean(b) => AstExpr::Boolean(*b),
        AstExpr::Length(l) => AstExpr::Length(*l),
        AstExpr::Angle(a) => AstExpr::Angle(*a),
        AstExpr::Duration(d) => AstExpr::Duration(*d),
        AstExpr::Vector3([x, y, z]) => AstExpr::Vector3([
            Box::new(substitute_expr(x, bindings)?),
            Box::new(substitute_expr(y, bindings)?),
            Box::new(substitute_expr(z, bindings)?),
        ]),
        AstExpr::Pose {
            position,
            orientation,
        } => AstExpr::Pose {
            position: Box::new(substitute_expr(position, bindings)?),
            orientation: Box::new(substitute_expr(orientation, bindings)?),
        },
        AstExpr::Call { callee, args } => AstExpr::Call {
            callee: callee.clone(),
            args: args
                .iter()
                .map(|a| substitute_arg(a, bindings))
                .collect::<Result<_, _>>()?,
        },
        AstExpr::MemberCall {
            object,
            method,
            args,
        } => AstExpr::MemberCall {
            object: object.clone(),
            method: method.clone(),
            args: args
                .iter()
                .map(|a| substitute_arg(a, bindings))
                .collect::<Result<_, _>>()?,
        },
        AstExpr::MemberAccess { object, member } => AstExpr::MemberAccess {
            object: object.clone(),
            member: member.clone(),
        },
        AstExpr::Binary { left, op, right } => AstExpr::Binary {
            left: Box::new(substitute_expr(left, bindings)?),
            op: *op,
            right: Box::new(substitute_expr(right, bindings)?),
        },
        AstExpr::Unary { op, operand } => AstExpr::Unary {
            op: *op,
            operand: Box::new(substitute_expr(operand, bindings)?),
        },
    })
}

// ───────────────────────────────────────────────────────────────────────────
// Runtime (Task 3)
// ───────────────────────────────────────────────────────────────────────────

/// Why an admission/notification rejected a request occurrence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectionReason {
    /// The arbiter has no room (active slot + bounded pending queue are full).
    Capacity,
    /// The admission notification could not be delivered to the planner.
    Notification,
}

impl RejectionReason {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Capacity => "capacity",
            Self::Notification => "notification",
        }
    }
}

/// The outcome of a request occurrence, as seen by the process.
///
/// `Admitted` and `Busy` are **not** terminal: they do not free the token.
#[derive(Debug, Clone, PartialEq)]
pub enum TokenOutcome {
    Admitted,
    Rejected { reason: RejectionReason },
    Completed,
    Failed { reason: String },
    Unconfirmed,
    Busy,
}

impl TokenOutcome {
    pub fn event(&self) -> TokenEvent {
        match self {
            Self::Admitted => TokenEvent::Admitted,
            Self::Rejected { .. } => TokenEvent::Rejected,
            Self::Completed => TokenEvent::Completed,
            Self::Failed { .. } => TokenEvent::Failed,
            Self::Unconfirmed => TokenEvent::Unconfirmed,
            Self::Busy => TokenEvent::Busy,
        }
    }
}

/// A terminal (or `busy`) outcome that could not be correlated to its occurrence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TokenDeliveryError {
    UnknownToken(String),
    StaleOccurrence {
        token: String,
        expected: Option<u64>,
        got: u64,
    },
}

impl std::fmt::Display for TokenDeliveryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownToken(token) => write!(f, "unknown token '{token}'"),
            Self::StaleOccurrence {
                token,
                expected,
                got,
            } => write!(
                f,
                "stale occurrence for token '{token}': expected {expected:?}, got {got}"
            ),
        }
    }
}

impl std::error::Error for TokenDeliveryError {}

#[derive(Debug, Clone)]
struct PendingOutcome {
    token: String,
    occurrence: u64,
    outcome: TokenOutcome,
}

#[derive(Debug, Clone, Default)]
struct TokenSlot {
    /// The occurrence currently occupying the token, if any.
    current: Option<u64>,
}

/// An observable effect produced by evaluating the process for one tick.
#[derive(Debug, Clone, PartialEq)]
pub enum Effect {
    /// The process asks for a unit of work (not yet admitted).
    Request {
        token: Option<String>,
        occurrence: u64,
        work: String,
        args: Vec<(Option<String>, RuntimeValue)>,
    },
    /// A request was rejected because its token already had an occurrence.
    /// The active occurrence is untouched and the token is not freed.
    Busy { token: String, occurrence: u64 },
    /// A process state changed.
    StateChanged {
        name: String,
        value: RuntimeValue,
    },
}

/// Evaluates a [`ProcessProgram`] against live observations, one tick at a time.
pub struct ProcessRuntime {
    program: ProcessProgram,
    state: HashMap<String, RuntimeValue>,
    /// Previous boolean value per signal, for edge detection (Key: `module.channel`).
    prev: HashMap<String, bool>,
    /// True once at least one tick has observed each signal.
    seen: HashSet<String>,
    /// Per-process occurrence tokens.
    tokens: HashMap<String, TokenSlot>,
    /// Monotonic occurrence id (internal, never exposed by the language).
    next_occurrence: u64,
    /// Outcomes delivered by the runtime, drained FIFO at the next tick.
    outcomes: std::collections::VecDeque<PendingOutcome>,
}

impl ProcessRuntime {
    pub fn new(program: ProcessProgram) -> Self {
        let mut state = HashMap::new();
        for s in &program.states {
            if let PExpr::Constant(value) = &s.init {
                state.insert(s.name.clone(), RuntimeValue::from_constant(value.clone()));
            }
        }
        let tokens = program
            .tokens
            .iter()
            .map(|token| (token.clone(), TokenSlot::default()))
            .collect();
        Self {
            program,
            state,
            prev: HashMap::new(),
            seen: HashSet::new(),
            tokens,
            next_occurrence: 0,
            outcomes: std::collections::VecDeque::new(),
        }
    }

    pub fn program(&self) -> &ProcessProgram {
        &self.program
    }

    pub fn state(&self, name: &str) -> Option<&RuntimeValue> {
        self.state.get(name)
    }

    /// Deliver an outcome for an exact occurrence. Outcomes are applied FIFO at
    /// the start of the next tick; this method only validates correlation.
    ///
    /// `Busy` is a non-terminal event for a *new* occurrence that did not
    /// replace the active one, so it skips the current-occurrence check.
    pub fn deliver_outcome(
        &mut self,
        token: &str,
        occurrence: u64,
        outcome: TokenOutcome,
    ) -> Result<(), TokenDeliveryError> {
        let slot = self
            .tokens
            .get(token)
            .ok_or_else(|| TokenDeliveryError::UnknownToken(token.to_string()))?;
        if !matches!(outcome, TokenOutcome::Busy) && slot.current != Some(occurrence) {
            return Err(TokenDeliveryError::StaleOccurrence {
                token: token.to_string(),
                expected: slot.current,
                got: occurrence,
            });
        }
        self.outcomes.push_back(PendingOutcome {
            token: token.to_string(),
            occurrence,
            outcome,
        });
        Ok(())
    }

    /// Evaluate one control tick.
    ///
    /// Order: drain pending outcomes FIFO (applying each slot and running its
    /// token reactions), then evaluate observation reactions, then record edges.
    /// This lets a terminal outcome free a token for reuse in the same tick,
    /// while an older event cannot affect a newer occurrence.
    pub fn tick(&mut self, observations: &ObservationBundle) -> Vec<Effect> {
        let mut effects = Vec::new();
        let pending: Vec<PendingOutcome> = self.outcomes.drain(..).collect();
        for outcome in pending {
            if self.apply_outcome(&outcome) {
                self.run_token_reactions(
                    &outcome.token,
                    outcome.outcome.event(),
                    observations,
                    &mut effects,
                );
            }
        }

        let reactions = self.program.reactions.clone();
        for reaction in &reactions {
            if !matches!(reaction.event, PEvent::Token { .. })
                && self.event_fires(&reaction.event, observations)
            {
                self.exec(&reaction.body, observations, &mut effects);
            }
        }
        self.record_edges(observations);
        effects
    }

    fn apply_outcome(&mut self, pending: &PendingOutcome) -> bool {
        if matches!(pending.outcome, TokenOutcome::Busy) {
            return true;
        }
        let Some(slot) = self.tokens.get_mut(&pending.token) else {
            return false;
        };
        if slot.current != Some(pending.occurrence) {
            return false;
        }
        if matches!(
            pending.outcome,
            TokenOutcome::Rejected { .. }
                | TokenOutcome::Completed
                | TokenOutcome::Failed { .. }
                | TokenOutcome::Unconfirmed
        ) {
            slot.current = None;
        }
        true
    }

    fn run_token_reactions(
        &mut self,
        token: &str,
        event: TokenEvent,
        observations: &ObservationBundle,
        effects: &mut Vec<Effect>,
    ) {
        let reactions = self.program.reactions.clone();
        for reaction in &reactions {
            if matches!(
                &reaction.event,
                PEvent::Token { token: reaction_token, event: reaction_event }
                    if reaction_token == token && *reaction_event == event
            ) {
                self.exec(&reaction.body, observations, effects);
            }
        }
    }

    /// True when the process declares a `cycle` body.
    pub fn has_cycle(&self) -> bool {
        !self.program.cycle.is_empty()
    }

    /// Run one iteration of the `cycle` body, returning its effects.
    ///
    /// The cycle is a **background** admission unit: the runtime drives it only
    /// when the arbiter is idle (see the process model in the Industrial
    /// language reference, `thalos-lang.md` §11).
    pub fn run_cycle(&mut self, observations: &ObservationBundle) -> Vec<Effect> {
        let cycle = self.program.cycle.clone();
        let mut effects = Vec::new();
        self.exec(&cycle, observations, &mut effects);
        effects
    }

    fn event_fires(&self, event: &PEvent, observations: &ObservationBundle) -> bool {
        match event {
            PEvent::Rising(sig) => {
                let key = signalkey(sig);
                let now = signal_bool(sig, observations);
                self.seen.contains(&key) && self.prev.get(&key) == Some(&false) && now == Some(true)
            }
            PEvent::Falling(sig) => {
                let key = signalkey(sig);
                let now = signal_bool(sig, observations);
                self.seen.contains(&key) && self.prev.get(&key) == Some(&true) && now == Some(false)
            }
            PEvent::Level(expr) => matches!(self.eval(expr, observations).as_bool(), Some(true)),
            // Token events are delivered through the outcome queue, never by
            // observation edge/level evaluation.
            PEvent::Token { .. } => false,
        }
    }

    fn record_edges(&mut self, observations: &ObservationBundle) {
        for reaction in &self.program.reactions {
            let sig = match &reaction.event {
                PEvent::Rising(s) | PEvent::Falling(s) => s,
                PEvent::Level(_) | PEvent::Token { .. } => continue,
            };
            let key = signalkey(sig);
            if let Some(value) = signal_bool(sig, observations) {
                self.prev.insert(key.clone(), value);
                self.seen.insert(key);
            }
        }
    }

    fn exec(&mut self, body: &[PStmt], observations: &ObservationBundle, effects: &mut Vec<Effect>) {
        for stmt in body {
            match stmt {
                PStmt::SetState { name, value } => {
                    let v = self.eval(value, observations);
                    self.state.insert(name.clone(), v.clone());
                    effects.push(Effect::StateChanged {
                        name: name.clone(),
                        value: v,
                    });
                }
                PStmt::Request { work, args, token } => {
                    let occurrence = self.next_occurrence;
                    self.next_occurrence += 1;
                    if let Some(token_name) = token {
                        let busy = self
                            .tokens
                            .get(token_name)
                            .is_some_and(|slot| slot.current.is_some());
                        if busy {
                            effects.push(Effect::Busy {
                                token: token_name.clone(),
                                occurrence,
                            });
                            self.outcomes.push_back(PendingOutcome {
                                token: token_name.clone(),
                                occurrence,
                                outcome: TokenOutcome::Busy,
                            });
                            continue;
                        }
                        if let Some(slot) = self.tokens.get_mut(token_name) {
                            slot.current = Some(occurrence);
                        }
                    }
                    let evaluated = args
                        .iter()
                        .map(|(name, expr)| (name.clone(), self.eval(expr, observations)))
                        .collect();
                    effects.push(Effect::Request {
                        token: token.clone(),
                        occurrence,
                        work: work.clone(),
                        args: evaluated,
                    });
                }
                PStmt::If {
                    condition,
                    then_branch,
                    else_branch,
                } => {
                    if matches!(self.eval(condition, observations).as_bool(), Some(true)) {
                        self.exec(then_branch, observations, effects);
                    } else if let Some(branch) = else_branch {
                        self.exec(branch, observations, effects);
                    }
                }
                // Physical operations are not the coordination layer's concern here.
                PStmt::SetOutput { .. } | PStmt::Call { .. } => {}
            }
        }
    }

    fn eval(&self, expr: &PExpr, observations: &ObservationBundle) -> RuntimeValue {
        match expr {
            PExpr::Constant(value) => RuntimeValue::from_constant(value.clone()),
            PExpr::Signal { module, channel, ty } => {
                match eval_channel_access(module, channel, observations) {
                    Ok(value) => match ty {
                        SignalType::Bool => RuntimeValue::Bool(value != 0.0),
                        SignalType::Int => RuntimeValue::Int(value as i64),
                        SignalType::Float => RuntimeValue::Float(value),
                    },
                    Err(_) => RuntimeValue::Unknown,
                }
            }
            PExpr::Ref(name) => self
                .state
                .get(name)
                .cloned()
                .unwrap_or(RuntimeValue::Unknown),
            PExpr::Unary {
                op: UnaryOp::Neg,
                operand,
            } => match self.eval(operand, observations) {
                RuntimeValue::Int(i) => RuntimeValue::Int(-i),
                RuntimeValue::Float(f) => RuntimeValue::Float(-f),
                other => other,
            },
            PExpr::Binary { left, op, right } => {
                let lhs = self.eval(left, observations);
                let rhs = self.eval(right, observations);
                apply_binary(*op, lhs, rhs)
            }
        }
    }
}

fn signalkey(sig: &SignalRef) -> String {
    format!("{}.{}", sig.module, sig.channel)
}

fn signal_bool(sig: &SignalRef, observations: &ObservationBundle) -> Option<bool> {
    eval_channel_access(&sig.module, &sig.channel, observations)
        .ok()
        .map(|v| v != 0.0)
}

fn apply_binary(op: BinaryOp, lhs: RuntimeValue, rhs: RuntimeValue) -> RuntimeValue {
    match op {
        BinaryOp::Gt | BinaryOp::Lt | BinaryOp::Gte | BinaryOp::Lte | BinaryOp::Eq | BinaryOp::Neq => {
            if let (Some(a), Some(b)) = (lhs.as_f64(), rhs.as_f64()) {
                return RuntimeValue::Bool(match op {
                    BinaryOp::Gt => a > b,
                    BinaryOp::Lt => a < b,
                    BinaryOp::Gte => a >= b,
                    BinaryOp::Lte => a <= b,
                    BinaryOp::Eq => (a - b).abs() < f64::EPSILON,
                    BinaryOp::Neq => (a - b).abs() >= f64::EPSILON,
                    _ => unreachable!(),
                });
            }
            match (op, lhs.as_bool(), rhs.as_bool()) {
                (BinaryOp::Eq, Some(x), Some(y)) => RuntimeValue::Bool(x == y),
                (BinaryOp::Neq, Some(x), Some(y)) => RuntimeValue::Bool(x != y),
                _ => RuntimeValue::Unknown,
            }
        }
        BinaryOp::Add | BinaryOp::Sub | BinaryOp::Mul | BinaryOp::Div => {
            match (lhs.as_f64(), rhs.as_f64()) {
                (Some(a), Some(b)) => RuntimeValue::Float(match op {
                    BinaryOp::Add => a + b,
                    BinaryOp::Sub => a - b,
                    BinaryOp::Mul => a * b,
                    BinaryOp::Div => a / b,
                    _ => unreachable!(),
                }),
                _ => RuntimeValue::Unknown,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::device::{ChannelObservation, ChannelValue, SignalQuality};
    use thalos_lang::parse_source;

    fn schema() -> SignalSchema {
        SignalSchema::new()
            .with("box_sensor", "present", SignalType::Bool)
            .with("box_sensor", "id", SignalType::Int)
            .with("pallet", "full", SignalType::Bool)
    }

    const SRC: &str = r#"
use station::{ box_sensor as box, pallet }

process palletizing {
    state admitting : Bool = true
    state next_slot : Int = 0

    on rising(box.present) {
        if admitting {
            request place_box(box_id = box.id, slot = next_slot)
        }
    }
    on rising(pallet.full) {
        admitting = false
    }
    cycle {
        request maintain_station()
    }
}

work place_box(box_id : Int, slot : Int) {
    pick(box_id);
}

fn main() {
    start palletizing
}
"#;

    fn obs(pairs: &[(&str, f64)]) -> ObservationBundle {
        let mut bundle = ObservationBundle::default();
        for (sequence, (channel, value)) in pairs.iter().enumerate() {
            bundle.observations.insert(
                (*channel).to_string(),
                ChannelObservation {
                    channel_id: (*channel).to_string(),
                    sequence: sequence as u64,
                    sampled_at_ns: 0,
                    received_at_ns: 0,
                    value: ChannelValue::Scalar(*value),
                    unit: None,
                    quality: SignalQuality::Nominal,
                },
            );
        }
        bundle
    }

    #[test]
    fn compile_resolves_aliases_and_contexts() {
        let program = parse_source(SRC).expect("parse");
        let compiled = compile_process(&program, &schema()).expect("compile");
        assert_eq!(compiled.name, "palletizing");
        assert_eq!(compiled.aliases.len(), 2);
        assert_eq!(compiled.states.len(), 2);
        assert_eq!(compiled.reactions.len(), 2);
        assert_eq!(compiled.works.len(), 1);
        // outer signal resolved to the real node, not the alias
        match &compiled.reactions[0].event {
            PEvent::Rising(SignalRef { module, channel }) => {
                assert_eq!(module, "box_sensor");
                assert_eq!(channel, "present");
            }
            other => panic!("expected rising, got {other:?}"),
        }
    }

    #[test]
    fn unknown_node_is_an_error() {
        let src = r#"
use station::{ ghost as g }
process p { on rising(g.x) { request w() } }
work w() {}
fn main() { start p }
"#;
        let program = parse_source(src).expect("parse");
        let err = compile_process(&program, &schema()).expect_err("unknown node must fail");
        assert!(
            err.iter().any(|e| e.contains("unknown signal node 'ghost'")),
            "got {err:?}"
        );
    }

    #[test]
    fn a_reaction_cannot_actuate() {
        let program = parse_source(
            r#"
use station::{ box_sensor as box }
process p {
    on rising(box.present) {
        set_output("gripper", true);
    }
}
fn main() { start p }
"#,
        )
        .expect("parse");
        let err = compile_process(&program, &schema()).expect_err("actuation in a reaction must fail");
        assert!(
            err.iter()
                .any(|e| e.contains("physical operation 'set_output'") && e.contains("on")),
            "got {err:?}"
        );
    }

    #[test]
    fn cycle_rejects_physical_operations() {
        let template = r#"
use station::{ box_sensor as box }
target slot_1 = joints(0deg, 0deg)
target pos_1 = position([0mm, 0mm, 0mm])
target pos_2 = position([100mm, 0mm, 0mm])
process p {
    cycle {
        BODY
    }
}
fn main() { start p }
"#;
        for (label, body) in [
            ("movej", "movej(slot_1)"),
            ("movel", "movel(pos_1)"),
            ("movec", "movec(pos_1, pos_2)"),
            ("set_output_ident", "set_output(gripper, true)"),
            ("set_output_string", "set_output(\"gripper\", true);"),
            ("wait", "wait(200ms)"),
        ] {
            let src = template.replace("BODY", body);
            let program = parse_source(&src).expect("parse");
            let err =
                compile_process(&program, &schema()).expect_err(&format!("{label} must fail"));
            assert!(
                err.iter().any(|e| e.contains("not allowed in cycle")),
                "{label} got {err:?}"
            );
        }
    }

    #[test]
    fn cycle_rejects_function_calls_physical_and_pure() {
        let err = compile_process(
            &parse_source(
                r#"
use station::{ box_sensor as box }
fn do_move() { movej(joints(0deg, 0deg)) }
process p { cycle { do_move(); } }
fn main() { start p }
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("a physical call in cycle must fail");
        assert!(
            err.iter()
                .any(|e| e.contains("physical operation 'do_move'") && e.contains("cycle")),
            "got {err:?}"
        );

        let err = compile_process(
            &parse_source(
                r#"
use station::{ box_sensor as box }
fn pure() -> Int { 1 }
process p { cycle { pure(); } }
fn main() { start p }
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("a pure call in cycle must fail too");
        assert!(
            err.iter()
                .any(|e| e.contains("function calls are not supported in cycle")),
            "got {err:?}"
        );
    }

    #[test]
    fn cycle_rejects_let_and_unknown_calls() {
        let err = compile_process(
            &parse_source(
                r#"
use station::{ box_sensor as box }
process p { cycle { let x = 1 } }
fn main() { start p }
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("let in cycle must fail");
        assert!(
            err.iter().any(|e| e.contains("`let`") && e.contains("cycle")),
            "got {err:?}"
        );

        let err = compile_process(
            &parse_source(
                r#"
use station::{ box_sensor as box }
process p { cycle { ghost(); } }
fn main() { start p }
"#,
            )
            .expect("parse"),
            &schema(),
        )
        .expect_err("an unknown call must fail by resolution");
        assert!(
            err.iter().any(|e| e.contains("unknown function 'ghost'")),
            "got {err:?}"
        );
    }

    #[test]
    fn cycle_accepts_coordination_only() {
        let program = parse_source(
            r#"
use station::{ box_sensor as box }
process p {
    state n : Int = 0
    cycle {
        if n == 0 {
            request do_work()
        }
        n = n + 1
    }
}
work do_work() { }
fn main() { start p }
"#,
        )
        .expect("parse");
        compile_process(&program, &schema())
            .expect("a coordination-only cycle must compile");
    }

    #[test]
    fn runtime_edge_detects_and_requests_work() {
        let program = parse_source(SRC).expect("parse");
        let compiled = compile_process(&program, &schema()).expect("compile");
        let mut rt = ProcessRuntime::new(compiled);

        // First observation: baseline (no event, even though present = true).
        let first = rt.tick(&obs(&[("present", 1.0), ("id", 7.0)]));
        assert!(
            first.iter().all(|e| !matches!(e, Effect::Request { .. })),
            "startup does not invent an arrival event: {first:?}"
        );

        // Falling then rising → an arrival.
        rt.tick(&obs(&[("present", 0.0), ("id", 7.0)]));
        let arrival = rt.tick(&obs(&[("present", 1.0), ("id", 7.0)]));
        let request = arrival
            .iter()
            .find_map(|e| match e {
                Effect::Request { work, args, .. } => Some((work, args)),
                _ => None,
            })
            .expect("an arrival must request work");
        assert_eq!(request.0, "place_box");
        assert!(request.1.iter().any(|(n, _)| n.as_deref() == Some("box_id")));
    }

    #[test]
    fn runtime_state_write_stops_admission() {
        let program = parse_source(SRC).expect("parse");
        let compiled = compile_process(&program, &schema()).expect("compile");
        let mut rt = ProcessRuntime::new(compiled);

        rt.tick(&obs(&[("full", 0.0), ("present", 0.0), ("id", 0.0)]));
        let effect = rt.tick(&obs(&[("full", 1.0), ("present", 0.0), ("id", 0.0)]));
        assert!(
            effect.iter().any(|e| matches!(e, Effect::StateChanged { name, .. } if name == "admitting")),
            "pallet.full must write `admitting`: {effect:?}"
        );
        assert_eq!(rt.state("admitting"), Some(&RuntimeValue::Bool(false)));
    }

    const SPECIALIZE_SRC: &str = r#"
target slot_1 = joints(10deg, 20deg)
target slot_2 = joints(30deg, 40deg)

fn place(slot: Int) {
    if slot == 1 {
        movej(slot_1)
    } else {
        movej(slot_2)
    }
}

work place_box(box_id: Int, slot: Int) {
    place(slot);
}
"#;

    fn resolved_for(slot: i64) -> crate::model::ResolvedProgram {
        let ast = parse_source(SPECIALIZE_SRC).expect("parse");
        let specialized = specialize_work(
            &ast,
            "place_box",
            &[
                ("box_id".to_string(), CompileTimeValue::Int(12)),
                ("slot".to_string(), CompileTimeValue::Int(slot)),
            ],
        )
        .expect("specialize");
        let semantic = crate::compiler::SemanticCompiler::compile(&specialized).expect("compile");
        crate::resolver::SemanticResolver::resolve(&semantic).expect("resolve")
    }

    #[test]
    fn specialize_binds_payload_and_compiles_through_the_pipeline() {
        let one = resolved_for(1);
        let two = resolved_for(2);
        assert_ne!(
            one, two,
            "different payloads must yield different resolved programs"
        );
        assert_eq!(one.statements.len(), 1);
    }

    #[test]
    fn specialize_requires_all_arguments() {
        let ast = parse_source(SPECIALIZE_SRC).expect("parse");
        let err = specialize_work(
            &ast,
            "place_box",
            &[("box_id".to_string(), CompileTimeValue::Int(1))],
        )
        .expect_err("missing slot must fail");
        assert!(
            err.iter().any(|e| e.contains("missing argument 'slot'")),
            "got {err:?}"
        );
    }

    #[test]
    fn specialize_unknown_work_is_an_error() {
        let ast = parse_source(SPECIALIZE_SRC).expect("parse");
        let err = specialize_work(&ast, "ghost", &[]).expect_err("unknown work must fail");
        assert!(
            err.iter().any(|e| e.contains("work 'ghost' not found")),
            "got {err:?}"
        );
    }
}
