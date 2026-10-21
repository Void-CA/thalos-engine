//! Reactive IR — a control-flow-preserving, **runtime-evaluable** representation.
//!
//! The canonical `.thls` pipeline lowers a program to a **flat** `ResolvedProgram`
//! ([`crate::model::ResolvedProgram`]): `if` is resolved at compile time and, when
//! the condition is not constant, both branches are appended. That model cannot
//! carry a runtime condition to execution.
//!
//! This module defines the complementary representation used by reactive
//! programs (see `docs/design/modelo-reactivo.md`):
//!
//! - **Signals are first-class**: a read `module.channel` becomes
//!   [`ReactiveExpr::Signal`], typed against a [`SignalSchema`].
//! - **Control flow is preserved**: a non-constant `if` becomes
//!   [`ReactiveStatement::If`], never flattened.
//!
//! It is **additive**: it does not modify `ResolvedProgram`, the checker or the
//! resolver, so the run-once path is unaffected. Wiring the `.thls` front-end to
//! produce this representation is a later step.

use std::collections::{HashMap, HashSet};

use serde::{Deserialize, Serialize};

use thalos_lang::ast::{BinaryOp, UnaryOp};

use crate::evaluator::{CompileTimeValue, negate_value, read_member};
use crate::model::{
    CallSite, JointConfiguration, MotionKind, MotionTarget, Provenance, SemanticExpr,
    SemanticFunction, SemanticMotion, SemanticProgram, SemanticStatement,
};
use crate::types::Type;

// ───────────────────────────────────────────────────────────────────────────
// Signals
// ───────────────────────────────────────────────────────────────────────────

/// The value type of a signal read. Mirrors the payloads a channel observation
/// can carry (`ChannelValue`): `Scalar → Float`, `Integer → Int`, `Boolean → Bool`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalType {
    Bool,
    Int,
    Float,
}

impl SignalType {
    /// The `.thls` semantic type of a signal read.
    pub fn as_type(self) -> Type {
        match self {
            SignalType::Bool => Type::Bool,
            SignalType::Int => Type::Int,
            SignalType::Float => Type::Float,
        }
    }
}

/// A reference to a signal: the module (participant) and the channel name.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SignalRef {
    pub module: String,
    pub channel: String,
}

/// A declared signal and its type.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignalBinding {
    pub module: String,
    pub channel: String,
    pub ty: SignalType,
}

/// The declared signals a program may read.
///
/// A signal access is only valid if it appears here; the schema is the source of
/// truth for the *type* of a channel read. Where concrete signals are declared
/// (station configuration, interconnection module) is out of scope here.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignalSchema {
    bindings: Vec<SignalBinding>,
}

impl SignalSchema {
    pub fn new() -> Self {
        Self::default()
    }

    /// Declare a signal, replacing any previous declaration of the same channel.
    pub fn declare(
        &mut self,
        module: impl Into<String>,
        channel: impl Into<String>,
        ty: SignalType,
    ) {
        let module = module.into();
        let channel = channel.into();
        self.bindings.retain(|b| !(b.module == module && b.channel == channel));
        self.bindings.push(SignalBinding {
            module,
            channel,
            ty,
        });
    }

    /// Builder form of [`SignalSchema::declare`].
    pub fn with(mut self, module: impl Into<String>, channel: impl Into<String>, ty: SignalType) -> Self {
        self.declare(module, channel, ty);
        self
    }

    /// Type of a signal, if declared.
    pub fn type_of(&self, module: &str, channel: &str) -> Option<SignalType> {
        self.bindings
            .iter()
            .find(|b| b.module == module && b.channel == channel)
            .map(|b| b.ty)
    }

    /// True if `module` declares at least one signal.
    pub fn knows_module(&self, module: &str) -> bool {
        self.bindings.iter().any(|b| b.module == module)
    }

    /// All declared bindings.
    pub fn bindings(&self) -> &[SignalBinding] {
        &self.bindings
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Reactive expressions
// ───────────────────────────────────────────────────────────────────────────

/// A runtime-evaluable expression. Constants are folded where possible; signal
/// reads and runtime locals stay symbolic.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ReactiveExpr {
    Constant(CompileTimeValue),
    Signal {
        module: String,
        channel: String,
        ty: SignalType,
    },
    Local {
        name: String,
    },
    Parameter {
        name: String,
    },
    Binary {
        left: Box<ReactiveExpr>,
        op: BinaryOp,
        right: Box<ReactiveExpr>,
    },
    Unary {
        op: UnaryOp,
        operand: Box<ReactiveExpr>,
    },
}

impl ReactiveExpr {
    /// True when the expression needs no runtime data (fully folded).
    pub fn is_constant(&self) -> bool {
        matches!(self, ReactiveExpr::Constant(_))
    }

    /// Best-effort type of the expression, used for diagnostics.
    ///
    /// Comparisons yield `Bool`; arithmetic yields the left operand's type. This
    /// is deliberately approximate and never used to gate execution.
    pub fn ty(&self) -> Type {
        match self {
            ReactiveExpr::Constant(v) => v.get_type(),
            ReactiveExpr::Signal { ty, .. } => ty.as_type(),
            ReactiveExpr::Local { .. } | ReactiveExpr::Parameter { .. } => Type::Error,
            ReactiveExpr::Unary { operand, .. } => operand.ty(),
            ReactiveExpr::Binary { left, op, .. } => match op {
                BinaryOp::Gt | BinaryOp::Lt | BinaryOp::Gte | BinaryOp::Lte | BinaryOp::Eq
                | BinaryOp::Neq => Type::Bool,
                _ => left.ty(),
            },
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Reactive statements & program
// ───────────────────────────────────────────────────────────────────────────

/// A runtime-evaluable statement. Unlike [`crate::model::ResolvedStatement`],
/// control flow is preserved.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ReactiveStatement {
    /// A runtime `let` binding (only kept when its value is not constant).
    Let {
        name: String,
        value: ReactiveExpr,
    },
    Motion {
        kind: MotionKind,
        target: MotionTarget,
        provenance: Provenance,
    },
    Wait {
        seconds: f64,
        provenance: Provenance,
    },
    SetOutput {
        name: String,
        value: bool,
        provenance: Provenance,
    },
    /// A condition decided at runtime (e.g. on a signal). Never flattened.
    If {
        condition: ReactiveExpr,
        then_branch: Vec<ReactiveStatement>,
        else_branch: Option<Vec<ReactiveStatement>>,
        provenance: Provenance,
    },
    /// Block until a signal matches. Reserved for the reactive execution model;
    /// the current lowering does not yet produce it.
    WaitSignal {
        signal: SignalRef,
        provenance: Provenance,
    },
}

/// A control-flow-preserving, runtime-evaluable program.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReactiveProgram {
    pub entry_point: String,
    pub statements: Vec<ReactiveStatement>,
}

// ───────────────────────────────────────────────────────────────────────────
// Lowering
// ───────────────────────────────────────────────────────────────────────────

/// Lower a [`SemanticProgram`] into a [`ReactiveProgram`], preserving control
/// flow and typing signal reads against `signals`.
///
/// - A constant `if` condition is still folded (its chosen branch is inlined).
/// - A **non-constant** condition is preserved as [`ReactiveStatement::If`].
/// - A `module.channel` read must be declared in `signals`, else it is an error.
pub fn lower_reactive(
    program: &SemanticProgram,
    signals: &SignalSchema,
) -> Result<ReactiveProgram, Vec<String>> {
    let mut target_map: HashMap<String, MotionTarget> = HashMap::new();
    for target in &program.targets {
        target_map.insert(target.name.clone(), target.value.clone());
    }

    let mut fn_map: HashMap<String, &SemanticFunction> = HashMap::new();
    for func in &program.functions {
        fn_map.insert(func.name.clone(), func);
    }

    let entry = match fn_map.get(&program.entry_point) {
        Some(f) => *f,
        None => {
            return Err(vec![format!(
                "Entry point function '{}' not found",
                program.entry_point
            )]);
        }
    };

    let mut ctx = LowerCtx {
        targets: &target_map,
        functions: &fn_map,
        signals,
        out: Vec::new(),
        errors: Vec::new(),
        call_stack: Vec::new(),
    };
    let env = HashMap::new();
    let runtime = HashSet::new();
    ctx.lower_function(entry, &env, &runtime);

    if ctx.errors.is_empty() {
        Ok(ReactiveProgram {
            entry_point: program.entry_point.clone(),
            statements: ctx.out,
        })
    } else {
        Err(ctx.errors)
    }
}

struct LowerCtx<'a> {
    targets: &'a HashMap<String, MotionTarget>,
    functions: &'a HashMap<String, &'a SemanticFunction>,
    signals: &'a SignalSchema,
    out: Vec<ReactiveStatement>,
    errors: Vec<String>,
    call_stack: Vec<CallSite>,
}

impl<'a> LowerCtx<'a> {
    fn lower_function(
        &mut self,
        func: &SemanticFunction,
        env: &HashMap<String, CompileTimeValue>,
        runtime: &HashSet<String>,
    ) {
        self.call_stack.push(CallSite {
            function: func.name.clone(),
            span: func.provenance.span,
        });
        let mut local_env = env.clone();
        let mut local_runtime = runtime.clone();
        for stmt in &func.body {
            self.lower_statement(stmt, &mut local_env, &mut local_runtime);
        }
        self.call_stack.pop();
    }

    fn lower_statement(
        &mut self,
        stmt: &SemanticStatement,
        env: &mut HashMap<String, CompileTimeValue>,
        runtime: &mut HashSet<String>,
    ) {
        match stmt {
            SemanticStatement::Let { name, value, .. } => {
                if let Some(constant) = self.try_eval(value, env, runtime) {
                    env.insert(name.clone(), constant);
                } else {
                    match self.lower_expr(value, env, runtime) {
                        Ok(rexpr) => {
                            runtime.insert(name.clone());
                            self.out.push(ReactiveStatement::Let {
                                name: name.clone(),
                                value: rexpr,
                            });
                        }
                        Err(e) => self.errors.push(e),
                    }
                }
            }
            SemanticStatement::Motion(SemanticMotion {
                kind,
                target,
                provenance,
            }) => match self.try_eval(target, env, runtime) {
                Some(value) => match to_motion_target(&value) {
                    Some(target) => {
                        let mut prov = provenance.clone();
                        prov.call_stack = self.call_stack.clone();
                        self.out.push(ReactiveStatement::Motion {
                            kind: kind.clone(),
                            target,
                            provenance: prov,
                        });
                    }
                    None => self
                        .errors
                        .push("Expression does not resolve to a motion target".to_string()),
                },
                None => self
                    .errors
                    .push("Motion target is not resolvable at compile time".to_string()),
            },
            SemanticStatement::Wait { duration, provenance } => {
                match self.try_eval(duration, env, runtime) {
                    Some(CompileTimeValue::Duration(secs)) => {
                        let mut prov = provenance.clone();
                        prov.call_stack = self.call_stack.clone();
                        self.out.push(ReactiveStatement::Wait {
                            seconds: secs,
                            provenance: prov,
                        });
                    }
                    Some(other) => self
                        .errors
                        .push(format!("wait expected Duration, got {:?}", other.get_type())),
                    None => self
                        .errors
                        .push("wait duration is not resolvable at compile time".to_string()),
                }
            }
            SemanticStatement::SetOutput {
                name,
                value,
                provenance,
            } => {
                let mut prov = provenance.clone();
                prov.call_stack = self.call_stack.clone();
                self.out.push(ReactiveStatement::SetOutput {
                    name: name.clone(),
                    value: *value,
                    provenance: prov,
                });
            }
            SemanticStatement::If {
                condition,
                then_branch,
                else_branch,
                provenance,
            } => {
                match self.try_eval(condition, env, runtime) {
                    Some(CompileTimeValue::Bool(true)) => {
                        for s in then_branch {
                            self.lower_statement(s, env, runtime);
                        }
                    }
                    Some(CompileTimeValue::Bool(false)) => {
                        if let Some(stmts) = else_branch {
                            for s in stmts {
                                self.lower_statement(s, env, runtime);
                            }
                        }
                    }
                    _ => {
                        let cond = match self.lower_expr(condition, env, runtime) {
                            Ok(c) => c,
                            Err(e) => {
                                self.errors.push(e);
                                return;
                            }
                        };
                        let then_lowered =
                            self.lower_branch(then_branch, env, runtime);
                        let else_lowered = else_branch
                            .as_ref()
                            .map(|stmts| self.lower_branch(stmts, env, runtime));
                        let mut prov = provenance.clone();
                        prov.call_stack = self.call_stack.clone();
                        self.out.push(ReactiveStatement::If {
                            condition: cond,
                            then_branch: then_lowered,
                            else_branch: else_lowered,
                            provenance: prov,
                        });
                    }
                }
            }
            SemanticStatement::Call {
                function, args, ..
            } => self.lower_call(function, args, env, runtime),
            SemanticStatement::Expr(_) => {}
        }
    }

    /// Lower a branch in its own nested scope (env/runtime are forked and merged
    /// back, mirroring the resolver's inlining of local declarations).
    fn lower_branch(
        &mut self,
        stmts: &[SemanticStatement],
        env: &mut HashMap<String, CompileTimeValue>,
        runtime: &mut HashSet<String>,
    ) -> Vec<ReactiveStatement> {
        let mut branch_env = env.clone();
        let mut branch_runtime = runtime.clone();
        let saved_out = std::mem::take(&mut self.out);
        for s in stmts {
            self.lower_statement(s, &mut branch_env, &mut branch_runtime);
        }
        std::mem::replace(&mut self.out, saved_out)
    }

    fn lower_call(
        &mut self,
        function: &str,
        args: &[SemanticExpr],
        env: &mut HashMap<String, CompileTimeValue>,
        runtime: &mut HashSet<String>,
    ) {
        let Some(target_fn) = self.functions.get(function).copied() else {
            self.errors
                .push(format!("Unresolved function call '{function}'"));
            return;
        };
        if target_fn.params.len() != args.len() {
            self.errors.push(format!(
                "Function '{}' expected {} arguments, got {}",
                function,
                target_fn.params.len(),
                args.len()
            ));
            return;
        }

        let mut call_env = HashMap::new();
        let mut call_runtime = HashSet::new();
        for (param, arg) in target_fn.params.iter().zip(args) {
            if let Some(constant) = self.try_eval(arg, env, runtime) {
                call_env.insert(param.clone(), constant);
            } else {
                match self.lower_expr(arg, env, runtime) {
                    Ok(rexpr) => {
                        call_runtime.insert(param.clone());
                        let mut prov = Provenance::new(Some(function.to_string()), None);
                        prov.call_stack = self.call_stack.clone();
                        self.out.push(ReactiveStatement::Let {
                            name: param.clone(),
                            value: rexpr,
                        });
                    }
                    Err(e) => self.errors.push(e),
                }
            }
        }
        self.lower_function(target_fn, &call_env, &call_runtime);
    }

    /// Compile-time evaluation, returning `None` when the expression needs
    /// runtime data (a signal) or cannot be folded.
    fn try_eval(
        &self,
        expr: &SemanticExpr,
        env: &HashMap<String, CompileTimeValue>,
        runtime: &HashSet<String>,
    ) -> Option<CompileTimeValue> {
        match expr {
            SemanticExpr::Constant(v) => Some(v.clone()),
            SemanticExpr::ConstRef(name)
            | SemanticExpr::LocalRef(name)
            | SemanticExpr::ParameterRef(name) => {
                if runtime.contains(name) {
                    None
                } else {
                    env.get(name).cloned()
                }
            }
            SemanticExpr::TargetRef(name) => {
                self.targets.get(name).map(motion_target_to_value)
            }
            SemanticExpr::Binary { left, op, right } => {
                let lhs = self.try_eval(left, env, runtime)?;
                let rhs = self.try_eval(right, env, runtime)?;
                crate::algebra::binary_value(*op, &lhs, &rhs).ok()
            }
            SemanticExpr::Unary {
                op: UnaryOp::Neg,
                operand,
            } => {
                let value = self.try_eval(operand, env, runtime)?;
                negate_value(&value)
            }
            SemanticExpr::MemberAccess { object, member } => {
                let value = self.try_eval(object, env, runtime)?;
                read_member(&value, member)
            }
            // Signal reads and everything else are runtime.
            _ => None,
        }
    }

    fn lower_expr(
        &self,
        expr: &SemanticExpr,
        env: &HashMap<String, CompileTimeValue>,
        runtime: &HashSet<String>,
    ) -> Result<ReactiveExpr, String> {
        if let Some(constant) = self.try_eval(expr, env, runtime) {
            return Ok(ReactiveExpr::Constant(constant));
        }
        match expr {
            SemanticExpr::ChannelAccess { module, channel } => {
                self.signal(module, channel)
            }
            // The current front-end lowers `module.channel` to `MemberAccess`
            // whose receiver is the module identifier. Treat a receiver that is a
            // known module as a signal read, so both shapes land here.
            SemanticExpr::MemberAccess { object, member } => match object.as_ref() {
                SemanticExpr::ParameterRef(m) | SemanticExpr::ConstRef(m) | SemanticExpr::LocalRef(m)
                    if self.signals.knows_module(m) =>
                {
                    self.signal(m, member)
                }
                _ => Err("Runtime member access is not supported yet".to_string()),
            },
            SemanticExpr::Binary { left, op, right } => Ok(ReactiveExpr::Binary {
                left: Box::new(self.lower_expr(left, env, runtime)?),
                op: *op,
                right: Box::new(self.lower_expr(right, env, runtime)?),
            }),
            SemanticExpr::Unary {
                op: UnaryOp::Neg,
                operand,
            } => Ok(ReactiveExpr::Unary {
                op: UnaryOp::Neg,
                operand: Box::new(self.lower_expr(operand, env, runtime)?),
            }),
            SemanticExpr::ParameterRef(name) => Ok(ReactiveExpr::Parameter { name: name.clone() }),
            SemanticExpr::LocalRef(name) | SemanticExpr::ConstRef(name) => {
                Ok(ReactiveExpr::Local { name: name.clone() })
            }
            _ => Err("Expression is not runtime-evaluable".to_string()),
        }
    }

    fn signal(&self, module: &str, channel: &str) -> Result<ReactiveExpr, String> {
        match self.signals.type_of(module, channel) {
            Some(ty) => Ok(ReactiveExpr::Signal {
                module: module.to_string(),
                channel: channel.to_string(),
                ty,
            }),
            None => Err(format!("Unknown signal '{module}.{channel}'")),
        }
    }
}

fn motion_target_to_value(target: &MotionTarget) -> CompileTimeValue {
    match target {
        MotionTarget::Position(p) => CompileTimeValue::Position(p.clone()),
        MotionTarget::Pose(p) => CompileTimeValue::Pose(p.clone()),
        MotionTarget::Joints(j) => CompileTimeValue::Joints(j.values.clone()),
    }
}

fn to_motion_target(value: &CompileTimeValue) -> Option<MotionTarget> {
    match value {
        CompileTimeValue::Position(p) => Some(MotionTarget::Position(p.clone())),
        CompileTimeValue::Pose(p) => Some(MotionTarget::Pose(p.clone())),
        CompileTimeValue::Joints(j) => Some(MotionTarget::Joints(JointConfiguration::new(j.clone()))),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::SemanticFunction;

    fn fnp(name: &str, body: Vec<SemanticStatement>) -> SemanticFunction {
        SemanticFunction {
            name: name.to_string(),
            params: vec![],
            return_type: Type::Unit,
            body,
            tail_expr: None,
            provenance: Provenance::new(Some(name.to_string()), None),
        }
    }

    fn program(functions: Vec<SemanticFunction>) -> SemanticProgram {
        SemanticProgram {
            targets: vec![],
            functions,
            entry_point: "main".to_string(),
        }
    }

    fn channel(module: &str, chan: &str) -> SemanticExpr {
        SemanticExpr::ChannelAccess {
            module: module.to_string(),
            channel: chan.to_string(),
        }
    }

    fn boolean_condition(signal: SemanticExpr, value: bool) -> SemanticExpr {
        SemanticExpr::Binary {
            left: Box::new(signal),
            op: BinaryOp::Eq,
            right: Box::new(SemanticExpr::Constant(CompileTimeValue::Bool(value))),
        }
    }

    fn schema() -> SignalSchema {
        SignalSchema::new()
            .with("box_sensor", "present", SignalType::Bool)
            .with("temp", "value", SignalType::Float)
    }

    #[test]
    fn if_on_signal_is_preserved_and_typed() {
        let program = program(vec![fnp(
            "main",
            vec![SemanticStatement::If {
                condition: boolean_condition(channel("box_sensor", "present"), true),
                then_branch: vec![SemanticStatement::SetOutput {
                    name: "gripper".to_string(),
                    value: true,
                    provenance: Provenance::default(),
                }],
                else_branch: Some(vec![SemanticStatement::SetOutput {
                    name: "gripper".to_string(),
                    value: false,
                    provenance: Provenance::default(),
                }]),
                provenance: Provenance::default(),
            }],
        )]);

        let reactive = lower_reactive(&program, &schema()).expect("lowering succeeds");
        assert_eq!(reactive.statements.len(), 1);
        match &reactive.statements[0] {
            ReactiveStatement::If {
                condition,
                then_branch,
                else_branch,
                ..
            } => {
                assert_eq!(then_branch.len(), 1, "then branch preserved");
                assert_eq!(else_branch.as_ref().map(Vec::len), Some(1), "else branch preserved");
                match condition {
                    ReactiveExpr::Binary { left, .. } => match left.as_ref() {
                        ReactiveExpr::Signal { module, channel, ty } => {
                            assert_eq!(module, "box_sensor");
                            assert_eq!(channel, "present");
                            assert_eq!(*ty, SignalType::Bool);
                        }
                        other => panic!("expected signal, got {other:?}"),
                    },
                    other => panic!("expected binary condition, got {other:?}"),
                }
            }
            other => panic!("expected a preserved If, got {other:?}"),
        }
    }

    #[test]
    fn constant_if_is_folded_not_preserved() {
        let program = program(vec![fnp(
            "main",
            vec![SemanticStatement::If {
                condition: SemanticExpr::Constant(CompileTimeValue::Bool(true)),
                then_branch: vec![SemanticStatement::SetOutput {
                    name: "gripper".to_string(),
                    value: true,
                    provenance: Provenance::default(),
                }],
                else_branch: Some(vec![SemanticStatement::SetOutput {
                    name: "gripper".to_string(),
                    value: false,
                    provenance: Provenance::default(),
                }]),
                provenance: Provenance::default(),
            }],
        )]);

        let reactive = lower_reactive(&program, &schema()).expect("lowering succeeds");
        assert_eq!(reactive.statements.len(), 1);
        assert!(
            matches!(
                &reactive.statements[0],
                ReactiveStatement::SetOutput { value: true, .. }
            ),
            "constant condition folds to the taken branch, no If node"
        );
    }

    #[test]
    fn unknown_signal_is_an_error() {
        let program = program(vec![fnp(
            "main",
            vec![SemanticStatement::If {
                condition: boolean_condition(channel("ghost", "present"), true),
                then_branch: vec![],
                else_branch: None,
                provenance: Provenance::default(),
            }],
        )]);

        let err = lower_reactive(&program, &schema()).expect_err("unknown signal must fail");
        assert!(
            err.iter().any(|e| e.contains("Unknown signal 'ghost.present'")),
            "error mentions the unknown signal: {err:?}"
        );
    }

    #[test]
    fn signal_type_is_resolved_from_schema() {
        let program = program(vec![fnp(
            "main",
            vec![SemanticStatement::Let {
                name: "t".to_string(),
                value: channel("temp", "value"),
                provenance: Provenance::default(),
            }],
        )]);

        let reactive = lower_reactive(&program, &schema()).expect("lowering succeeds");
        match &reactive.statements[0] {
            ReactiveStatement::Let { value, .. } => assert_eq!(value.ty(), Type::Float),
            other => panic!("expected a runtime Let, got {other:?}"),
        }
    }

    #[test]
    fn constant_let_stays_compile_time() {
        let program = program(vec![fnp(
            "main",
            vec![SemanticStatement::Let {
                name: "k".to_string(),
                value: SemanticExpr::Constant(CompileTimeValue::Int(3)),
                provenance: Provenance::default(),
            }],
        )]);

        let reactive = lower_reactive(&program, &schema()).expect("lowering succeeds");
        assert!(reactive.statements.is_empty(), "constant let is not emitted");
    }
}
