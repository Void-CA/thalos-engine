//! Reactive runtime evaluator — a pure program driver over [`ReactiveProgram`].
//!
//! It walks a control-flow-preserving program against live observations and
//! yields the **next operation** to execute, one step per call. It performs no
//! I/O and no planning: it resolves signal conditions, follows branches, binds
//! runtime `let`s, and blocks on `WaitSignal`. Turning a step into a concrete
//! robot command (IK, trajectory, dispatch) is the runtime's job.
//!
//! The tree is flattened once, at construction, into a small linear instruction
//! list with jump targets, so the cursor is a single program counter.

use std::collections::HashMap;

#[cfg(test)]
use thalos_core::device::{ChannelObservation, ChannelValue, SignalQuality};
use thalos_core::execution::session::{ObservationBundle, eval_channel_access};
use thalos_lang::ast::{BinaryOp, UnaryOp};

use crate::evaluator::CompileTimeValue;
use crate::model::{MotionKind, MotionTarget};
use crate::reactive::{ReactiveExpr, ReactiveProgram, ReactiveStatement, SignalRef, SignalType};

/// A value produced while evaluating a reactive program at runtime.
#[derive(Debug, Clone, PartialEq)]
pub enum RuntimeValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    /// A non-numeric constant carried through unchanged.
    Constant(CompileTimeValue),
    /// Not available this tick (missing signal or unbound name).
    Unknown,
}

impl RuntimeValue {
    /// Truthiness used by branch conditions. `Unknown` is `None` — never true.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            RuntimeValue::Bool(b) => Some(*b),
            RuntimeValue::Int(i) => Some(*i != 0),
            RuntimeValue::Float(f) => Some(*f != 0.0),
            RuntimeValue::Constant(_) | RuntimeValue::Unknown => None,
        }
    }

    pub(crate) fn as_f64(&self) -> Option<f64> {
        match self {
            RuntimeValue::Bool(b) => Some(if *b { 1.0 } else { 0.0 }),
            RuntimeValue::Int(i) => Some(*i as f64),
            RuntimeValue::Float(f) => Some(*f),
            RuntimeValue::Constant(_) | RuntimeValue::Unknown => None,
        }
    }

    pub(crate) fn from_constant(value: CompileTimeValue) -> Self {
        match value {
            CompileTimeValue::Bool(b) => RuntimeValue::Bool(b),
            CompileTimeValue::Int(i) => RuntimeValue::Int(i),
            CompileTimeValue::Float(f) => RuntimeValue::Float(f),
            other => RuntimeValue::Constant(other),
        }
    }
}

/// The next operation the driver wants executed.
#[derive(Debug, Clone, PartialEq)]
pub enum ReactiveStep {
    Motion { kind: MotionKind, target: MotionTarget },
    Wait { seconds: f64 },
    SetOutput { name: String, value: bool },
    /// The program is blocked on a signal that is not yet satisfied.
    WaitSignal { signal: SignalRef },
    /// The program reached its end.
    Completed,
}

#[derive(Debug, Clone)]
enum Instr {
    Let {
        name: String,
        value: ReactiveExpr,
    },
    Motion {
        kind: MotionKind,
        target: MotionTarget,
    },
    Wait {
        seconds: f64,
    },
    SetOutput {
        name: String,
        value: bool,
    },
    JumpIfFalse {
        condition: ReactiveExpr,
        target: usize,
    },
    Jump {
        target: usize,
    },
    WaitSignal {
        signal: SignalRef,
    },
    Halt,
}

/// A pure driver over a [`ReactiveProgram`].
pub struct ReactiveEvaluator {
    instructions: Vec<Instr>,
    pc: usize,
    locals: HashMap<String, RuntimeValue>,
}

impl ReactiveEvaluator {
    pub fn new(program: &ReactiveProgram) -> Self {
        let mut instructions = Vec::new();
        compile_statements(&program.statements, &mut instructions);
        instructions.push(Instr::Halt);
        Self {
            instructions,
            pc: 0,
            locals: HashMap::new(),
        }
    }

    /// True once the program has run to completion.
    pub fn is_completed(&self) -> bool {
        matches!(self.instructions.get(self.pc), Some(Instr::Halt))
    }

    /// Advance the program by one step against the current observations.
    ///
    /// Internal instructions (`Let`, `If`, `Jump`) are processed transparently;
    /// the returned [`ReactiveStep`] is the next operation for the runtime, or
    /// [`ReactiveStep::Completed`].
    pub fn step(&mut self, observations: &ObservationBundle) -> ReactiveStep {
        loop {
            let instr = match self.instructions.get(self.pc) {
                Some(instr) => instr.clone(),
                None => return ReactiveStep::Completed,
            };
            match instr {
                Instr::Halt => return ReactiveStep::Completed,
                Instr::Let { name, value } => {
                    let evaluated = self.eval(&value, observations);
                    self.locals.insert(name, evaluated);
                    self.pc += 1;
                }
                Instr::JumpIfFalse { condition, target } => {
                    // Unknown is NOT satisfied: an unresolved condition takes the
                    // false path (never a silent "true").
                    let goes_to_else = !matches!(self.eval(&condition, observations).as_bool(), Some(true));
                    self.pc = if goes_to_else { target } else { self.pc + 1 };
                }
                Instr::Jump { target } => {
                    self.pc = target;
                }
                Instr::WaitSignal { signal } => {
                    if self.signal_truthy(&signal, observations) {
                        self.pc += 1;
                    } else {
                        return ReactiveStep::WaitSignal { signal };
                    }
                }
                Instr::Motion { kind, target } => {
                    self.pc += 1;
                    return ReactiveStep::Motion { kind, target };
                }
                Instr::Wait { seconds } => {
                    self.pc += 1;
                    return ReactiveStep::Wait { seconds };
                }
                Instr::SetOutput { name, value } => {
                    self.pc += 1;
                    return ReactiveStep::SetOutput { name, value };
                }
            }
        }
    }

    fn signal_truthy(&self, signal: &SignalRef, observations: &ObservationBundle) -> bool {
        match eval_channel_access(&signal.module, &signal.channel, observations) {
            Ok(value) => value != 0.0,
            Err(_) => false,
        }
    }

    fn eval(&self, expr: &ReactiveExpr, observations: &ObservationBundle) -> RuntimeValue {
        match expr {
            ReactiveExpr::Constant(value) => RuntimeValue::from_constant(value.clone()),
            ReactiveExpr::Signal { module, channel, ty } => {
                match eval_channel_access(module, channel, observations) {
                    Ok(value) => match ty {
                        SignalType::Bool => RuntimeValue::Bool(value != 0.0),
                        SignalType::Int => RuntimeValue::Int(value as i64),
                        SignalType::Float => RuntimeValue::Float(value),
                    },
                    Err(_) => RuntimeValue::Unknown,
                }
            }
            ReactiveExpr::Local { name } | ReactiveExpr::Parameter { name } => {
                self.locals.get(name).cloned().unwrap_or(RuntimeValue::Unknown)
            }
            ReactiveExpr::Unary {
                op: UnaryOp::Neg,
                operand,
            } => match self.eval(operand, observations) {
                RuntimeValue::Int(i) => RuntimeValue::Int(-i),
                RuntimeValue::Float(f) => RuntimeValue::Float(-f),
                other => other,
            },
            ReactiveExpr::Binary { left, op, right } => {
                let lhs = self.eval(left, observations);
                let rhs = self.eval(right, observations);
                Self::apply_binary(*op, lhs, rhs)
            }
        }
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
}

fn compile_statements(statements: &[ReactiveStatement], out: &mut Vec<Instr>) {
    for statement in statements {
        match statement {
            ReactiveStatement::Let { name, value } => out.push(Instr::Let {
                name: name.clone(),
                value: value.clone(),
            }),
            ReactiveStatement::Motion { kind, target, .. } => out.push(Instr::Motion {
                kind: kind.clone(),
                target: target.clone(),
            }),
            ReactiveStatement::Wait { seconds, .. } => out.push(Instr::Wait { seconds: *seconds }),
            ReactiveStatement::SetOutput { name, value, .. } => out.push(Instr::SetOutput {
                name: name.clone(),
                value: *value,
            }),
            ReactiveStatement::WaitSignal { signal, .. } => {
                out.push(Instr::WaitSignal { signal: signal.clone() })
            }
            ReactiveStatement::If {
                condition,
                then_branch,
                else_branch,
                ..
            } => {
                let jump_if_false = out.len();
                out.push(Instr::JumpIfFalse {
                    condition: condition.clone(),
                    target: usize::MAX,
                });
                compile_statements(then_branch, out);
                if let Some(else_body) = else_branch {
                    let jump_over_else = out.len();
                    out.push(Instr::Jump { target: usize::MAX });
                    let else_start = out.len();
                    if let Instr::JumpIfFalse { target, .. } = &mut out[jump_if_false] {
                        *target = else_start;
                    }
                    compile_statements(else_body, out);
                    let end = out.len();
                    if let Instr::Jump { target } = &mut out[jump_over_else] {
                        *target = end;
                    }
                } else {
                    let end = out.len();
                    if let Instr::JumpIfFalse { target, .. } = &mut out[jump_if_false] {
                        *target = end;
                    }
                }
            }
        }
    }
}

/// Build an `ObservationBundle` from `(channel, scalar)` pairs. Test helper.
#[cfg(test)]
fn observations(pairs: &[(&str, f64)]) -> ObservationBundle {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{JointConfiguration, Provenance};

    fn bool_signal(module: &str, channel: &str) -> ReactiveExpr {
        ReactiveExpr::Signal {
            module: module.to_string(),
            channel: channel.to_string(),
            ty: SignalType::Bool,
        }
    }

    fn set_output(value: bool) -> ReactiveStatement {
        ReactiveStatement::SetOutput {
            name: "gripper".to_string(),
            value,
            provenance: Provenance::default(),
        }
    }

    fn joints_motion() -> ReactiveStatement {
        ReactiveStatement::Motion {
            kind: MotionKind::MoveJ,
            target: MotionTarget::Joints(JointConfiguration::new(vec![0.0])),
            provenance: Provenance::default(),
        }
    }

    fn program(statements: Vec<ReactiveStatement>) -> ReactiveProgram {
        ReactiveProgram {
            entry_point: "main".to_string(),
            statements,
        }
    }

    #[test]
    fn if_selects_then_branch_when_signal_true() {
        let program = program(vec![
            ReactiveStatement::If {
                condition: bool_signal("box", "present"),
                then_branch: vec![set_output(true)],
                else_branch: Some(vec![set_output(false)]),
                provenance: Provenance::default(),
            },
            joints_motion(),
        ]);
        let mut evaluator = ReactiveEvaluator::new(&program);

        assert_eq!(
            evaluator.step(&observations(&[("present", 1.0)])),
            ReactiveStep::SetOutput {
                name: "gripper".to_string(),
                value: true
            }
        );
        assert!(matches!(
            evaluator.step(&observations(&[("present", 1.0)])),
            ReactiveStep::Motion { .. }
        ));
        assert_eq!(evaluator.step(&observations(&[])), ReactiveStep::Completed);
    }

    #[test]
    fn if_selects_else_branch_when_signal_false() {
        let program = program(vec![ReactiveStatement::If {
            condition: bool_signal("box", "present"),
            then_branch: vec![set_output(true)],
            else_branch: Some(vec![set_output(false)]),
            provenance: Provenance::default(),
        }]);
        let mut evaluator = ReactiveEvaluator::new(&program);
        assert_eq!(
            evaluator.step(&observations(&[("present", 0.0)])),
            ReactiveStep::SetOutput {
                name: "gripper".to_string(),
                value: false
            }
        );
    }

    #[test]
    fn unknown_signal_takes_false_path_not_true() {
        let program = program(vec![ReactiveStatement::If {
            condition: bool_signal("box", "present"),
            then_branch: vec![set_output(true)],
            else_branch: Some(vec![set_output(false)]),
            provenance: Provenance::default(),
        }]);
        let mut evaluator = ReactiveEvaluator::new(&program);
        // No observation for `present` → Unknown → NOT satisfied → else branch.
        assert_eq!(
            evaluator.step(&observations(&[])),
            ReactiveStep::SetOutput {
                name: "gripper".to_string(),
                value: false
            }
        );
    }

    #[test]
    fn wait_signal_blocks_until_satisfied() {
        let program = program(vec![
            ReactiveStatement::WaitSignal {
                signal: SignalRef {
                    module: "box".to_string(),
                    channel: "present".to_string(),
                },
                provenance: Provenance::default(),
            },
            set_output(true),
        ]);
        let mut evaluator = ReactiveEvaluator::new(&program);

        assert!(matches!(
            evaluator.step(&observations(&[("present", 0.0)])),
            ReactiveStep::WaitSignal { .. }
        ));
        assert_eq!(
            evaluator.step(&observations(&[("present", 1.0)])),
            ReactiveStep::SetOutput {
                name: "gripper".to_string(),
                value: true
            }
        );
    }

    #[test]
    fn runtime_let_binds_and_is_read_by_condition() {
        // let t = temp.value; if t > 50 { gripper = true }
        let program = program(vec![
            ReactiveStatement::Let {
                name: "t".to_string(),
                value: ReactiveExpr::Signal {
                    module: "temp".to_string(),
                    channel: "value".to_string(),
                    ty: SignalType::Float,
                },
            },
            ReactiveStatement::If {
                condition: ReactiveExpr::Binary {
                    left: Box::new(ReactiveExpr::Local {
                        name: "t".to_string(),
                    }),
                    op: BinaryOp::Gt,
                    right: Box::new(ReactiveExpr::Constant(CompileTimeValue::Float(50.0))),
                },
                then_branch: vec![set_output(true)],
                else_branch: Some(vec![set_output(false)]),
                provenance: Provenance::default(),
            },
        ]);
        let mut evaluator = ReactiveEvaluator::new(&program);
        assert_eq!(
            evaluator.step(&observations(&[("value", 60.0)])),
            ReactiveStep::SetOutput {
                name: "gripper".to_string(),
                value: true
            }
        );
    }
}
