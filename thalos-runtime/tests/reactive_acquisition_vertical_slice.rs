//! Vertical Slice Integration Test for Reactive Channel Acquisition and Control Flow.
//!
//! Validates:
//! 1. Module registration and bulk channel declaration (`FakeAcquisitionModule`).
//! 2. Symbol resolution (`"camera.target_x"` -> `ChannelRef("camera", "target_x")`).
//! 3. Tick sampling and dynamic expression evaluation (`target_x > 80.0`).
//! 4. Reactive execution branching (`if` condition evaluates dynamically as acquired state snapshot changes).
//! 5. Unknown channel rejection and type safety.

use std::collections::HashMap;
use thalos_ports::device::{ChannelObservation, ChannelValue, SignalQuality};

/// Trait defining an Acquisition Module that updates state snapshots in bulk.
pub trait AcquisitionModule: Send + Sync {
    fn id(&self) -> &str;
    fn declared_channels(&self) -> Vec<&str>;
    fn update_snapshot(&mut self, values: HashMap<String, ChannelValue>);
    fn read_observation(&self, channel_name: &str) -> Option<ChannelObservation>;
}

/// FakeAcquisitionModule implementation for zero-hardware testing.
pub struct FakeAcquisitionModule {
    id: String,
    channels: Vec<String>,
    snapshot: HashMap<String, ChannelObservation>,
}

impl FakeAcquisitionModule {
    pub fn new(id: &str, channels: Vec<&str>) -> Self {
        Self {
            id: id.to_string(),
            channels: channels.into_iter().map(|s| s.to_string()).collect(),
            snapshot: HashMap::new(),
        }
    }
}

impl AcquisitionModule for FakeAcquisitionModule {
    fn id(&self) -> &str {
        &self.id
    }

    fn declared_channels(&self) -> Vec<&str> {
        self.channels.iter().map(|s| s.as_str()).collect()
    }

    fn update_snapshot(&mut self, values: HashMap<String, ChannelValue>) {
        let now_ns = 1_000_000_000;
        for (channel_name, val) in values {
            if self.channels.contains(&channel_name) {
                let fq_id = format!("{}.{}", self.id, channel_name);
                self.snapshot.insert(
                    channel_name.clone(),
                    ChannelObservation {
                        channel_id: fq_id,
                        sampled_at_ns: now_ns,
                        received_at_ns: now_ns + 500,
                        value: val,
                        unit: Some("px".into()),
                        quality: SignalQuality::Nominal,
                    },
                );
            }
        }
    }

    fn read_observation(&self, channel_name: &str) -> Option<ChannelObservation> {
        self.snapshot.get(channel_name).cloned()
    }
}

/// Registry mapping "module.channel" symbols to AcquisitionModules.
pub struct ChannelRegistry {
    modules: HashMap<String, Box<dyn AcquisitionModule>>,
}

impl ChannelRegistry {
    pub fn new() -> Self {
        Self {
            modules: HashMap::new(),
        }
    }

    pub fn register_module(&mut self, module: Box<dyn AcquisitionModule>) {
        self.modules.insert(module.id().to_string(), module);
    }

    pub fn update_module_snapshot(&mut self, module_id: &str, values: HashMap<String, ChannelValue>) {
        if let Some(module) = self.modules.get_mut(module_id) {
            module.update_snapshot(values);
        }
    }

    pub fn resolve_observation(&self, module_name: &str, channel_name: &str) -> Result<ChannelObservation, String> {
        let module = self
            .modules
            .get(module_name)
            .ok_or_else(|| format!("Module '{}' not found", module_name))?;

        module
            .read_observation(channel_name)
            .ok_or_else(|| format!("Channel '{}.{}' not found or uninitialized", module_name, channel_name))
    }
}

/// Generic Expression AST node.
pub enum Expression {
    ChannelAccess { module: String, channel: String },
    LiteralScalar(f64),
    GreaterThan(Box<Expression>, Box<Expression>),
}

impl Expression {
    pub fn evaluate(&self, registry: &ChannelRegistry) -> Result<ChannelValue, String> {
        match self {
            Expression::ChannelAccess { module, channel } => {
                let obs = registry.resolve_observation(module, channel)?;
                Ok(obs.value)
            }
            Expression::LiteralScalar(v) => Ok(ChannelValue::Scalar(*v)),
            Expression::GreaterThan(left, right) => {
                let l_val = left.evaluate(registry)?;
                let r_val = right.evaluate(registry)?;

                match (l_val, r_val) {
                    (ChannelValue::Scalar(l), ChannelValue::Scalar(r)) => Ok(ChannelValue::Boolean(l > r)),
                    _ => Err("Type mismatch: GreaterThan expects numeric scalar values".into()),
                }
            }
        }
    }
}

/// Generic Statement AST node for reactive control flow.
pub enum Statement {
    If {
        condition: Expression,
        then_branch: Vec<Statement>,
        else_branch: Vec<Statement>,
    },
    RobotAction { action: String },
}

pub struct ExecutionEngine;

impl ExecutionEngine {
    pub fn execute_tick(stmt: &Statement, registry: &ChannelRegistry, log: &mut Vec<String>) -> Result<(), String> {
        match stmt {
            Statement::If {
                condition,
                then_branch,
                else_branch,
            } => {
                let cond_result = condition.evaluate(registry)?;
                if let ChannelValue::Boolean(cond_bool) = cond_result {
                    let branch = if cond_bool { then_branch } else { else_branch };
                    for sub_stmt in branch {
                        Self::execute_tick(sub_stmt, registry, log)?;
                    }
                    Ok(())
                } else {
                    Err("If condition must evaluate to a boolean".into())
                }
            }
            Statement::RobotAction { action } => {
                log.push(action.clone());
                Ok(())
            }
        }
    }
}

#[test]
fn test_reactive_acquisition_vertical_slice() {
    // 1. Module Registration
    let mut registry = ChannelRegistry::new();
    let camera_module = FakeAcquisitionModule::new("camera", vec!["target_x", "target_y"]);
    registry.register_module(Box::new(camera_module));

    // Initial state snapshot (t0): target_x = 100.0, target_y = 50.0
    let mut t0_values = HashMap::new();
    t0_values.insert("target_x".into(), ChannelValue::Scalar(100.0));
    t0_values.insert("target_y".into(), ChannelValue::Scalar(50.0));
    registry.update_module_snapshot("camera", t0_values);

    // 2. Build Reactive Program: if camera.target_x > 80.0 { move_to_target } else { wait_for_signal }
    let program = Statement::If {
        condition: Expression::GreaterThan(
            Box::new(Expression::ChannelAccess {
                module: "camera".into(),
                channel: "target_x".into(),
            }),
            Box::new(Expression::LiteralScalar(80.0)),
        ),
        then_branch: vec![Statement::RobotAction {
            action: "move_to_target".into(),
        }],
        else_branch: vec![Statement::RobotAction {
            action: "wait_for_signal".into(),
        }],
    };

    // 3. Tick 0 Evaluation: target_x = 100.0 > 80.0 -> true -> "move_to_target"
    let mut log_t0 = Vec::new();
    ExecutionEngine::execute_tick(&program, &registry, &mut log_t0).unwrap();
    assert_eq!(log_t0, vec!["move_to_target"]);

    // 4. Tick 1 Evaluation: Acquired state changes dynamically (target_x = 50.0)
    let mut t1_values = HashMap::new();
    t1_values.insert("target_x".into(), ChannelValue::Scalar(50.0));
    t1_values.insert("target_y".into(), ChannelValue::Scalar(50.0));
    registry.update_module_snapshot("camera", t1_values);

    let mut log_t1 = Vec::new();
    ExecutionEngine::execute_tick(&program, &registry, &mut log_t1).unwrap();
    assert_eq!(log_t1, vec!["wait_for_signal"]);

    // 5. Unregistered Channel Rejection Error
    let invalid_program = Statement::If {
        condition: Expression::GreaterThan(
            Box::new(Expression::ChannelAccess {
                module: "camera".into(),
                channel: "non_existent_channel".into(),
            }),
            Box::new(Expression::LiteralScalar(80.0)),
        ),
        then_branch: vec![],
        else_branch: vec![],
    };

    let mut err_log = Vec::new();
    let err = ExecutionEngine::execute_tick(&invalid_program, &registry, &mut err_log).unwrap_err();
    assert!(err.contains("not found or uninitialized"));
}
