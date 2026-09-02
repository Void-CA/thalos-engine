//! Full-Stack Integration Test for DSL Reactive Channel Acquisition.
//!
//! Validates:
//! 1. Parsing raw DSL text into Thalos AST containing `MemberAccess` (e.g. `camera.target_x`) and `If` statements.
//! 2. Lowering AST through `thalos-semantic::compiler::SemanticCompiler` into `SemanticProgram` with `ChannelAccess` and `SemanticStatement::If`.
//! 3. Evaluating compiled semantic IR against `ChannelRegistry` telemetric observations across ticks.
//! 4. Proving that telemetry snapshot updates dynamically alter control flow execution without recompiling the program.

use std::collections::HashMap;
use thalos_lang::parser::parse_source;
use thalos_ports::device::{ChannelObservation, ChannelValue, SignalQuality};
use thalos_semantic::compiler::SemanticCompiler;
use thalos_semantic::model::{
    MotionKind, SemanticExpr, SemanticMotion, SemanticStatement,
};

/// Trait defining an Acquisition Module that updates state snapshots in bulk.
pub trait AcquisitionModule: Send + Sync {
    fn id(&self) -> &str;
    fn update_snapshot(&mut self, values: HashMap<String, ChannelValue>);
    fn read_observation(&self, channel_name: &str) -> Option<ChannelObservation>;
}

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

/// Dynamic evaluator for compiled SemanticExpr using ChannelRegistry observations.
fn eval_semantic_expr(expr: &SemanticExpr, registry: &ChannelRegistry) -> Result<f64, String> {
    match expr {
        SemanticExpr::Constant(val) => match val {
            thalos_semantic::evaluator::CompileTimeValue::Float(f) => Ok(*f),
            thalos_semantic::evaluator::CompileTimeValue::Int(i) => Ok(*i as f64),
            _ => Err("Non-numeric constant".to_string()),
        },
        SemanticExpr::ChannelAccess { module, channel } => {
            let obs = registry.resolve_observation(module, channel)?;
            match obs.value {
                ChannelValue::Scalar(val) => Ok(val),
                _ => Err(format!("Channel {}.{} is not a scalar value", module, channel)),
            }
        }
        SemanticExpr::Binary { left, op, right } => {
            let l = eval_semantic_expr(left, registry)?;
            let r = eval_semantic_expr(right, registry)?;
            match op {
                thalos_lang::ast::BinaryOp::Gt => Ok(if l > r { 1.0 } else { 0.0 }),
                thalos_lang::ast::BinaryOp::Lt => Ok(if l < r { 1.0 } else { 0.0 }),
                thalos_lang::ast::BinaryOp::Gte => Ok(if l >= r { 1.0 } else { 0.0 }),
                thalos_lang::ast::BinaryOp::Lte => Ok(if l <= r { 1.0 } else { 0.0 }),
                thalos_lang::ast::BinaryOp::Eq => Ok(if (l - r).abs() < f64::EPSILON { 1.0 } else { 0.0 }),
                thalos_lang::ast::BinaryOp::Neq => Ok(if (l - r).abs() >= f64::EPSILON { 1.0 } else { 0.0 }),
                thalos_lang::ast::BinaryOp::Add => Ok(l + r),
                thalos_lang::ast::BinaryOp::Sub => Ok(l - r),
                thalos_lang::ast::BinaryOp::Mul => Ok(l * r),
                thalos_lang::ast::BinaryOp::Div => Ok(l / r),
            }
        }
        _ => Err(format!("Unhandled semantic expression: {:?}", expr)),
    }
}

/// Evaluates statements in SemanticProgram for a given execution tick.
fn execute_semantic_statements(
    stmts: &[SemanticStatement],
    registry: &ChannelRegistry,
    actions_out: &mut Vec<String>,
) -> Result<(), String> {
    for stmt in stmts {
        match stmt {
            SemanticStatement::If {
                condition,
                then_branch,
                else_branch,
                ..
            } => {
                let cond_val = eval_semantic_expr(condition, registry)?;
                if cond_val > 0.0 {
                    execute_semantic_statements(then_branch, registry, actions_out)?;
                } else if let Some(else_stmts) = else_branch {
                    execute_semantic_statements(else_stmts, registry, actions_out)?;
                }
            }
            SemanticStatement::Motion(SemanticMotion { kind, target, provenance }) => {
                let target_name = match target {
                    SemanticExpr::TargetRef(t) => t.clone(),
                    SemanticExpr::ParameterRef(p) => p.clone(),
                    _ => provenance.source_name.clone().unwrap_or_else(|| "unknown_target".to_string()),
                };
                let kind_str = match kind {
                    MotionKind::MoveJ => "movej",
                    MotionKind::MoveL => "movel",
                    MotionKind::MoveC { .. } => "movec",
                };
                actions_out.push(format!("{}({})", kind_str, target_name));
            }
            SemanticStatement::SetOutput { name, value, .. } => {
                actions_out.push(format!("set_output({}, {})", name, value));
            }
            _ => {}
        }
    }
    Ok(())
}

#[test]
fn test_dsl_reactive_acquisition_end_to_end() {
    // 1. Setup Acquisition Hardware Registry
    let mut registry = ChannelRegistry::new();
    let camera_module = FakeAcquisitionModule::new("camera", vec!["target_x", "target_y"]);
    registry.register_module(Box::new(camera_module));

    // 2. Define DSL Source Code
    let dsl_source = r#"
        target target_high = position([100mm, 200mm, 300mm]);
        target target_low = position([10mm, 20mm, 30mm]);

        fn main() {
            if camera.target_x > 80.0 {
                movej(target_high);
            } else {
                movej(target_low);
            }
        }
    "#;

    // 3. Parse DSL into AST
    let ast_file = parse_source(dsl_source).expect("DSL source should parse cleanly");
    assert_eq!(ast_file.items.len(), 3);

    // 4. Lower AST into Semantic Program
    let semantic_program = SemanticCompiler::compile(&ast_file).expect("Compilation to SemanticProgram should succeed");
    assert_eq!(semantic_program.functions.len(), 1);
    let main_fn = &semantic_program.functions[0];

    // Verify AST -> Semantic IR lowering produced a SemanticStatement::If
    assert!(matches!(main_fn.body.first(), Some(SemanticStatement::If { .. })));

    // 5. Tick 0: camera.target_x = 95.0 (> 80.0) -> Expects movej(target_high)
    let mut t0_telemetry = HashMap::new();
    t0_telemetry.insert("target_x".into(), ChannelValue::Scalar(95.0));
    t0_telemetry.insert("target_y".into(), ChannelValue::Scalar(20.0));
    registry.update_module_snapshot("camera", t0_telemetry);

    let mut t0_actions = Vec::new();
    execute_semantic_statements(&main_fn.body, &registry, &mut t0_actions).unwrap();
    assert_eq!(t0_actions, vec!["movej(target_high)"]);

    // 6. Tick 1: Telemetry updates dynamically: camera.target_x = 42.0 (<= 80.0) -> Expects movej(target_low)
    let mut t1_telemetry = HashMap::new();
    t1_telemetry.insert("target_x".into(), ChannelValue::Scalar(42.0));
    t1_telemetry.insert("target_y".into(), ChannelValue::Scalar(20.0));
    registry.update_module_snapshot("camera", t1_telemetry);

    let mut t1_actions = Vec::new();
    execute_semantic_statements(&main_fn.body, &registry, &mut t1_actions).unwrap();
    assert_eq!(t1_actions, vec!["movej(target_low)"]);
}
