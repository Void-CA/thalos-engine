use std::collections::HashMap;
use thalos_lang::ast::BinaryOp;
use crate::evaluator::{CompileTimeValue, Position};
use crate::model::{
    CallSite, JointConfiguration, MotionTarget, ResolvedMotion, ResolvedProgram,
    ResolvedStatement, SemanticExpr, SemanticFunction, SemanticMotion, SemanticProgram,
    SemanticStatement,
};

pub struct SemanticResolver;

impl SemanticResolver {
    pub fn resolve(program: &SemanticProgram) -> Result<ResolvedProgram, Vec<String>> {
        let mut target_map: HashMap<String, MotionTarget> = HashMap::new();
        for target in &program.targets {
            target_map.insert(target.name.clone(), target.value.clone());
        }

        let mut fn_map: HashMap<String, &SemanticFunction> = HashMap::new();
        for func in &program.functions {
            fn_map.insert(func.name.clone(), func);
        }

        let entry_fn = match fn_map.get(&program.entry_point) {
            Some(f) => f,
            None => return Err(vec![format!("Entry point function '{}' not found", program.entry_point)]),
        };

        let mut resolved_statements = Vec::new();
        let mut errors = Vec::new();
        let mut call_stack = Vec::new();
        let env = HashMap::new();

        Self::resolve_function(
            entry_fn,
            &env,
            &target_map,
            &fn_map,
            &mut call_stack,
            &mut resolved_statements,
            &mut errors,
        );

        if !errors.is_empty() {
            Err(errors)
        } else {
            Ok(ResolvedProgram {
                statements: resolved_statements,
            })
        }
    }

    fn resolve_function(
        func: &SemanticFunction,
        env: &HashMap<String, CompileTimeValue>,
        targets: &HashMap<String, MotionTarget>,
        functions: &HashMap<String, &SemanticFunction>,
        call_stack: &mut Vec<CallSite>,
        out: &mut Vec<ResolvedStatement>,
        errors: &mut Vec<String>,
    ) {
        call_stack.push(CallSite {
            function: func.name.clone(),
            span: func.provenance.span.clone(),
        });

        for stmt in &func.body {
            match stmt {
                SemanticStatement::Motion(SemanticMotion { kind, target, provenance }) => {
                    match Self::eval_expr(target, env, targets) {
                        Ok(target_val) => {
                            let mut merged_prov = provenance.clone();
                            merged_prov.call_stack = call_stack.clone();
                            out.push(ResolvedStatement::Motion(ResolvedMotion {
                                kind: kind.clone(),
                                target: target_val,
                                provenance: merged_prov,
                            }));
                        }
                        Err(err) => errors.push(err),
                    }
                }
                SemanticStatement::Wait { duration, provenance } => {
                    if let Ok(secs) = Self::eval_scalar(duration, env) {
                        let mut merged_prov = provenance.clone();
                        merged_prov.call_stack = call_stack.clone();
                        out.push(ResolvedStatement::Wait {
                            seconds: secs,
                            provenance: merged_prov,
                        });
                    } else {
                        errors.push("Could not evaluate wait duration".to_string());
                    }
                }
                SemanticStatement::SetOutput { name, value, provenance } => {
                    let mut merged_prov = provenance.clone();
                    merged_prov.call_stack = call_stack.clone();
                    out.push(ResolvedStatement::SetOutput {
                        name: name.clone(),
                        value: *value,
                        provenance: merged_prov,
                    });
                }
                SemanticStatement::Call { function, args, provenance: _ } => {
                    if let Some(target_fn) = functions.get(function) {
                        let mut arg_vals = Vec::new();
                        for arg in args {
                            match Self::eval_value(arg, env, targets) {
                                Ok(val) => arg_vals.push(val),
                                Err(e) => errors.push(e),
                            }
                        }

                        if target_fn.params.len() != arg_vals.len() {
                            errors.push(format!(
                                "Function '{}' expected {} arguments, got {}",
                                function,
                                target_fn.params.len(),
                                arg_vals.len()
                            ));
                            continue;
                        }

                        let mut new_env = HashMap::new();
                        for (param, val) in target_fn.params.iter().zip(arg_vals) {
                            new_env.insert(param.clone(), val);
                        }

                        Self::resolve_function(
                            target_fn,
                            &new_env,
                            targets,
                            functions,
                            call_stack,
                            out,
                            errors,
                        );
                    } else {
                        errors.push(format!("Unresolved function call '{}'", function));
                    }
                }
            }
        }

        call_stack.pop();
    }

    fn eval_value(
        expr: &SemanticExpr,
        env: &HashMap<String, CompileTimeValue>,
        targets: &HashMap<String, MotionTarget>,
    ) -> Result<CompileTimeValue, String> {
        match expr {
            SemanticExpr::Constant(val) => Ok(val.clone()),
            SemanticExpr::ParameterRef(p) => {
                if let Some(val) = env.get(p) {
                    Ok(val.clone())
                } else {
                    Err(format!("Unbound parameter '{}'", p))
                }
            }
            SemanticExpr::TargetRef(t) => {
                if let Some(target_val) = targets.get(t) {
                    match target_val {
                        MotionTarget::Position(p) => Ok(CompileTimeValue::Position(p.clone())),
                        MotionTarget::Pose(p) => Ok(CompileTimeValue::Pose(p.clone())),
                        MotionTarget::Joints(j) => Ok(CompileTimeValue::Joints(j.values.clone())),
                    }
                } else {
                    Err(format!("Unresolved target reference '{}'", t))
                }
            }
            _ => Err("Complex expression resolution not supported yet".to_string()),
        }
    }

    fn eval_scalar(expr: &SemanticExpr, env: &HashMap<String, CompileTimeValue>) -> Result<f64, String> {
        match expr {
            SemanticExpr::Constant(CompileTimeValue::Duration(d)) => Ok(*d),
            SemanticExpr::Constant(CompileTimeValue::Float(f)) => Ok(*f),
            SemanticExpr::Constant(CompileTimeValue::Int(i)) => Ok(*i as f64),
            SemanticExpr::ParameterRef(p) => {
                if let Some(val) = env.get(p) {
                    match val {
                        CompileTimeValue::Duration(d) => Ok(*d),
                        CompileTimeValue::Float(f) => Ok(*f),
                        CompileTimeValue::Int(i) => Ok(*i as f64),
                        _ => Err(format!("Parameter '{}' is not a scalar number", p)),
                    }
                } else {
                    Err(format!("Unbound parameter '{}'", p))
                }
            }
            _ => Err("Could not resolve scalar expression".to_string()),
        }
    }

    fn eval_expr(
        expr: &SemanticExpr,
        env: &HashMap<String, CompileTimeValue>,
        targets: &HashMap<String, MotionTarget>,
    ) -> Result<MotionTarget, String> {
        match expr {
            SemanticExpr::Constant(CompileTimeValue::Position(p)) => Ok(MotionTarget::Position(p.clone())),
            SemanticExpr::Constant(CompileTimeValue::Pose(p)) => Ok(MotionTarget::Pose(p.clone())),
            SemanticExpr::Constant(CompileTimeValue::Joints(j)) => Ok(MotionTarget::Joints(JointConfiguration::new(j.clone()))),
            SemanticExpr::TargetRef(name) => {
                if let Some(t) = targets.get(name) {
                    Ok(t.clone())
                } else {
                    Err(format!("Unresolved target reference '{}'", name))
                }
            }
            SemanticExpr::ParameterRef(name) => {
                if let Some(val) = env.get(name) {
                    match val {
                        CompileTimeValue::Position(p) => Ok(MotionTarget::Position(p.clone())),
                        CompileTimeValue::Pose(p) => Ok(MotionTarget::Pose(p.clone())),
                        CompileTimeValue::Joints(j) => Ok(MotionTarget::Joints(JointConfiguration::new(j.clone()))),
                        _ => Err(format!("Parameter '{}' does not evaluate to a motion target", name)),
                    }
                } else {
                    Err(format!("Unbound parameter '{}'", name))
                }
            }
            SemanticExpr::Binary { left, op, right } => {
                let lhs = Self::eval_expr(left, env, targets)?;
                match (lhs, op, right.as_ref()) {
                    (MotionTarget::Position(p), BinaryOp::Add, SemanticExpr::Constant(CompileTimeValue::Vector3(v))) => {
                        Ok(MotionTarget::Position(Position { point: p.point + *v }))
                    }
                    (MotionTarget::Position(p), BinaryOp::Sub, SemanticExpr::Constant(CompileTimeValue::Vector3(v))) => {
                        Ok(MotionTarget::Position(Position { point: p.point - *v }))
                    }
                    _ => Err("Unsupported binary operation on motion target during resolution".to_string()),
                }
            }
            _ => Err("Expression does not resolve to a motion target".to_string()),
        }
    }
}
