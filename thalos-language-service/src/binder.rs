//! Argument binding / canonicalization.
//!
//! Named arguments are resolved **once**, here, into canonical positional form.
//! Downstream stages (checker, lowering, evaluator, resolver) only ever see
//! positional arguments, so no stage re-implements named-argument semantics.
//!
//! ```text
//! source → parser → Arg{name, value} → binder → positional args → evaluator
//! ```
//!
//! For example `joints(j3 = 30deg, j1 = 10deg)` is rewritten to the positional
//! call `joints(10deg, 0deg, 30deg)` before the evaluator ever sees it.
//!
//! This increment handles `joints` (a self-contained, index-based schema). The
//! receiver-type-driven `MemberSchema` binding (`offset(receiver, x = ...)`,
//! member calls) extends this pass in a later increment.

use thalos_lang::ast::{Arg, ConstDecl, Expr, FnDecl, Item, Program, Statement, TargetDecl};
use thalos_lang::units::AngleRadians;

/// Rewrite every named-argument call in `program` into canonical positional
/// form, returning the rewritten program and any binding errors.
pub fn normalize_program(program: &Program) -> (Program, Vec<String>) {
    let mut errors = Vec::new();
    let items = program
        .items
        .iter()
        .map(|item| normalize_item(item, &mut errors))
        .collect();
    (Program { items }, errors)
}

fn normalize_item(item: &Item, errors: &mut Vec<String>) -> Item {
    match item {
        Item::Use(use_decl) => Item::Use(use_decl.clone()),
        Item::Const(c) => Item::Const(ConstDecl {
            name: c.name.clone(),
            type_ann: c.type_ann.clone(),
            value: normalize_expr(&c.value, errors),
        }),
        Item::Target(t) => Item::Target(TargetDecl {
            name: t.name.clone(),
            pose: normalize_expr(&t.pose, errors),
        }),
        Item::Function(f) => Item::Function(FnDecl {
            name: f.name.clone(),
            params: f.params.clone(),
            return_type: f.return_type.clone(),
            body: f
                .body
                .iter()
                .map(|stmt| normalize_statement(stmt, errors))
                .collect(),
            tail_expr: f
                .tail_expr
                .as_ref()
                .map(|expr| Box::new(normalize_expr(expr, errors))),
        }),
    }
}

fn normalize_statement(stmt: &Statement, errors: &mut Vec<String>) -> Statement {
    match stmt {
        Statement::Let {
            name,
            type_ann,
            value,
        } => Statement::Let {
            name: name.clone(),
            type_ann: type_ann.clone(),
            value: normalize_expr(value, errors),
        },
        Statement::MoveJ { target } => Statement::MoveJ {
            target: normalize_expr(target, errors),
        },
        Statement::MoveL { target } => Statement::MoveL {
            target: normalize_expr(target, errors),
        },
        Statement::MoveC { via, target } => Statement::MoveC {
            via: normalize_expr(via, errors),
            target: normalize_expr(target, errors),
        },
        Statement::Wait(expr) => Statement::Wait(normalize_expr(expr, errors)),
        Statement::SetOutput { output, value } => Statement::SetOutput {
            output: output.clone(),
            value: normalize_expr(value, errors),
        },
        Statement::If {
            condition,
            then_branch,
            else_branch,
        } => Statement::If {
            condition: normalize_expr(condition, errors),
            then_branch: then_branch
                .iter()
                .map(|s| normalize_statement(s, errors))
                .collect(),
            else_branch: else_branch.as_ref().map(|branch| {
                branch
                    .iter()
                    .map(|s| normalize_statement(s, errors))
                    .collect()
            }),
        },
        Statement::Expr(expr) => Statement::Expr(normalize_expr(expr, errors)),
    }
}

fn normalize_expr(expr: &Expr, errors: &mut Vec<String>) -> Expr {
    match expr {
        Expr::Vector3([x, y, z]) => Expr::Vector3([
            Box::new(normalize_expr(x, errors)),
            Box::new(normalize_expr(y, errors)),
            Box::new(normalize_expr(z, errors)),
        ]),
        Expr::Pose {
            position,
            orientation,
        } => Expr::Pose {
            position: Box::new(normalize_expr(position, errors)),
            orientation: Box::new(normalize_expr(orientation, errors)),
        },
        Expr::Binary { left, op, right } => Expr::Binary {
            left: Box::new(normalize_expr(left, errors)),
            op: *op,
            right: Box::new(normalize_expr(right, errors)),
        },
        Expr::Unary { op, operand } => Expr::Unary {
            op: *op,
            operand: Box::new(normalize_expr(operand, errors)),
        },
        Expr::Call { callee, args } => {
            let normalized = normalize_args(args, errors);
            normalize_call(callee, normalized, errors)
        }
        Expr::MemberCall {
            object,
            method,
            args,
        } => {
            let normalized = normalize_args(args, errors);
            if !crate::member_schema::is_member_operation(method) {
                errors.push(format!(
                    "Unknown member operation '{}' on '{}'",
                    method, object
                ));
                return Expr::MemberCall {
                    object: object.clone(),
                    method: method.clone(),
                    args: normalized,
                };
            }
            // `receiver.op(args)` desugars to `op(receiver, args)`; the operation
            // itself (e.g. `offset`) owns all its semantics.
            let mut call_args = Vec::with_capacity(normalized.len() + 1);
            call_args.push(Arg::positional(Expr::Identifier(object.clone())));
            call_args.extend(normalized);
            normalize_call(method, call_args, errors)
        }
        other => other.clone(),
    }
}

fn normalize_args(args: &[Arg], errors: &mut Vec<String>) -> Vec<Arg> {
    args.iter()
        .map(|arg| Arg {
            name: arg.name.clone(),
            value: normalize_expr(&arg.value, errors),
        })
        .collect()
}

fn normalize_call(callee: &str, args: Vec<Arg>, errors: &mut Vec<String>) -> Expr {
    if callee == "joints" {
        return normalize_joints(callee, args, errors);
    }
    if callee == "offset" {
        // `offset` binding is receiver-type-directed, so it cannot be resolved
        // here (types are not known yet). Named delta arguments are preserved
        // and canonicalized later by `crate::offset`, called from the checker
        // and the evaluator.
        return Expr::Call {
            callee: callee.to_string(),
            args,
        };
    }
    if args.iter().any(Arg::is_named) {
        errors.push(format!("Named arguments are not supported for '{}'", callee));
    }
    Expr::Call {
        callee: callee.to_string(),
        args: drop_names(args),
    }
}

/// Canonicalize a `joints(...)` call.
///
/// `jK` selects joint `K` (1-based); omitted joints default to a zero angle.
/// Mixing named and positional arguments is rejected.
fn normalize_joints(callee: &str, args: Vec<Arg>, errors: &mut Vec<String>) -> Expr {
    let any_named = args.iter().any(Arg::is_named);
    let any_positional = args.iter().any(|arg| !arg.is_named());

    if !any_named {
        return Expr::Call {
            callee: callee.to_string(),
            args,
        };
    }
    if any_positional {
        errors.push("joints(...) cannot mix named and positional arguments".to_string());
        return Expr::Call {
            callee: callee.to_string(),
            args: drop_names(args),
        };
    }

    let mut slots: Vec<Option<Expr>> = Vec::new();
    let mut seen: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for arg in &args {
        let name = arg.name.as_deref().unwrap_or_default();
        match crate::member_schema::joint_index(name) {
            Some(index) if index >= 1 => {
                if !seen.insert(index) {
                    errors.push(format!("Duplicate joint argument '{}'", name));
                }
                if slots.len() < index {
                    slots.resize(index, None);
                }
                slots[index - 1] = Some(arg.value.clone());
            }
            _ => errors.push(format!(
                "Unknown joint argument '{}'; expected j1, j2, ...",
                name
            )),
        }
    }

    let args = slots
        .into_iter()
        .map(|slot| Arg::positional(slot.unwrap_or(Expr::Angle(AngleRadians(0.0)))))
        .collect();
    Expr::Call {
        callee: callee.to_string(),
        args,
    }
}

fn drop_names(args: Vec<Arg>) -> Vec<Arg> {
    args.into_iter().map(|arg| Arg::positional(arg.value)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_lang::ast::{Arg, Expr};
    use thalos_lang::units::AngleRadians;

    fn angle(deg: f64) -> Expr {
        Expr::Angle(AngleRadians(deg.to_radians()))
    }

    fn joints_positional(args: Vec<Expr>) -> Expr {
        Expr::Call {
            callee: "joints".to_string(),
            args: args.into_iter().map(Arg::positional).collect(),
        }
    }

    fn normalize_single(expr: Expr) -> (Expr, Vec<String>) {
        let program = Program {
            items: vec![Item::Target(TargetDecl {
                name: "T".to_string(),
                pose: expr,
            })],
        };
        let (program, errors) = normalize_program(&program);
        let pose = match program.items.into_iter().next().unwrap() {
            Item::Target(t) => t.pose,
            _ => unreachable!(),
        };
        (pose, errors)
    }

    #[test]
    fn positional_joints_pass_through_unchanged() {
        let call = joints_positional(vec![angle(10.0), angle(20.0), angle(30.0)]);
        let (out, errors) = normalize_single(call.clone());
        assert!(errors.is_empty());
        assert_eq!(out, call);
    }

    #[test]
    fn named_joints_are_canonicalized_in_order_with_zero_defaults() {
        let call = Expr::Call {
            callee: "joints".to_string(),
            args: vec![
                Arg::named("j3", angle(30.0)),
                Arg::named("j1", angle(10.0)),
            ],
        };
        let (out, errors) = normalize_single(call);
        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
        assert_eq!(
            out,
            joints_positional(vec![angle(10.0), Expr::Angle(AngleRadians(0.0)), angle(30.0)])
        );
    }

    #[test]
    fn duplicate_named_joint_is_reported() {
        let call = Expr::Call {
            callee: "joints".to_string(),
            args: vec![
                Arg::named("j1", angle(10.0)),
                Arg::named("j1", angle(20.0)),
            ],
        };
        let (_, errors) = normalize_single(call);
        assert_eq!(errors, vec!["Duplicate joint argument 'j1'".to_string()]);
    }

    #[test]
    fn unknown_named_joint_is_reported() {
        let call = Expr::Call {
            callee: "joints".to_string(),
            args: vec![Arg::named("j0", angle(10.0))],
        };
        let (_, errors) = normalize_single(call);
        assert_eq!(
            errors,
            vec!["Unknown joint argument 'j0'; expected j1, j2, ...".to_string()]
        );
    }

    #[test]
    fn mixing_named_and_positional_joints_is_rejected() {
        let call = Expr::Call {
            callee: "joints".to_string(),
            args: vec![Arg::positional(angle(10.0)), Arg::named("j2", angle(20.0))],
        };
        let (_, errors) = normalize_single(call);
        assert_eq!(
            errors,
            vec!["joints(...) cannot mix named and positional arguments".to_string()]
        );
    }

    #[test]
    fn named_args_on_other_callees_are_rejected() {
        let call = Expr::Call {
            callee: "position".to_string(),
            args: vec![Arg::named("x", angle(10.0))],
        };
        let (_, errors) = normalize_single(call);
        assert_eq!(
            errors,
            vec!["Named arguments are not supported for 'position'".to_string()]
        );
    }
}
