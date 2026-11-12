//! Non-fatal compiler diagnostics.
//!
//! The compiler consumes an unspanned AST, so warnings here carry no source
//! location. The editor produces the same `THL_LEGACY_CONSTRUCTOR` code with
//! precise spans through the spanned tree.

use thalos_lang::ast::{Expr, Item, Program, Statement};

/// A non-fatal compiler diagnostic.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompilerWarning {
    pub code: &'static str,
    pub message: String,
}

/// Warnings for deprecated constructor aliases in a normalized program.
pub(crate) fn collect_legacy_warnings(program: &Program) -> Vec<CompilerWarning> {
    let mut warnings = Vec::new();
    for item in &program.items {
        match item {
            Item::Const(constant) => collect_expr_warnings(&constant.value, &mut warnings),
            Item::Target(target) => collect_expr_warnings(&target.pose, &mut warnings),
            Item::Function(function) => {
                for statement in &function.body {
                    collect_statement_warnings(statement, &mut warnings);
                }
                if let Some(tail) = &function.tail_expr {
                    collect_expr_warnings(tail, &mut warnings);
                }
            }
            Item::Process(process) => {
                for item in &process.items {
                    match item {
                        thalos_lang::ast::ProcessItem::State(state) => {
                            collect_expr_warnings(&state.value, &mut warnings);
                        }
                        thalos_lang::ast::ProcessItem::On(on) => {
                            match &on.event {
                                thalos_lang::ast::Event::Rising(expr)
                                | thalos_lang::ast::Event::Falling(expr)
                                | thalos_lang::ast::Event::Level(expr) => {
                                    collect_expr_warnings(expr, &mut warnings);
                                }
                            }
                            for statement in &on.body {
                                collect_statement_warnings(statement, &mut warnings);
                            }
                        }
                        thalos_lang::ast::ProcessItem::Cycle(cycle) => {
                            for statement in &cycle.body {
                                collect_statement_warnings(statement, &mut warnings);
                            }
                        }
                    }
                }
            }
            Item::Work(work) => {
                for statement in &work.body {
                    collect_statement_warnings(statement, &mut warnings);
                }
            }
            Item::Use(_) | Item::Expect(_) => {}
        }
    }
    warnings
}

fn collect_statement_warnings(statement: &Statement, warnings: &mut Vec<CompilerWarning>) {
    match statement {
        Statement::Let { value, .. } => collect_expr_warnings(value, warnings),
        Statement::MoveJ { target } | Statement::MoveL { target } => {
            collect_expr_warnings(target, warnings)
        }
        Statement::MoveC { via, target } => {
            collect_expr_warnings(via, warnings);
            collect_expr_warnings(target, warnings);
        }
        Statement::Wait(expr) => collect_expr_warnings(expr, warnings),
        Statement::SetOutput { value, .. } => collect_expr_warnings(value, warnings),
        Statement::If {
            condition,
            then_branch,
            else_branch,
        } => {
            collect_expr_warnings(condition, warnings);
            for statement in then_branch {
                collect_statement_warnings(statement, warnings);
            }
            if let Some(branch) = else_branch {
                for statement in branch {
                    collect_statement_warnings(statement, warnings);
                }
            }
        }
        Statement::Request { args, .. } => {
            for arg in args {
                collect_expr_warnings(&arg.value, warnings);
            }
        }
        Statement::Assign { value, .. } => collect_expr_warnings(value, warnings),
        Statement::Start { .. } => {}
        Statement::Expr(expr) => collect_expr_warnings(expr, warnings),
    }
}

fn collect_expr_warnings(expr: &Expr, warnings: &mut Vec<CompilerWarning>) {
    match expr {
        Expr::Call { callee, args } => {
            if let Some(replacement) = crate::registry::registry().legacy_replacement(callee) {
                warnings.push(CompilerWarning {
                    code: "THL_LEGACY_CONSTRUCTOR",
                    message: format!(
                        "Constructor '{}' is deprecated; use '{}' instead",
                        callee, replacement
                    ),
                });
            }
            for arg in args {
                collect_expr_warnings(&arg.value, warnings);
            }
        }
        Expr::MemberCall { object, args, .. } => {
            collect_expr_warnings(object, warnings);
            for arg in args {
                collect_expr_warnings(&arg.value, warnings);
            }
        }
        Expr::MemberAccess { object, .. } => collect_expr_warnings(object, warnings),
        Expr::Vector3([x, y, z]) => {
            collect_expr_warnings(x, warnings);
            collect_expr_warnings(y, warnings);
            collect_expr_warnings(z, warnings);
        }
        Expr::Binary { left, right, .. } => {
            collect_expr_warnings(left, warnings);
            collect_expr_warnings(right, warnings);
        }
        Expr::Unary { operand, .. } => collect_expr_warnings(operand, warnings),
        Expr::Identifier(_)
        | Expr::Number(_)
        | Expr::StringLiteral(_)
        | Expr::Boolean(_)
        | Expr::Length(_)
        | Expr::Angle(_)
        | Expr::Duration(_) => {}
    }
}
