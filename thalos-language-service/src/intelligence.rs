//! Type intelligence snapshot for editor tooling (hover / inlay hints).
//!
//! This module projects the checker's knowledge into a serializable, revision-
//! keyed snapshot. It deliberately exposes **structured type facts**, never
//! presentation text (engine invariant I1): the frontend decides how to render
//! a `Type` into markdown/tooltips.
//!
//! Pipeline:
//!
//! ```text
//! parse_source_spanned ──→ spans (char offsets)
//!        │ unspan
//!        ▼
//!  semantic AST ──→ prepare_symbols ──→ checker ──→ DocumentIntelligence
//! ```
//!
//! ## Offset discipline
//!
//! The spanned tree holds **character** offsets. This module performs the only
//! char → byte conversion (`char_range_to_byte_span`) before facts leave the
//! engine, so every consumer above this boundary deals exclusively in byte
//! offsets.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use thalos_lang::ast::spanned::{
    SpannedExpr, SpannedExprKind, SpannedItem, SpannedStatement, SpannedStatementKind,
};
use thalos_lang::ast::{Expr, Item, Statement};
use thalos_lang::parser::parse_source_spanned;
use thalos_lang::span::Span;

use crate::checker::TypeChecker;
use crate::compiler::{prepare_symbols, PreparedSymbols};
use crate::scope::ScopeKind;
use crate::symbols::{Symbol, SymbolKind as InternalSymbolKind};
use crate::types::{FunctionType, Type};
use crate::{char_range_to_byte_span, Diagnostic, DiagnosticSeverity, SourceSpan, SymbolKind};

/// Complete, revision-keyed analysis snapshot consumed by the editor.
///
/// The frontend treats it as immutable: a response for revision N is discarded
/// if a newer revision has already been received.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DocumentIntelligence {
    pub revision: u64,
    /// Named bindings (targets, consts, functions, parameters, locals) with types.
    pub symbols: Vec<TypedSymbol>,
    /// Inlay hints for declarations/parameters.
    pub hints: Vec<InlayHint>,
    /// Every expression node with its inferred type and span (hover lookup).
    pub expressions: Vec<TypedExpression>,
    /// Function signatures (builtins + user functions), grouped by name.
    pub signatures: Vec<SignatureInfo>,
    /// Parser + semantic diagnostics.
    pub diagnostics: Vec<Diagnostic>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TypedSymbol {
    pub name: String,
    pub kind: SymbolKind,
    pub ty: Type,
    pub span: SourceSpan,
    pub name_span: SourceSpan,
    pub detail: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InlayHint {
    pub name: String,
    pub kind: SymbolKind,
    pub ty: Type,
    pub span: SourceSpan,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TypedExpression {
    pub ty: Type,
    pub span: SourceSpan,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignatureInfo {
    pub name: String,
    pub overloads: Vec<FunctionType>,
}

impl DocumentIntelligence {
    fn empty(revision: u64, diagnostics: Vec<Diagnostic>) -> Self {
        Self {
            revision,
            symbols: Vec::new(),
            hints: Vec::new(),
            expressions: Vec::new(),
            signatures: Vec::new(),
            diagnostics,
        }
    }
}

/// Build the tooling intelligence snapshot for `source` at `revision`.
pub fn analyze_intelligence(source: &str, revision: u64) -> DocumentIntelligence {
    let spanned = match parse_source_spanned(source) {
        Ok(spanned) => spanned,
        Err(errors) => {
            let diagnostics = errors
                .into_iter()
                .map(|err| Diagnostic {
                    severity: DiagnosticSeverity::Error,
                    code: Some("THL_PARSER_ERROR".to_string()),
                    message: format!("{err}"),
                    span: byte_span(source, char_span(err.span())),
                })
                .collect();
            return DocumentIntelligence::empty(revision, diagnostics);
        }
    };

    let program = spanned.clone().unspan();
    // Resolve named arguments once, exactly like the compiler does, so tooling
    // and the semantic pipeline agree on the canonical argument order.
    let (program, bind_errors) = crate::binder::normalize_program(&program);
    let PreparedSymbols {
        mut table,
        errors: prepare_errors,
        ..
    } = prepare_symbols(&program);

    let mut symbols = Vec::new();
    let mut hints = Vec::new();
    let mut expressions = Vec::new();
    let mut diagnostics: Vec<Diagnostic> = prepare_errors
        .into_iter()
        .chain(bind_errors)
        .map(|message| semantic_diagnostic(source, message))
        .collect();

    let signatures = collect_signatures(&table);

    // Global declarations (targets + consts) carry their type in the table.
    for item in &spanned.items {
        match item {
            SpannedItem::Const(decl) => {
                record_global(
                    &table,
                    &mut symbols,
                    &mut hints,
                    source,
                    &decl.name,
                    SymbolKind::Const,
                    decl.span,
                    decl.name_span,
                );
            }
            SpannedItem::Target(decl) => {
                record_global(
                    &table,
                    &mut symbols,
                    &mut hints,
                    source,
                    &decl.name,
                    SymbolKind::Target,
                    decl.span,
                    decl.name_span,
                );
            }
            SpannedItem::Function(decl) => {
                symbols.push(TypedSymbol {
                    name: decl.name.clone(),
                    kind: SymbolKind::Function,
                    ty: Type::Function(function_type(&table, &decl.name).unwrap_or_else(|| {
                        FunctionType::new(Vec::new(), Type::Unit)
                    })),
                    span: byte_span(source, decl.span),
                    name_span: byte_span(source, decl.name_span),
                    detail: Some(format!("fn {}({} params)", decl.name, decl.params.len())),
                });
            }
            SpannedItem::Use { .. } => {}
        }
    }

    // Function bodies: check for diagnostics + collect expression/binding types.
    let mut checker = TypeChecker::new(&mut table);
    for (spanned_item, item) in spanned.items.iter().zip(program.items.iter()) {
        let (SpannedItem::Function(spanned_fn), Item::Function(fn_decl)) = (spanned_item, item)
        else {
            continue;
        };

        let expected_ret = fn_decl
            .return_type
            .as_deref()
            .and_then(Type::from_name)
            .unwrap_or(Type::Unit);

        checker.current_fn_name = Some(fn_decl.name.clone());
        checker.current_fn_return_type = Some(expected_ret.clone());
        checker.symbol_table.push_scope(ScopeKind::Function);

        for (spanned_param, param) in spanned_fn.params.iter().zip(fn_decl.params.iter()) {
            let param_ty = param
                .type_ann
                .as_deref()
                .and_then(Type::from_name)
                .unwrap_or(Type::Position);
            let _ = checker.symbol_table.declare(Symbol::new(
                param.name.clone(),
                InternalSymbolKind::Parameter,
                param_ty.clone(),
                None,
            ));
            hints.push(InlayHint {
                name: param.name.clone(),
                kind: SymbolKind::Parameter,
                ty: param_ty,
                span: byte_span(source, spanned_param.span),
            });
        }

        for (spanned_stmt, stmt) in spanned_fn.body.iter().zip(fn_decl.body.iter()) {
            checker.check_statement(stmt);
            collect_statement(
                spanned_stmt,
                stmt,
                &mut checker,
                source,
                &mut symbols,
                &mut hints,
                &mut expressions,
            );
        }

        if let (Some(spanned_tail), Some(tail)) =
            (spanned_fn.tail_expr.as_ref(), fn_decl.tail_expr.as_ref())
        {
            collect_expr(spanned_tail, tail, &mut checker, source, &mut expressions);
        }

        checker.symbol_table.pop_scope();
        checker.current_fn_name = None;
        checker.current_fn_return_type = None;
    }

    diagnostics.extend(
        checker
            .diagnostics
            .into_iter()
            .map(|diag| semantic_diagnostic(source, diag.message)),
    );

    DocumentIntelligence {
        revision,
        symbols,
        hints,
        expressions,
        signatures,
        diagnostics,
    }
}

#[allow(clippy::too_many_arguments)]
fn record_global(
    table: &crate::scope::SymbolTable,
    symbols: &mut Vec<TypedSymbol>,
    hints: &mut Vec<InlayHint>,
    source: &str,
    name: &str,
    kind: SymbolKind,
    span: Span,
    name_span: Span,
) {
    let ty = table
        .lookup(name)
        .and_then(|entries| entries.first())
        .map(|symbol| symbol.ty.clone())
        .unwrap_or(Type::Error);

    symbols.push(TypedSymbol {
        name: name.to_string(),
        kind,
        ty: ty.clone(),
        span: byte_span(source, span),
        name_span: byte_span(source, name_span),
        detail: None,
    });
    hints.push(InlayHint {
        name: name.to_string(),
        kind,
        ty,
        span: byte_span(source, name_span),
    });
}

fn collect_signatures(table: &crate::scope::SymbolTable) -> Vec<SignatureInfo> {
    let mut grouped: BTreeMap<String, Vec<FunctionType>> = BTreeMap::new();
    for symbol in table.iter() {
        if symbol.kind != InternalSymbolKind::Function {
            continue;
        }
        if let Type::Function(signature) = &symbol.ty {
            grouped
                .entry(symbol.name.clone())
                .or_default()
                .push(signature.clone());
        }
    }
    grouped
        .into_iter()
        .map(|(name, overloads)| SignatureInfo { name, overloads })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn collect_statement(
    spanned: &SpannedStatement,
    stmt: &Statement,
    checker: &mut TypeChecker<'_>,
    source: &str,
    symbols: &mut Vec<TypedSymbol>,
    hints: &mut Vec<InlayHint>,
    expressions: &mut Vec<TypedExpression>,
) {
    match (&spanned.kind, stmt) {
        (
            SpannedStatementKind::Let {
                name,
                name_span,
                value: spanned_value,
                ..
            },
            Statement::Let { value, .. },
        ) => {
            let ty = checker.infer_expr_silent(value).ty;
            symbols.push(TypedSymbol {
                name: name.clone(),
                kind: SymbolKind::Variable,
                ty: ty.clone(),
                span: byte_span(source, spanned.span),
                name_span: byte_span(source, *name_span),
                detail: None,
            });
            hints.push(InlayHint {
                name: name.clone(),
                kind: SymbolKind::Variable,
                ty,
                span: byte_span(source, *name_span),
            });
            collect_expr(spanned_value, value, checker, source, expressions);
        }
        (SpannedStatementKind::MoveJ { target: st }, Statement::MoveJ { target }) => {
            collect_expr(st, target, checker, source, expressions);
        }
        (SpannedStatementKind::MoveL { target: st }, Statement::MoveL { target }) => {
            collect_expr(st, target, checker, source, expressions);
        }
        (
            SpannedStatementKind::MoveC { via: sv, target: st },
            Statement::MoveC { via, target },
        ) => {
            collect_expr(sv, via, checker, source, expressions);
            collect_expr(st, target, checker, source, expressions);
        }
        (SpannedStatementKind::Wait(se), Statement::Wait(e)) => {
            collect_expr(se, e, checker, source, expressions);
        }
        (
            SpannedStatementKind::SetOutput { value: sv, .. },
            Statement::SetOutput { value, .. },
        ) => {
            collect_expr(sv, value, checker, source, expressions);
        }
        (
            SpannedStatementKind::If {
                condition: sc,
                then_branch: stb,
                else_branch: seb,
            },
            Statement::If {
                condition: c,
                then_branch: tb,
                else_branch: eb,
            },
        ) => {
            collect_expr(sc, c, checker, source, expressions);
            for (s, d) in stb.iter().zip(tb.iter()) {
                collect_statement(s, d, checker, source, symbols, hints, expressions);
            }
            if let (Some(seb), Some(eb)) = (seb.as_ref(), eb.as_ref()) {
                for (s, d) in seb.iter().zip(eb.iter()) {
                    collect_statement(s, d, checker, source, symbols, hints, expressions);
                }
            }
        }
        (SpannedStatementKind::Expr(se), Statement::Expr(e)) => {
            collect_expr(se, e, checker, source, expressions);
        }
        _ => {}
    }
}

fn collect_expr(
    spanned: &SpannedExpr,
    expr: &Expr,
    checker: &mut TypeChecker<'_>,
    source: &str,
    expressions: &mut Vec<TypedExpression>,
) {
    let ty = checker.infer_expr_silent(expr).ty;
    expressions.push(TypedExpression {
        ty,
        span: byte_span(source, spanned.span),
    });

    match (&spanned.kind, expr) {
        (SpannedExprKind::Vector3(xs), Expr::Vector3(ys)) => {
            for (sx, sy) in xs.iter().zip(ys.iter()) {
                collect_expr(sx, sy, checker, source, expressions);
            }
        }
        (
            SpannedExprKind::Pose {
                position: sp,
                orientation: so,
            },
            Expr::Pose {
                position: p,
                orientation: o,
            },
        ) => {
            collect_expr(sp, p, checker, source, expressions);
            collect_expr(so, o, checker, source, expressions);
        }
        // NOTE: the semantic side is the *normalized* program, so a named call
        // may have more (defaulted) positional args than the spanned source.
        // `zip` truncates to the source args; for `joints` every member is an
        // `Angle`, so the inferred types stay correct.
        (SpannedExprKind::Call { args: sa, .. }, Expr::Call { args, .. }) => {
            for (s, d) in sa.iter().zip(args.iter()) {
                collect_expr(&s.value, &d.value, checker, source, expressions);
            }
        }
        (SpannedExprKind::MemberCall { args: sa, .. }, Expr::MemberCall { args, .. }) => {
            for (s, d) in sa.iter().zip(args.iter()) {
                collect_expr(&s.value, &d.value, checker, source, expressions);
            }
        }
        (
            SpannedExprKind::Binary {
                left: sl,
                right: sr,
                ..
            },
            Expr::Binary {
                left: l,
                right: r,
                ..
            },
        ) => {
            collect_expr(sl, l, checker, source, expressions);
            collect_expr(sr, r, checker, source, expressions);
        }
        (
            SpannedExprKind::Unary { operand: so, .. },
            Expr::Unary { operand: o, .. },
        ) => {
            collect_expr(so, o, checker, source, expressions);
        }
        _ => {}
    }
}

fn function_type(table: &crate::scope::SymbolTable, name: &str) -> Option<FunctionType> {
    table
        .lookup(name)
        .and_then(|entries| entries.first())
        .and_then(|symbol| match &symbol.ty {
            Type::Function(signature) => Some(signature.clone()),
            _ => None,
        })
}

fn char_span(range: std::ops::Range<usize>) -> Span {
    Span::new(range.start, range.end)
}

fn byte_span(source: &str, span: Span) -> SourceSpan {
    char_range_to_byte_span(source, span.start..span.end)
}

/// Build a semantic diagnostic, attaching a best-effort location by looking up
/// the first quoted token in the message (e.g. `Unknown identifier 'ptt2'`).
fn semantic_diagnostic(source: &str, message: String) -> Diagnostic {
    let span = quoted_token(&message)
        .and_then(|token| find_token_char_span(source, token))
        .map(|span| byte_span(source, span))
        .unwrap_or_else(|| SourceSpan::new(0, 0));

    Diagnostic {
        severity: DiagnosticSeverity::Error,
        code: Some("THL_SEMANTIC_ERROR".to_string()),
        message,
        span,
    }
}

fn quoted_token(message: &str) -> Option<&str> {
    let start = message.find('\'')?;
    let rest = &message[start + 1..];
    let end = rest.find('\'')?;
    Some(&rest[..end])
}

fn find_token_char_span(source: &str, token: &str) -> Option<Span> {
    let byte_pos = source.find(token)?;
    let char_start = source[..byte_pos].chars().count();
    Some(Span::new(char_start, char_start + token.chars().count()))
}
