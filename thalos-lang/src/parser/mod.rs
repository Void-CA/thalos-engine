use std::ops::Range;

use chumsky::prelude::*;

use crate::ast::spanned::{
    SpannedArg, SpannedConstDecl, SpannedExpr, SpannedExprKind, SpannedFnDecl,
    SpannedItem, SpannedParam, SpannedProgram, SpannedStatement, SpannedStatementKind,
    SpannedTargetDecl,
};
use crate::ast::item::UseDecl;
use crate::ast::program::Program;
use crate::span::Span;
use crate::units::{AngleRadians, DurationSeconds, LengthMeters};

pub type ParseError = Simple<char>;

/// Convert a chumsky char-index span into the language [`Span`].
///
/// Spans stay in **character offsets** throughout the parser/spanned tree; the
/// language service performs the single char → byte conversion at its boundary.
fn char_span(range: Range<usize>) -> Span {
    Span::new(range.start, range.end)
}

pub fn parser() -> impl Parser<char, SpannedProgram, Error = Simple<char>> {
    let ident = text::ident();

    let ident_spanned = ident
        .clone()
        .map_with_span(|name: String, span: Range<usize>| (name, char_span(span)));

    let digits = filter(|c: &char| c.is_ascii_digit() || *c == '.')
        .repeated()
        .at_least(1);

    let number = just('-')
        .or_not()
        .then(digits)
        .map(|(neg, dig)| {
            let mut s = String::new();
            if neg.is_some() {
                s.push('-');
            }
            s.extend(dig);
            s.parse::<f64>().unwrap_or(0.0)
        });

    let duration_unit = choice((
        just("ms").map(|_| 0.001),
        just("s").map(|_| 1.0),
    ));

    let duration_expr = number
        .then(duration_unit)
        .map_with_span(|(val, mult), span: Range<usize>| SpannedExpr {
            kind: SpannedExprKind::Duration(DurationSeconds(val * mult)),
            span: char_span(span),
        });

    let length_unit = choice((
        just("mm").map(|_| 0.001),
        just("m").map(|_| 1.0),
    ));

    let length_expr = number
        .then(length_unit)
        .map_with_span(|(val, mult), span: Range<usize>| SpannedExpr {
            kind: SpannedExprKind::Length(LengthMeters(val * mult)),
            span: char_span(span),
        });

    let angle_unit = choice((
        just("deg").map(|_| std::f64::consts::PI / 180.0),
        just("rad").map(|_| 1.0),
    ));

    let angle_expr = number
        .then(angle_unit)
        .map_with_span(|(val, mult), span: Range<usize>| SpannedExpr {
            kind: SpannedExprKind::Angle(AngleRadians(val * mult)),
            span: char_span(span),
        });

    let expr = recursive(|expr| {
        let vector3_expr = expr
            .clone()
            .padded()
            .separated_by(just(','))
            .allow_trailing()
            .delimited_by(just('[').padded(), just(']').padded())
            .try_map(|elems: Vec<SpannedExpr>, span: Range<usize>| {
                if elems.len() == 3 {
                    Ok(SpannedExpr {
                        kind: SpannedExprKind::Vector3([
                            Box::new(elems[0].clone()),
                            Box::new(elems[1].clone()),
                            Box::new(elems[2].clone()),
                        ]),
                        span: char_span(span),
                    })
                } else {
                    Err(Simple::custom(span, "Vector3 expects exactly 3 elements"))
                }
            });

        // Assignment token: `=` that is not the start of `==`. The lookahead
        // keeps `f(x == 1)` parsing as a comparison while still accepting
        // `joints(j2 = 5deg)` and `joints(j2=5deg)`.
        let assign = just('=').then_ignore(none_of('=').rewind());

        // An argument is either `name = expr` (named) or a bare expression
        // (positional). The `rewind()` lookahead distinguishes the two without
        // committing: when no assignment follows the identifier, the positional
        // branch parses the full expression instead (e.g. `PICK + LIFT`).
        let arg = ident_spanned
            .clone()
            .then_ignore(assign.clone().padded())
            .rewind()
            .then_ignore(ident_spanned.clone())
            .then_ignore(assign.clone().padded())
            .then(expr.clone())
            .map(|((name, name_span), value): ((String, Span), SpannedExpr)| SpannedArg {
                name: Some(name),
                name_span: Some(name_span),
                value,
            })
            .or(expr.clone().map(|value: SpannedExpr| SpannedArg {
                name: None,
                name_span: None,
                value,
            }));

        let call_expr = ident_spanned
            .clone()
            .then(
                arg.clone()
                    .separated_by(just(',').padded())
                    .allow_trailing()
                    .delimited_by(just('('), just(')')),
            )
            .map_with_span(
                |((callee, callee_span), args): ((String, Span), Vec<SpannedArg>),
                 span: Range<usize>| SpannedExpr {
                    kind: SpannedExprKind::Call {
                        callee,
                        callee_span,
                        args,
                    },
                    span: char_span(span),
                },
            );

        let string_expr = filter(|c: &char| *c != '"')
            .repeated()
            .collect::<String>()
            .delimited_by(just('"'), just('"'))
            .map_with_span(|value: String, span: Range<usize>| SpannedExpr {
                kind: SpannedExprKind::StringLiteral(value),
                span: char_span(span),
            });

        // `receiver.member` is a member access; `receiver.member(args)` is a
        // member call. Both share one rule so the optional argument list never
        // forces backtracking.
        let member_expr = ident_spanned
            .clone()
            .then_ignore(just('.'))
            .then(ident_spanned.clone())
            .then(
                arg.clone()
                    .separated_by(just(',').padded())
                    .allow_trailing()
                    .delimited_by(just('('), just(')'))
                    .or_not(),
            )
            .map_with_span(
                |(((object, object_span), (method, method_span)), args): (
                    ((String, Span), (String, Span)),
                    Option<Vec<SpannedArg>>,
                ),
                 span: Range<usize>| SpannedExpr {
                    kind: match args {
                        Some(args) => SpannedExprKind::MemberCall {
                            object,
                            object_span,
                            method,
                            method_span,
                            args,
                        },
                        None => SpannedExprKind::MemberAccess {
                            object,
                            object_span,
                            member: method,
                            member_span: method_span,
                        },
                    },
                    span: char_span(span),
                },
            );

        let boolean_expr = choice((
            just("true").map(|_| true),
            just("false").map(|_| false),
        ))
        .map_with_span(|value: bool, span: Range<usize>| SpannedExpr {
            kind: SpannedExprKind::Boolean(value),
            span: char_span(span),
        });

        let number_expr = number
            .clone()
            .map_with_span(|value: f64, span: Range<usize>| SpannedExpr {
                kind: SpannedExprKind::Number(value),
                span: char_span(span),
            });

        let identifier_expr = ident_spanned
            .clone()
            .map_with_span(|(name, _): (String, Span), span: Range<usize>| SpannedExpr {
                kind: SpannedExprKind::Identifier(name),
                span: char_span(span),
            });

        let literal_expr = choice((
            duration_expr,
            length_expr,
            angle_expr,
            number_expr,
            string_expr,
            boolean_expr,
            member_expr,
            identifier_expr,
        ));

        let atom = choice((vector3_expr, call_expr, literal_expr)).padded();

        let op = choice((
            just(">=").map(|_| crate::ast::BinaryOp::Gte),
            just("<=").map(|_| crate::ast::BinaryOp::Lte),
            just("==").map(|_| crate::ast::BinaryOp::Eq),
            just("!=").map(|_| crate::ast::BinaryOp::Neq),
            just('>').map(|_| crate::ast::BinaryOp::Gt),
            just('<').map(|_| crate::ast::BinaryOp::Lt),
            just('+').map(|_| crate::ast::BinaryOp::Add),
            just('-').map(|_| crate::ast::BinaryOp::Sub),
            just('*').map(|_| crate::ast::BinaryOp::Mul),
            just('/').map(|_| crate::ast::BinaryOp::Div),
        ))
        .padded();

        // Prefix negation: `-expr`. Numeric literals keep their sign inside the
        // literal itself (`-10mm` parses as a negative `Length`), so the atom is
        // tried first; only when it cannot start (e.g. `-side`, `-p.x`) is `-`
        // read as the unary operator. Repetition makes `--side` a double
        // negation for free.
        let unary = recursive(|unary| {
            atom.clone().or(just('-')
                .padded()
                .ignore_then(unary)
                .map_with_span(|operand: SpannedExpr, span: Range<usize>| SpannedExpr {
                    kind: SpannedExprKind::Unary {
                        op: crate::ast::UnaryOp::Neg,
                        operand: Box::new(operand),
                    },
                    span: char_span(span),
                }))
        });

        unary
            .clone()
            .then(op.then(unary).repeated())
            .map(|(first, rest): (SpannedExpr, Vec<(crate::ast::BinaryOp, SpannedExpr)>)| {
                rest.into_iter().fold(first, |acc, (op, val)| {
                    let span = Span::new(acc.span.start, val.span.end);
                    SpannedExpr {
                        kind: SpannedExprKind::Binary {
                            left: Box::new(acc),
                            op,
                            right: Box::new(val),
                        },
                        span,
                    }
                })
            })
    });

    // Explicit type annotation `: Type`, preserving both the type name and the
    // span of the type token so tooling can classify it without re-parsing.
    let type_ann = just(':').padded().ignore_then(
        ident
            .clone()
            .map_with_span(|name: String, span: Range<usize>| (name, char_span(span)))
            .padded(),
    );

    let stmt_parser = recursive(|stmt| {
        let let_stmt = just("let")
            .ignore_then(ident_spanned.clone().padded())
            .then(type_ann.or_not())
            .then_ignore(just('=').padded())
            .then(expr.clone())
            .then_ignore(just(';').or_not())
            .map_with_span(
                |(((name, name_span), type_ann), value): (
                    ((String, Span), Option<(String, Span)>),
                    SpannedExpr,
                ),
                 span: Range<usize>| {
                    let (type_ann, type_span) = match type_ann {
                        Some((ann, ann_span)) => (Some(ann), Some(ann_span)),
                        None => (None, None),
                    };
                    SpannedStatement {
                        kind: SpannedStatementKind::Let {
                            name,
                            name_span,
                            type_ann,
                            type_span,
                            value,
                        },
                        span: char_span(span),
                    }
                },
            );

        let movej_stmt = just("movej")
            .ignore_then(expr.clone().delimited_by(just('('), just(')')))
            .then_ignore(just(';').or_not())
            .map_with_span(|target: SpannedExpr, span: Range<usize>| SpannedStatement {
                kind: SpannedStatementKind::MoveJ { target },
                span: char_span(span),
            });

        let movel_stmt = just("movel")
            .ignore_then(expr.clone().delimited_by(just('('), just(')')))
            .then_ignore(just(';').or_not())
            .map_with_span(|target: SpannedExpr, span: Range<usize>| SpannedStatement {
                kind: SpannedStatementKind::MoveL { target },
                span: char_span(span),
            });

        // movec(VIA, TARGET): circular move through an intermediate (via) point
        // to the final target. Both are expressions resolving to positions/poses.
        let movec_stmt = just("movec")
            .ignore_then(
                expr.clone()
                    .then_ignore(just(',').padded())
                    .then(expr.clone())
                    .delimited_by(just('(').padded(), just(')').padded()),
            )
            .then_ignore(just(';').or_not())
            .map_with_span(
                |(via, target): (SpannedExpr, SpannedExpr),
                 span: Range<usize>| SpannedStatement {
                    kind: SpannedStatementKind::MoveC { via, target },
                    span: char_span(span),
                },
            );

        let wait_stmt = just("wait")
            .ignore_then(expr.clone().delimited_by(just('('), just(')')))
            .then_ignore(just(';').or_not())
            .map_with_span(|duration: SpannedExpr, span: Range<usize>| SpannedStatement {
                kind: SpannedStatementKind::Wait(duration),
                span: char_span(span),
            });

        // set_output(CHANNEL, value): operational (non-geometric) instruction.
        let set_output_stmt = just("set_output")
            .ignore_then(
                ident_spanned
                    .clone()
                    .padded()
                    .then_ignore(just(',').padded())
                    .then(expr.clone())
                    .delimited_by(just('(').padded(), just(')').padded()),
            )
            .then_ignore(just(';').or_not())
            .map_with_span(
                |((output, output_span), value): ((String, Span), SpannedExpr),
                 span: Range<usize>| SpannedStatement {
                    kind: SpannedStatementKind::SetOutput {
                        output,
                        output_span,
                        value,
                    },
                    span: char_span(span),
                },
            );

        let block = stmt
            .clone()
            .repeated()
            .delimited_by(just('{').padded(), just('}').padded());

        let else_branch = just("else")
            .padded()
            .ignore_then(block.clone());

        let if_stmt = just("if")
            .padded()
            .ignore_then(expr.clone())
            .then(block)
            .then(else_branch.or_not())
            .map_with_span(
                |((condition, then_branch), else_branch): (
                    (SpannedExpr, Vec<SpannedStatement>),
                    Option<Vec<SpannedStatement>>,
                ),
                 span: Range<usize>| SpannedStatement {
                    kind: SpannedStatementKind::If {
                        condition,
                        then_branch,
                        else_branch,
                    },
                    span: char_span(span),
                },
            );

        let expr_stmt = expr
            .clone()
            .then_ignore(just(';'))
            .map_with_span(|expr: SpannedExpr, span: Range<usize>| SpannedStatement {
                kind: SpannedStatementKind::Expr(expr),
                span: char_span(span),
            });

        choice((
            if_stmt,
            let_stmt,
            movej_stmt,
            movel_stmt,
            movec_stmt,
            wait_stmt,
            set_output_stmt,
            expr_stmt,
        ))
        .padded()
    });

    let fn_body = stmt_parser
        .clone()
        .repeated()
        .then(expr.clone().or_not())
        .delimited_by(just('{').padded(), just('}').padded());

    let param = ident_spanned
        .clone()
        .padded()
        .then(type_ann.or_not())
        .map_with_span(
            |((name, name_span), type_ann): ((String, Span), Option<(String, Span)>),
             span: Range<usize>| {
                let (type_ann, type_span) = match type_ann {
                    Some((ann, ann_span)) => (Some(ann), Some(ann_span)),
                    None => (None, None),
                };
                SpannedParam {
                    name,
                    name_span,
                    type_ann,
                    type_span,
                    span: char_span(span),
                }
            },
        );

    let fn_decl = just("fn")
        .ignore_then(ident_spanned.clone().padded())
        .then(
            param
                .separated_by(just(',').padded())
                .allow_trailing()
                .delimited_by(just('('), just(')')),
        )
        .then(
            just("->")
                .padded()
                .ignore_then(
                    ident
                        .clone()
                        .map_with_span(|name: String, span: Range<usize>| {
                            (name, char_span(span))
                        })
                        .padded(),
                )
                .or_not(),
        )
        .then(fn_body)
        .map_with_span(
            |((((name, name_span), params), return_type), (body, tail_expr)): (
                (((String, Span), Vec<SpannedParam>), Option<(String, Span)>),
                (Vec<SpannedStatement>, Option<SpannedExpr>),
            ),
             span: Range<usize>| {
                let (return_type, return_type_span) = match return_type {
                    Some((ty, ty_span)) => (Some(ty), Some(ty_span)),
                    None => (None, None),
                };
                SpannedItem::Function(SpannedFnDecl {
                    name,
                    name_span,
                    params,
                    return_type,
                    return_type_span,
                    body,
                    tail_expr,
                    span: char_span(span),
                })
            },
        );

    let const_decl = just("const")
        .ignore_then(ident_spanned.clone().padded())
        .then(type_ann.or_not())
        .then_ignore(just('=').padded())
        .then(expr.clone().padded())
        .then_ignore(just(';').or_not())
        .map_with_span(
            |(((name, name_span), type_ann), value): (
                ((String, Span), Option<(String, Span)>),
                SpannedExpr,
            ),
             span: Range<usize>| {
                let (type_ann, type_span) = match type_ann {
                    Some((ann, ann_span)) => (Some(ann), Some(ann_span)),
                    None => (None, None),
                };
                SpannedItem::Const(SpannedConstDecl {
                    name,
                    name_span,
                    type_ann,
                    type_span,
                    value,
                    span: char_span(span),
                })
            },
        );

    let target_decl = just("target")
        .ignore_then(ident_spanned.clone().padded())
        .then_ignore(just('='))
        .then(expr.clone().padded())
        .then_ignore(just(';').or_not())
        .map_with_span(
            |((name, name_span), pose): ((String, Span), SpannedExpr), span: Range<usize>| {
                SpannedItem::Target(SpannedTargetDecl {
                    name,
                    name_span,
                    pose,
                    span: char_span(span),
                })
            },
        );

    let use_decl = just("use")
        .ignore_then(ident.padded())
        .then_ignore(just(';').or_not())
        .map_with_span(|path: String, span: Range<usize>| SpannedItem::Use {
            decl: UseDecl { path },
            span: char_span(span),
        });

    let item = choice((const_decl, fn_decl, target_decl, use_decl)).padded();

    item.repeated()
        .then_ignore(end())
        .map(|items| SpannedProgram { items })
}

/// Parse source into the spanned (tooling) tree.
pub fn parse_source_spanned(source: &str) -> Result<SpannedProgram, Vec<Simple<char>>> {
    parser().parse(source)
}

/// Parse source into the semantic AST (spans stripped).
pub fn parse_source(source: &str) -> Result<Program, Vec<Simple<char>>> {
    parse_source_spanned(source).map(SpannedProgram::unspan)
}
