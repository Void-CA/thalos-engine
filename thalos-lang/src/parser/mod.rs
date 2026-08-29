use chumsky::prelude::*;
use crate::ast::*;
use crate::units::{AngleRadians, DurationSeconds, LengthMeters};

pub type ParseError = Simple<char>;

pub fn parser() -> impl Parser<char, Program, Error = Simple<char>> {
    let ident = text::ident();

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
        .map(|(val, mult)| Expr::Duration(DurationSeconds(val * mult)));

    let length_unit = choice((
        just("mm").map(|_| 0.001),
        just("m").map(|_| 1.0),
    ));

    let length_expr = number
        .then(length_unit)
        .map(|(val, mult)| Expr::Length(LengthMeters(val * mult)));

    let angle_unit = choice((
        just("deg").map(|_| std::f64::consts::PI / 180.0),
        just("rad").map(|_| 1.0),
    ));

    let angle_expr = number
        .then(angle_unit)
        .map(|(val, mult)| Expr::Angle(AngleRadians(val * mult)));

    let expr = recursive(|expr| {
        let vector3_expr = expr
            .clone()
            .padded()
            .separated_by(just(','))
            .allow_trailing()
            .delimited_by(just('[').padded(), just(']').padded())
            .try_map(|elems: Vec<Expr>, span| {
                if elems.len() == 3 {
                    Ok(Expr::Vector3([
                        Box::new(elems[0].clone()),
                        Box::new(elems[1].clone()),
                        Box::new(elems[2].clone()),
                    ]))
                } else {
                    Err(Simple::custom(span, "Vector3 expects exactly 3 elements"))
                }
            });

        let call_expr = ident
            .then(
                expr.clone()
                    .separated_by(just(',').padded())
                    .allow_trailing()
                    .delimited_by(just('('), just(')')),
            )
            .map(|(callee, args)| Expr::Call { callee, args });

        let string_expr = filter(|c: &char| *c != '"')
            .repeated()
            .collect::<String>()
            .delimited_by(just('"'), just('"'))
            .map(Expr::StringLiteral);

        let literal_expr = choice((
            duration_expr,
            length_expr,
            angle_expr,
            number.map(Expr::Number),
            string_expr,
            just("true").map(|_| Expr::Boolean(true)),
            just("false").map(|_| Expr::Boolean(false)),
            ident.map(Expr::Identifier),
        ));

        let atom = choice((vector3_expr, call_expr, literal_expr)).padded();

        let op = choice((
            just('+').map(|_| BinaryOp::Add),
            just('-').map(|_| BinaryOp::Sub),
            just('*').map(|_| BinaryOp::Mul),
            just('/').map(|_| BinaryOp::Div),
        ))
        .padded();

        atom.clone()
            .then(op.then(atom.clone()).repeated())
            .map(|(first, rest)| {
                rest.into_iter().fold(first, |acc, (op, val)| Expr::Binary {
                    left: Box::new(acc),
                    op,
                    right: Box::new(val),
                })
            })
    });

    let type_ann = just(':').padded().ignore_then(ident.padded());

    let let_stmt = just("let")
        .ignore_then(ident.padded())
        .then(type_ann.clone().or_not())
        .then_ignore(just('=').padded())
        .then(expr.clone())
        .then_ignore(just(';').or_not())
        .map(|((name, type_ann), value)| Statement::Let {
            name,
            type_ann,
            value,
        });

    let movej_stmt = just("movej")
        .ignore_then(expr.clone().delimited_by(just('('), just(')')))
        .then_ignore(just(';').or_not())
        .map(|target| Statement::MoveJ { target });

    let movel_stmt = just("movel")
        .ignore_then(expr.clone().delimited_by(just('('), just(')')))
        .then_ignore(just(';').or_not())
        .map(|target| Statement::MoveL { target });

    let wait_stmt = just("wait")
        .ignore_then(expr.clone().delimited_by(just('('), just(')')))
        .then_ignore(just(';').or_not())
        .map(Statement::Wait);

    let stmt_with_semi = choice((
        let_stmt,
        movej_stmt,
        movel_stmt,
        wait_stmt,
        expr.clone()
            .then_ignore(just(';'))
            .map(Statement::Expr),
    ))
    .padded();

    let fn_body = stmt_with_semi
        .repeated()
        .then(expr.clone().or_not())
        .delimited_by(just('{').padded(), just('}').padded());

    let fn_decl = just("fn")
        .ignore_then(ident.padded())
        .then(
            ident
                .padded()
                .then(type_ann.clone().or_not())
                .map(|(name, type_ann)| Param { name, type_ann })
                .separated_by(just(',').padded())
                .allow_trailing()
                .delimited_by(just('('), just(')')),
        )
        .then(just("->").padded().ignore_then(ident.padded()).or_not())
        .then(fn_body)
        .map(|(((name, params), return_type), (body, tail_expr))| {
            Item::Function(FnDecl {
                name,
                params,
                return_type,
                body,
                tail_expr: tail_expr.map(Box::new),
            })
        });

    let const_decl = just("const")
        .ignore_then(ident.padded())
        .then(type_ann.clone().or_not())
        .then_ignore(just('=').padded())
        .then(expr.clone().padded())
        .then_ignore(just(';').or_not())
        .map(|((name, type_ann), value)| {
            Item::Const(ConstDecl {
                name,
                type_ann,
                value,
            })
        });

    let target_decl = just("target")
        .ignore_then(ident.padded())
        .then_ignore(just('='))
        .then(expr.clone().padded())
        .then_ignore(just(';').or_not())
        .map(|(name, pose)| Item::Target(TargetDecl { name, pose }));

    let use_decl = just("use")
        .ignore_then(ident.padded())
        .then_ignore(just(';').or_not())
        .map(|path| Item::Use(UseDecl { path }));

    let item = choice((const_decl, fn_decl, target_decl, use_decl)).padded();

    item.repeated().then_ignore(end()).map(|items| Program { items })
}

pub fn parse_source(source: &str) -> Result<Program, Vec<Simple<char>>> {
    parser().parse(source)
}
