use chumsky::prelude::*;
use crate::ast::*;
use crate::units::{AngleRadians, DurationSeconds, LengthMeters};

pub type ParseError = Simple<char>;

pub fn parser() -> impl Parser<char, Program, Error = Simple<char>> {
    let ident = text::ident();

    let number = filter(|c: &char| c.is_ascii_digit() || *c == '.')
        .repeated()
        .at_least(1)
        .collect::<String>()
        .map(|s| s.parse::<f64>().unwrap_or(0.0));

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
        let literal_expr = choice((
            duration_expr,
            length_expr,
            angle_expr,
            number.map(Expr::Number),
            just("true").map(|_| Expr::Boolean(true)),
            just("false").map(|_| Expr::Boolean(false)),
            ident.map(Expr::Identifier),
        ));

        let call_expr = ident
            .then(
                expr.clone()
                    .separated_by(just(',').padded())
                    .allow_trailing()
                    .delimited_by(just('('), just(')')),
            )
            .map(|(callee, args)| Expr::Call { callee, args });

        choice((call_expr, literal_expr)).padded()
    });

    let movej_stmt = just("movej")
        .ignore_then(expr.clone().delimited_by(just('('), just(')')))
        .map(|target| Statement::MoveJ { target });

    let movel_stmt = just("movel")
        .ignore_then(expr.clone().delimited_by(just('('), just(')')))
        .map(|target| Statement::MoveL { target });

    let wait_stmt = just("wait")
        .ignore_then(expr.clone().delimited_by(just('('), just(')')))
        .map(Statement::Wait);

    let statement = choice((movej_stmt, movel_stmt, wait_stmt, expr.clone().map(Statement::Expr)))
        .padded()
        .then_ignore(just(';').or_not());

    let fn_decl = just("fn")
        .ignore_then(ident.padded())
        .then(
            ident
                .map(|name| Param { name, type_ann: None })
                .separated_by(just(',').padded())
                .delimited_by(just('('), just(')')),
        )
        .then(statement.repeated().delimited_by(just('{').padded(), just('}').padded()))
        .map(|((name, params), body)| Item::Function(FnDecl { name, params, body }));

    let target_decl = just("target")
        .ignore_then(ident.padded())
        .then_ignore(just('='))
        .then(expr.padded())
        .map(|(name, pose)| Item::Target(TargetDecl { name, pose }));

    let use_decl = just("use")
        .ignore_then(ident.padded())
        .map(|path| Item::Use(UseDecl { path }));

    let item = choice((fn_decl, target_decl, use_decl)).padded();

    item.repeated().then_ignore(end()).map(|items| Program { items })
}

pub fn parse_source(source: &str) -> Result<Program, Vec<Simple<char>>> {
    parser().parse(source)
}
