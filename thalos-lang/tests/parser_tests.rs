use thalos_lang::parse_source;
use thalos_lang::ast::*;
use thalos_lang::parser::parse_source_spanned;
use thalos_lang::units::{DurationSeconds, LengthMeters};

fn char_slice(source: &str, span: thalos_lang::Span) -> String {
    source.chars().skip(span.start).take(span.end - span.start).collect()
}

#[test]
fn test_parse_simple_program() {
    let source = r#"
        use material_handling

        target pick_pos = 420mm

        fn main() {
            movej(pick_pos)
            wait(500ms)
        }
    "#;

    let program = parse_source(source).expect("failed to parse program");
    assert_eq!(program.items.len(), 3);

    match &program.items[0] {
        Item::Use(u) => assert_eq!(u.path, "material_handling"),
        _ => panic!("expected UseDecl"),
    }

    match &program.items[1] {
        Item::Target(t) => {
            assert_eq!(t.name, "pick_pos");
            assert_eq!(t.pose, Expr::Length(LengthMeters(0.42)));
        }
        _ => panic!("expected TargetDecl"),
    }

    match &program.items[2] {
        Item::Function(f) => {
            assert_eq!(f.name, "main");
            assert_eq!(f.body.len(), 2);
            assert_eq!(
                f.body[0],
                Statement::MoveJ {
                    target: Expr::Identifier("pick_pos".to_string())
                }
            );
            assert_eq!(
                f.body[1],
                Statement::Wait(Expr::Duration(DurationSeconds(0.5)))
            );
        }
        _ => panic!("expected FnDecl"),
    }
}

#[test]
fn test_unspan_roundtrip_equivalence() {
    // Guard: stripping spans must reproduce the semantic AST exactly.
    let sources = [
        r#"const CLEARANCE = [0mm, 0mm, 150mm]

target PARK = joints(0deg, -30deg, -25deg, 0deg)
target PICK = position([1320mm, 140mm, 80mm])

fn main() {
    movej(PARK)
    movel(PICK)
    wait(500ms)
    movej(PARK)
}
"#,
        r#"
target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)
target ptt = position([2.152, 0.783, 1.882])

fn main() {
    movej(jtt)
    movel(ptt)

    let offset1 = [1, 0, 0]
    movel(ptt - offset1)
}
"#,
        r#"
use material_handling

const CLEARANCE: Length = 150mm

target ABOVE = pose(PART_CENTER + [0mm, 0mm, 300mm], euler(0deg, 180deg, 0deg))

fn main() {
    movec(A, B)
    wait(1s)
    set_output(GRIPPER, true)
    if sensors.value > 1mm {
        movej(B)
    } else {
        wait(50ms)
    }
}
"#,
    ];

    for source in sources {
        let semantic = parse_source(source).expect("semantic parse");
        let spanned = parse_source_spanned(source).expect("spanned parse");
        assert_eq!(
            spanned.unspan(),
            semantic,
            "unspan(spanned) must equal the semantic AST"
        );
    }
}

#[test]
fn test_spanned_spans_disambiguate_repeated_identifiers() {
    let source = r#"target PARK = position([1mm, 0mm, 0mm])
target PICK = position([2mm, 0mm, 0mm])

fn main() {
    movej(PARK)
    movel(PICK)
    movej(PARK)
}
"#;

    let program = parse_source_spanned(source).expect("must parse");

    // Declaration name spans point at the declarations, not the first match.
    let park_decl = match &program.items[0] {
        SpannedItem::Target(t) => t,
        other => panic!("expected target, got {other:?}"),
    };
    assert_eq!(char_slice(source, park_decl.name_span), "PARK");

    let function = match &program.items[2] {
        SpannedItem::Function(f) => f,
        other => panic!("expected function, got {other:?}"),
    };
    assert_eq!(function.body.len(), 3);

    let first_use = match &function.body[0].kind {
        SpannedStatementKind::MoveJ { target } => target,
        other => panic!("expected movej, got {other:?}"),
    };
    let repeated_use = match &function.body[2].kind {
        SpannedStatementKind::MoveJ { target } => target,
        other => panic!("expected movej, got {other:?}"),
    };

    assert_eq!(char_slice(source, first_use.span), "PARK");
    assert_eq!(char_slice(source, repeated_use.span), "PARK");
    assert_ne!(
        first_use.span, repeated_use.span,
        "repeated identifiers must resolve to distinct source locations"
    );
    assert!(repeated_use.span.start > first_use.span.start);

    // Call callee spans are captured separately from the whole call.
    let pick_call = match &function.body[1].kind {
        SpannedStatementKind::MoveL { target } => target,
        other => panic!("expected movel, got {other:?}"),
    };
    match &park_decl.pose.kind {
        SpannedExprKind::Call { callee, callee_span, .. } => {
            assert_eq!(callee, "position");
            assert_eq!(char_slice(source, *callee_span), "position");
        }
        other => panic!("expected call, got {other:?}"),
    }
    assert_eq!(char_slice(source, pick_call.span), "PICK");
}

#[test]
fn test_parse_named_arguments_capture_name_and_span() {
    let source = r#"target JTT = joints(j3 = 30deg, j1 = 10deg)"#;
    let program = parse_source_spanned(source).expect("must parse");

    let target = match &program.items[0] {
        SpannedItem::Target(t) => t,
        other => panic!("expected target, got {other:?}"),
    };

    match &target.pose.kind {
        SpannedExprKind::Call { callee, args, .. } => {
            assert_eq!(callee, "joints");
            assert_eq!(args.len(), 2);

            assert_eq!(args[0].name.as_deref(), Some("j3"));
            assert_eq!(char_slice(source, args[0].name_span.expect("name span")), "j3");
            assert_eq!(args[1].name.as_deref(), Some("j1"));
            assert_eq!(char_slice(source, args[1].name_span.expect("name span")), "j1");
        }
        other => panic!("expected call, got {other:?}"),
    }
}

#[test]
fn test_positional_arguments_are_unnamed() {
    let source = r#"target JTT = joints(10deg, 20deg)"#;
    let program = parse_source_spanned(source).expect("must parse");

    let target = match &program.items[0] {
        SpannedItem::Target(t) => t,
        other => panic!("expected target, got {other:?}"),
    };

    match &target.pose.kind {
        SpannedExprKind::Call { args, .. } => {
            assert!(args.iter().all(|a| a.name.is_none()));
            assert!(args.iter().all(|a| a.name_span.is_none()));
        }
        other => panic!("expected call, got {other:?}"),
    }
}

#[test]
fn test_named_argument_lookahead_does_not_break_comparisons() {
    // `a == b` must parse as a comparison argument, not as `a = (b)`.
    let source = r#"fn main() {
    movel(choose(a == b))
}"#;
    let program = parse_source_spanned(source).expect("must parse");

    let function = match &program.items[0] {
        SpannedItem::Function(f) => f,
        other => panic!("expected function, got {other:?}"),
    };
    let target = match &function.body[0].kind {
        SpannedStatementKind::MoveL { target } => target,
        other => panic!("expected movel, got {other:?}"),
    };

    match &target.kind {
        SpannedExprKind::Call { callee, args, .. } => {
            assert_eq!(callee, "choose");
            assert_eq!(args.len(), 1);
            assert!(args[0].name.is_none());
            assert!(matches!(
                args[0].value.kind,
                SpannedExprKind::Binary { op: BinaryOp::Eq, .. }
            ));
        }
        other => panic!("expected call, got {other:?}"),
    }
}

#[test]
fn test_parse_member_call_captures_object_method_and_args() {
    let source = r#"target P = position([1mm, 2mm, 3mm])
fn main() {
    movel(P.offset(x = -10mm))
}"#;
    let program = parse_source_spanned(source).expect("must parse");

    let function = match &program.items[1] {
        SpannedItem::Function(f) => f,
        other => panic!("expected function, got {other:?}"),
    };
    let target = match &function.body[0].kind {
        SpannedStatementKind::MoveL { target } => target,
        other => panic!("expected movel, got {other:?}"),
    };

    match &target.kind {
        SpannedExprKind::MemberCall {
            object,
            method,
            args,
            ..
        } => {
            assert_eq!(object, "P");
            assert_eq!(method, "offset");
            assert_eq!(args.len(), 1);
            assert_eq!(args[0].name.as_deref(), Some("x"));
            assert_eq!(char_slice(source, args[0].name_span.expect("name span")), "x");
        }
        other => panic!("expected member call, got {other:?}"),
    }
}

#[test]
fn test_parse_member_access_without_args_stays_a_member_access() {
    let source = r#"target P = position([1mm, 2mm, 3mm])
fn main() {
    movel(P.x)
}"#;
    let program = parse_source_spanned(source).expect("must parse");

    let function = match &program.items[1] {
        SpannedItem::Function(f) => f,
        other => panic!("expected function, got {other:?}"),
    };
    let target = match &function.body[0].kind {
        SpannedStatementKind::MoveL { target } => target,
        other => panic!("expected movel, got {other:?}"),
    };

    match &target.kind {
        SpannedExprKind::MemberAccess { object, member, .. } => {
            assert_eq!(object, "P");
            assert_eq!(member, "x");
        }
        other => panic!("expected member access, got {other:?}"),
    }
}

#[test]
fn test_parse_captures_type_and_member_spans() {
    let source = r#"fn f(p : Position) {
    let q : Length = p.offset(x = 1mm)
}"#;
    let program = parse_source_spanned(source).expect("must parse");

    let SpannedItem::Function(f) = &program.items[0] else {
        panic!("expected function");
    };
    let param = &f.params[0];
    assert_eq!(char_slice(source, param.name_span), "p");
    assert_eq!(char_slice(source, param.type_span.expect("param type span")), "Position");

    let SpannedStatementKind::Let {
        name_span,
        type_span,
        value,
        ..
    } = &f.body[0].kind
    else {
        panic!("expected let");
    };
    assert_eq!(char_slice(source, *name_span), "q");
    assert_eq!(char_slice(source, type_span.expect("let type span")), "Length");

    let SpannedExprKind::MemberCall {
        object_span,
        method_span,
        ..
    } = &value.kind
    else {
        panic!("expected member call, got {:?}", value.kind);
    };
    assert_eq!(char_slice(source, *object_span), "p");
    assert_eq!(char_slice(source, *method_span), "offset");
}

#[test]
fn test_parse_movec_circular_move() {
    let source = r#"
        fn main() {
            movec(VIA_POINT, END_POINT)
        }
    "#;

    let program = parse_source(source).expect("failed to parse movec program");

    match &program.items[0] {
        Item::Function(f) => {
            assert_eq!(f.body.len(), 1);
            assert_eq!(
                f.body[0],
                Statement::MoveC {
                    via: Expr::Identifier("VIA_POINT".to_string()),
                    target: Expr::Identifier("END_POINT".to_string()),
                }
            );
        }
        _ => panic!("expected FnDecl"),
    }
}

#[test]
fn test_parse_wait_and_set_output() {
    let source = r#"
        fn main() {
            wait(150ms)
            set_output(GRIPPER, true)
        }
    "#;

    let program = parse_source(source).expect("failed to parse wait/set_output program");

    match &program.items[0] {
        Item::Function(f) => {
            assert_eq!(f.body.len(), 2);
            assert!(matches!(f.body[0], Statement::Wait(_)));
            assert_eq!(
                f.body[1],
                Statement::SetOutput {
                    output: "GRIPPER".to_string(),
                    value: Expr::Boolean(true),
                }
            );
        }
        _ => panic!("expected FnDecl"),
    }
}

#[test]
fn test_parse_unary_negation_preserves_signed_literals() {
    let program = parse_source(
        r#"
        const NEG_LITERAL = -10mm
        const NEG_VALUE = -side
        const DOUBLE = --side
        fn main() {}
    "#,
    )
    .expect("must parse");

    let consts: Vec<&Expr> = program
        .items
        .iter()
        .filter_map(|item| match item {
            Item::Const(c) => Some(&c.value),
            _ => None,
        })
        .collect();
    assert_eq!(consts.len(), 3);

    // A leading `-` on a numeric literal stays inside the literal.
    assert!(
        matches!(consts[0], Expr::Length(LengthMeters(v)) if (*v - -0.010).abs() < 1e-12),
        "expected a negative Length literal, got {:?}",
        consts[0]
    );

    // `-side` becomes an explicit unary node.
    match consts[1] {
        Expr::Unary {
            op: UnaryOp::Neg,
            operand,
        } => assert!(matches!(**operand, Expr::Identifier(ref id) if id == "side")),
        other => panic!("expected unary negation, got {other:?}"),
    }

    // `--side` is a natural double negation.
    match consts[2] {
        Expr::Unary { operand, .. } => match &**operand {
            Expr::Unary { operand: inner, .. } => {
                assert!(matches!(**inner, Expr::Identifier(ref id) if id == "side"));
            }
            other => panic!("expected nested unary, got {other:?}"),
        },
        other => panic!("expected unary negation, got {other:?}"),
    }
}

#[test]
fn test_parse_unary_binds_tighter_than_binary_subtraction() {
    let program = parse_source("fn main() { movel(BASE - -LIFT) }").expect("must parse");

    let Item::Function(f) = &program.items[0] else {
        panic!("expected FnDecl");
    };
    let Statement::MoveL { target } = &f.body[0] else {
        panic!("expected movel");
    };

    match target {
        Expr::Binary {
            left,
            op: BinaryOp::Sub,
            right,
        } => {
            assert!(matches!(**left, Expr::Identifier(ref id) if id == "BASE"));
            assert!(matches!(
                **right,
                Expr::Unary {
                    op: UnaryOp::Neg,
                    ..
                }
            ));
        }
        other => panic!("expected subtraction with unary right operand, got {other:?}"),
    }
}
