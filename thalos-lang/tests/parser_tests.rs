use thalos_lang::parse_source;
use thalos_lang::ast::*;
use thalos_lang::units::{DurationSeconds, LengthMeters};

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
