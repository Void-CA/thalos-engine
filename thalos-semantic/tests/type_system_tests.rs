use thalos_semantic::builtins::register_builtins;
use thalos_semantic::scope::{ScopeKind, SymbolTable};
use thalos_semantic::symbols::{Symbol, SymbolKind};
use thalos_semantic::types::Type;

#[test]
fn test_symbol_table_scopes_and_builtins() {
    let mut table = SymbolTable::new();
    register_builtins(&mut table);

    // Verify movej overload resolution exists
    let movej_symbols = table.lookup("movej").expect("movej built-in should exist");
    assert_eq!(movej_symbols.len(), 3);

    // Verify movel overload has 2 signatures (Position, Pose) and NOT Joints
    let movel_symbols = table.lookup("movel").expect("movel built-in should exist");
    assert_eq!(movel_symbols.len(), 2);
    for sym in movel_symbols {
        if let Type::Function(ref ft) = sym.ty {
            assert!(ft.params[0] == Type::Position || ft.params[0] == Type::Pose);
            assert_ne!(ft.params[0], Type::Joints { dimension: None });
        }
    }

    // Verify user declaration in global scope
    let target_sym = Symbol::new("pick_pos", SymbolKind::Target, Type::Position, None);
    table.declare(target_sym.clone()).expect("declaration should succeed");

    assert_eq!(table.lookup("pick_pos").unwrap()[0], target_sym);

    // Verify shadowing in local scope
    table.push_scope(ScopeKind::Function);
    let local_target = Symbol::new("pick_pos", SymbolKind::Variable, Type::Vector3, None);
    table.declare(local_target.clone()).expect("local declaration should succeed");

    assert_eq!(table.lookup("pick_pos").unwrap()[0], local_target);

    // Pop scope and verify restoration
    table.pop_scope();
    assert_eq!(table.lookup("pick_pos").unwrap()[0], target_sym);
}
