use crate::scope::SymbolTable;
use crate::symbols::{Symbol, SymbolKind};
use crate::types::{FunctionType, Type};

pub fn register_builtins(table: &mut SymbolTable) {
    let overloaded_builtins = vec![
        // movej
        ("movej", vec![Type::Joints { dimension: None }], Type::Unit),
        ("movej", vec![Type::Position], Type::Unit),
        ("movej", vec![Type::Pose], Type::Unit),

        // movel
        ("movel", vec![Type::Position], Type::Unit),
        ("movel", vec![Type::Pose], Type::Unit),

        // movec
        ("movec", vec![Type::Position, Type::Position], Type::Unit),
        ("movec", vec![Type::Pose, Type::Pose], Type::Unit),

        // wait
        ("wait", vec![Type::Duration], Type::Unit),

        // set_output
        ("set_output", vec![Type::String, Type::Bool], Type::Unit),

        // Built-in constructors
        ("position", vec![Type::Vector3], Type::Position),
        ("pose", vec![Type::Vector3, Type::Quaternion], Type::Pose),
        (
            "euler",
            vec![Type::Angle, Type::Angle, Type::Angle],
            Type::Quaternion,
        ),
        (
            "quaternion",
            vec![Type::Float, Type::Float, Type::Float, Type::Float],
            Type::Quaternion,
        ),
    ];

    for (name, params, return_type) in overloaded_builtins {
        let fn_type = Type::Function(FunctionType {
            params,
            return_type: Box::new(return_type),
        });
        let _ = table.declare_builtin(Symbol::new(name, SymbolKind::Function, fn_type, None));
    }
}
