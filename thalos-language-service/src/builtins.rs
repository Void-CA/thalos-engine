use crate::scope::SymbolTable;
use crate::symbols::{Symbol, SymbolKind};
use crate::types::{FunctionType, Type};

pub fn register_builtins(table: &mut SymbolTable) {
    // (name, parameter types, parameter names, return type). Parameter names are
    // metadata for named-argument binding and tooling; they do not affect
    // overload identity. Commands that are parsed as statements carry no names.
    let overloaded_builtins: Vec<(&str, Vec<Type>, Vec<&str>, Type)> = vec![
        // movej
        ("movej", vec![Type::Joints { dimension: None }], vec![], Type::Unit),
        ("movej", vec![Type::Position], vec![], Type::Unit),
        ("movej", vec![Type::Pose], vec![], Type::Unit),

        // movel
        ("movel", vec![Type::Position], vec![], Type::Unit),
        ("movel", vec![Type::Pose], vec![], Type::Unit),

        // movec
        ("movec", vec![Type::Position, Type::Position], vec![], Type::Unit),
        ("movec", vec![Type::Pose, Type::Pose], vec![], Type::Unit),

        // wait
        ("wait", vec![Type::Duration], vec![], Type::Unit),

        // set_output
        ("set_output", vec![Type::String, Type::Bool], vec![], Type::Unit),

        // Built-in constructors
        ("position", vec![Type::Vector3], vec!["x", "y", "z"], Type::Position),
        (
            "pose",
            vec![Type::Vector3, Type::Quaternion],
            vec!["position", "orientation"],
            Type::Pose,
        ),
        (
            "pose",
            vec![Type::Position, Type::Quaternion],
            vec!["position", "orientation"],
            Type::Pose,
        ),
        (
            "euler",
            vec![Type::Angle, Type::Angle, Type::Angle],
            vec!["rx", "ry", "rz"],
            Type::Quaternion,
        ),
        (
            "quaternion",
            vec![Type::Float, Type::Float, Type::Float, Type::Float],
            vec!["w", "x", "y", "z"],
            Type::Quaternion,
        ),
    ];

    for (name, params, names, return_type) in overloaded_builtins {
        let fn_type = Type::Function(FunctionType::with_names(
            params,
            names.into_iter().map(str::to_string).collect(),
            return_type,
        ));
        let _ = table.declare_builtin(Symbol::new(name, SymbolKind::Function, fn_type, None));
    }
}
