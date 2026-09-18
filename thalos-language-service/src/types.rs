use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FunctionType {
    pub params: Vec<Type>,
    pub return_type: Box<Type>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Type {
    // Primitives
    Bool,
    Int,
    Float,
    String,

    // Physical Units
    Length,
    Angle,
    Duration,

    // Geometry
    Vector3,
    Quaternion,
    Transform3D,

    // Robot Semantics
    Position,
    Pose,
    Joints { dimension: Option<usize> },

    // Functions & Special
    Function(FunctionType),
    Unit,

    /// Internal poison type: the expression's type could not be inferred
    /// because an error was already reported for it. It suppresses cascading
    /// diagnostics and is never surfaced to the user.
    Error,
}

impl Type {
    pub fn is_spatial_target(&self) -> bool {
        matches!(self, Type::Position | Type::Pose)
    }

    pub fn is_target(&self) -> bool {
        matches!(self, Type::Position | Type::Pose | Type::Joints { .. })
    }

    /// True when this type is the internal error poison. Used to avoid emitting
    /// diagnostics that are merely downstream consequences of an already
    /// reported error.
    pub fn is_error(&self) -> bool {
        matches!(self, Type::Error)
    }

    pub fn from_name(name: &str) -> Option<Type> {
        match name {
            "Bool" => Some(Type::Bool),
            "Int" => Some(Type::Int),
            "Float" => Some(Type::Float),
            "String" => Some(Type::String),
            "Length" => Some(Type::Length),
            "Angle" => Some(Type::Angle),
            "Duration" => Some(Type::Duration),
            "Vector3" => Some(Type::Vector3),
            "Quaternion" => Some(Type::Quaternion),
            "Transform3D" => Some(Type::Transform3D),
            "Position" => Some(Type::Position),
            "Pose" => Some(Type::Pose),
            "Joints" => Some(Type::Joints { dimension: None }),
            "Unit" => Some(Type::Unit),
            _ => None,
        }
    }
}
