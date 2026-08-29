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
}

impl Type {
    pub fn is_spatial_target(&self) -> bool {
        matches!(self, Type::Position | Type::Pose)
    }

    pub fn is_target(&self) -> bool {
        matches!(self, Type::Position | Type::Pose | Type::Joints { .. })
    }
}
