use thalos_lang::ast::BinaryOp;
use crate::types::Type;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BinaryOpRule;

impl BinaryOpRule {
    pub fn infer(lhs: &Type, op: BinaryOp, rhs: &Type) -> Result<Type, String> {
        match (lhs, op, rhs) {
            // Position operations
            (Type::Position, BinaryOp::Add, Type::Vector3) => Ok(Type::Position),
            (Type::Position, BinaryOp::Sub, Type::Vector3) => Ok(Type::Position),
            (Type::Position, BinaryOp::Sub, Type::Position) => Ok(Type::Vector3),

            // Pose operations
            (Type::Pose, BinaryOp::Add, Type::Vector3) => Ok(Type::Pose),
            (Type::Pose, BinaryOp::Sub, Type::Vector3) => Ok(Type::Pose),
            (Type::Pose, BinaryOp::Mul, Type::Transform3D) => Ok(Type::Pose),

            // Transform3D operations
            (Type::Transform3D, BinaryOp::Mul, Type::Transform3D) => Ok(Type::Transform3D),

            // Quaternion operations
            (Type::Quaternion, BinaryOp::Mul, Type::Quaternion) => Ok(Type::Quaternion),
            (Type::Quaternion, BinaryOp::Mul, Type::Vector3) => Ok(Type::Vector3),

            // Vector3 operations
            (Type::Vector3, BinaryOp::Add, Type::Vector3) => Ok(Type::Vector3),
            (Type::Vector3, BinaryOp::Sub, Type::Vector3) => Ok(Type::Vector3),
            (Type::Vector3, BinaryOp::Mul, Type::Float | Type::Int) => Ok(Type::Vector3),
            (Type::Float | Type::Int, BinaryOp::Mul, Type::Vector3) => Ok(Type::Vector3),
            (Type::Vector3, BinaryOp::Div, Type::Float | Type::Int) => Ok(Type::Vector3),

            // Physical Units operations
            (Type::Length, BinaryOp::Add | BinaryOp::Sub, Type::Length) => Ok(Type::Length),
            (Type::Length, BinaryOp::Mul, Type::Float | Type::Int) => Ok(Type::Length),
            (Type::Float | Type::Int, BinaryOp::Mul, Type::Length) => Ok(Type::Length),

            (Type::Angle, BinaryOp::Add | BinaryOp::Sub, Type::Angle) => Ok(Type::Angle),
            (Type::Angle, BinaryOp::Mul, Type::Float | Type::Int) => Ok(Type::Angle),
            (Type::Float | Type::Int, BinaryOp::Mul, Type::Angle) => Ok(Type::Angle),

            (Type::Duration, BinaryOp::Add | BinaryOp::Sub, Type::Duration) => Ok(Type::Duration),
            (Type::Duration, BinaryOp::Mul, Type::Float | Type::Int) => Ok(Type::Duration),
            (Type::Float | Type::Int, BinaryOp::Mul, Type::Duration) => Ok(Type::Duration),

            // Primitive Scalar operations
            (Type::Float, _, Type::Float) => Ok(Type::Float),
            (Type::Int, _, Type::Int) => Ok(Type::Int),

            _ => Err(format!(
                "Invalid binary operation '{:?}' between types {:?} and {:?}",
                op, lhs, rhs
            )),
        }
    }
}
