//! Value member schema — the single concept behind field access, named
//! arguments and member operations.
//!
//! A [`MemberSchema`] describes which names a typed value exposes and how they
//! behave:
//!
//! ```text
//! MemberSchema(type)
//!   fields:     x: Length | j1: Angle | ...
//!   operations: offset(...)
//!
//!   PTT.x          → field  → Length
//!   offset(PTT, x=) → schema validates/binds the name
//!   PTT.offset(x=)  → resolves the operation, then reuses the schema for args
//! ```
//!
//! This module only models the schema. Consumers (the checker, the evaluator,
//! accessors and member calls) are added incrementally; see
//! `.opencode/plans/thls-value-member-schema.md`.

use serde::{Deserialize, Serialize};

use crate::types::{FunctionType, Type};

/// Whether a member is readable data or a callable operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MemberKind {
    Field,
    Operation,
}

/// A single named member of a value type.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Member {
    pub name: String,
    pub kind: MemberKind,
    /// The member's value type. For an [`MemberKind::Operation`] this is the
    /// operation's return type.
    pub ty: Type,
    /// Present for operations; absent for plain fields.
    pub signature: Option<FunctionType>,
}

impl Member {
    pub fn field(name: impl Into<String>, ty: Type) -> Self {
        Self {
            name: name.into(),
            kind: MemberKind::Field,
            ty,
            signature: None,
        }
    }

    pub fn operation(name: impl Into<String>, signature: FunctionType) -> Self {
        let return_type = (*signature.return_type).clone();
        Self {
            name: name.into(),
            kind: MemberKind::Operation,
            ty: return_type,
            signature: Some(signature),
        }
    }
}

/// The ordered set of members exposed by a value type.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MemberSchema {
    pub members: Vec<Member>,
}

impl MemberSchema {
    pub fn new(members: Vec<Member>) -> Self {
        Self { members }
    }

    pub fn is_empty(&self) -> bool {
        self.members.is_empty()
    }

    pub fn get(&self, name: &str) -> Option<&Member> {
        self.members.iter().find(|m| m.name == name)
    }

    pub fn has(&self, name: &str) -> bool {
        self.get(name).is_some()
    }

    /// Type of a readable field, if `name` is a field of this value.
    pub fn field_ty(&self, name: &str) -> Option<&Type> {
        self.get(name).filter(|m| m.kind == MemberKind::Field).map(|m| &m.ty)
    }

    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.members.iter().map(|m| m.name.as_str())
    }

    /// `x`, `y`, `z: Length` — the geometry of a `Vector3`, `Position` or `Pose`.
    pub fn xyz() -> Self {
        Self::new(vec![
            Member::field("x", Type::Length),
            Member::field("y", Type::Length),
            Member::field("z", Type::Length),
        ])
    }

    /// `j1..jN: Angle` for an N-dimensional joint configuration.
    ///
    /// The index carries operational meaning (`j2` is joint position 2), not the
    /// robot's physical joint name. Joint-kind awareness (revolute → `Angle`,
    /// prismatic → `Length`) is deferred until a robot joint-kind profile exists;
    /// for now every joint member is typed `Angle`.
    pub fn joints(dimension: usize) -> Self {
        Self::new(
            (1..=dimension)
                .map(|i| Member::field(format!("j{i}"), Type::Angle))
                .collect(),
        )
    }
}

impl Type {
    /// The value member schema for this type, if it exposes named members.
    ///
    /// `Joints` without a known dimension has no concrete schema (its arity, and
    /// therefore its member set, is unknown). The `Pose` schema currently models
    /// only translation; rotation members are deferred.
    pub fn member_schema(&self) -> Option<MemberSchema> {
        match self {
            Type::Vector3 | Type::Position | Type::Pose => Some(MemberSchema::xyz()),
            Type::Joints { dimension: Some(n) } => Some(MemberSchema::joints(*n)),
            Type::Joints { dimension: None } => None,
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_exposes_xyz_fields() {
        let schema = Type::Position.member_schema().expect("Position has a schema");
        assert_eq!(schema.names().collect::<Vec<_>>(), vec!["x", "y", "z"]);
        assert_eq!(schema.field_ty("x"), Some(&Type::Length));
        assert_eq!(schema.field_ty("j2"), None);
    }

    #[test]
    fn joints_schema_is_derived_from_dimension() {
        let schema = Type::Joints { dimension: Some(4) }
            .member_schema()
            .expect("Joints<4> has a schema");
        assert_eq!(schema.names().collect::<Vec<_>>(), vec!["j1", "j2", "j3", "j4"]);
        assert_eq!(schema.field_ty("j4"), Some(&Type::Angle));
        assert_eq!(schema.field_ty("j5"), None);
    }

    #[test]
    fn joints_without_dimension_has_no_schema() {
        assert!(Type::Joints { dimension: None }.member_schema().is_none());
    }

    #[test]
    fn non_structured_types_have_no_schema() {
        assert!(Type::Length.member_schema().is_none());
        assert!(Type::Quaternion.member_schema().is_none());
        assert!(Type::Bool.member_schema().is_none());
    }
}
