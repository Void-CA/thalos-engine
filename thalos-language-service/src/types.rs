use std::hash::{Hash, Hasher};

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FunctionType {
    pub params: Vec<Type>,
    /// Parameter names, used for named-argument binding, diagnostics and tooling.
    ///
    /// They are deliberately **excluded from equality and hashing**: two
    /// signatures that differ only in parameter names are the same overload.
    #[serde(default)]
    pub param_names: Vec<String>,
    pub return_type: Box<Type>,
}

impl FunctionType {
    /// Build a signature with positional parameters only (no names).
    pub fn new(params: Vec<Type>, return_type: Type) -> Self {
        Self {
            params,
            param_names: Vec::new(),
            return_type: Box::new(return_type),
        }
    }

    /// Build a signature with named parameters.
    ///
    /// `param_names` is parallel to `params`; a shorter list is allowed and the
    /// missing trailing names are simply absent.
    pub fn with_names(params: Vec<Type>, param_names: Vec<String>, return_type: Type) -> Self {
        Self {
            params,
            param_names,
            return_type: Box::new(return_type),
        }
    }

    /// Name of the parameter at `index`, if one was declared.
    pub fn name_of(&self, index: usize) -> Option<&str> {
        self.param_names.get(index).map(String::as_str)
    }
}

impl PartialEq for FunctionType {
    fn eq(&self, other: &Self) -> bool {
        self.params == other.params && self.return_type == other.return_type
    }
}

impl Eq for FunctionType {}

impl Hash for FunctionType {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.params.hash(state);
        self.return_type.hash(state);
    }
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

    // Derived physical units
    /// `Length / Duration` (m/s).
    Speed,
    /// `Angle / Duration` (rad/s).
    AngularSpeed,

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

    /// Numeric widening: an `Int` value satisfies a `Float` expectation.
    ///
    /// This is **not** subtyping and **not** dimensional coercion — it only
    /// promotes between the two numeric representations of the `Number`
    /// category. `Float` never satisfies `Int`, and no physical dimension is
    /// ever silently satisfied by a `Number`.
    pub fn accepts_numeric_widening(&self, actual: &Type) -> bool {
        self == actual || (*self == Type::Float && *actual == Type::Int)
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
            "Speed" => Some(Type::Speed),
            "AngularSpeed" => Some(Type::AngularSpeed),
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    fn hash_of<T: Hash>(value: &T) -> u64 {
        let mut hasher = DefaultHasher::new();
        value.hash(&mut hasher);
        hasher.finish()
    }

    #[test]
    fn parameter_names_do_not_affect_signature_identity() {
        let named = FunctionType::with_names(
            vec![Type::Length],
            vec!["x".to_string()],
            Type::Position,
        );
        let differently_named = FunctionType::with_names(
            vec![Type::Length],
            vec!["distance".to_string()],
            Type::Position,
        );
        let unnamed = FunctionType::new(vec![Type::Length], Type::Position);

        assert_eq!(named, differently_named);
        assert_eq!(named, unnamed);
        assert_eq!(hash_of(&named), hash_of(&differently_named));
        assert_eq!(hash_of(&named), hash_of(&unnamed));
    }

    #[test]
    fn differing_parameter_types_still_break_identity() {
        let length_param = FunctionType::new(vec![Type::Length], Type::Position);
        let angle_param = FunctionType::new(vec![Type::Angle], Type::Position);
        assert_ne!(length_param, angle_param);
    }

    #[test]
    fn name_of_reads_the_parallel_name_list() {
        let sig = FunctionType::with_names(
            vec![Type::Length, Type::Length, Type::Length],
            vec!["x".to_string(), "y".to_string(), "z".to_string()],
            Type::Position,
        );
        assert_eq!(sig.name_of(0), Some("x"));
        assert_eq!(sig.name_of(2), Some("z"));
        assert_eq!(sig.name_of(3), None);
    }
}
