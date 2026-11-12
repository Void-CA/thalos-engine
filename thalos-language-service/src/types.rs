use std::fmt;
use std::hash::{Hash, Hasher};

use serde::{Deserialize, Serialize};
use thalos_lang::ast::{TypeArg, TypeExpr};
use thalos_lang::span::Span;

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

/// The dimension of a `Joints` configuration.
///
/// `Unknown` is **not** “any dimension”: it means the analysis does not know the
/// dimension. It is compatible at assignment/call boundaries but never enables an
/// operation that requires knowing the dimension.
///
/// Serde keeps the historical wire shape (`4` or `null`) so the frontend contract
/// does not change in Fase 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Dimension {
    Known(usize),
    Unknown,
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
    Joints { dimension: Dimension },

    // Functions & Special
    Function(FunctionType),
    Unit,

    /// Analysis state: the type is not known yet. This is not a value type and
    /// cannot be written as an annotation; it appears when a declaration omits
    /// its annotation (parameters) or when inference cannot determine a type.
    Unknown,

    /// Internal poison type: the expression's type could not be inferred
    /// because an error was already reported for it. It suppresses cascading
    /// diagnostics and never crosses the compilation boundary.
    Error,
}

/// A type annotation that could not be resolved.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeAnnotationError {
    pub message: String,
    pub span: Option<Span>,
}

impl TypeAnnotationError {
    fn new(message: impl Into<String>, span: Span) -> Self {
        Self {
            message: message.into(),
            span: Some(span),
        }
    }
}

/// Resolve a syntactic [`TypeExpr`] into a semantic [`Type`].
///
/// This is the single entry point for every annotation site: `let`, `const`,
/// function/work parameters, return types and process `state`. Unknown names and
/// invalid type arguments are errors, never silent fallbacks.
pub fn resolve_type(annotation: &TypeExpr) -> Result<Type, TypeAnnotationError> {
    match annotation {
        TypeExpr::Name { name, span } => Type::from_name(name)
            .ok_or_else(|| TypeAnnotationError::new(format!("Unknown type annotation '{name}'"), *span)),
        TypeExpr::Applied { name, args, span } => {
            if name != "Joints" {
                return Err(TypeAnnotationError::new(
                    format!("Type '{name}' does not take type arguments"),
                    *span,
                ));
            }
            if args.len() != 1 {
                return Err(TypeAnnotationError::new(
                    "Joints expects exactly one dimension argument",
                    *span,
                ));
            }
            match &args[0] {
                TypeArg::Integer(dimension) if *dimension >= 1 => Ok(Type::Joints {
                    dimension: Dimension::Known(*dimension),
                }),
                TypeArg::Integer(_) => Err(TypeAnnotationError::new(
                    "Joints dimension must be greater than or equal to 1",
                    *span,
                )),
                TypeArg::Type(_) => Err(TypeAnnotationError::new(
                    "Joints dimension must be an integer literal",
                    *span,
                )),
            }
        }
    }
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

    pub fn is_unknown(&self) -> bool {
        matches!(self, Type::Unknown)
    }

    /// The single compatibility rule: `expected.accepts(actual)`.
    ///
    /// - `Int` satisfies `Float` (numeric promotion).
    /// - `Joints<Unknown>` accepts a known dimension at a boundary.
    /// - A known dimension never accepts an unknown one (it cannot be proven).
    /// - `Unknown` (an unannotated parameter) accepts anything at the boundary.
    /// - `Error` poisons compatibility; callers should check `is_error` first.
    ///
    /// This method deliberately contains **no per-operation exceptions**. A
    /// command such as `movej` accepts `Joints<Unknown>` through its signature,
    /// not by special-casing compatibility.
    pub fn accepts(&self, actual: &Type) -> bool {
        match (self, actual) {
            (Type::Error, _) | (_, Type::Error) => false,
            (Type::Unknown, _) => true,
            (_, Type::Unknown) => false,
            (Type::Float, Type::Int) => true,
            (Type::Joints { dimension: Dimension::Unknown }, Type::Joints { .. }) => true,
            (
                Type::Joints { dimension: Dimension::Known(expected) },
                Type::Joints { dimension: Dimension::Known(actual) },
            ) => expected == actual,
            (
                Type::Joints { dimension: Dimension::Known(_) },
                Type::Joints { dimension: Dimension::Unknown },
            ) => false,
            _ => self == actual,
        }
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
            "Joints" => Some(Type::Joints {
                dimension: Dimension::Unknown,
            }),
            "Unit" => Some(Type::Unit),
            _ => None,
        }
    }
}

impl fmt::Display for Type {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Type::Bool => f.write_str("Bool"),
            Type::Int => f.write_str("Int"),
            Type::Float => f.write_str("Float"),
            Type::String => f.write_str("String"),
            Type::Length => f.write_str("Length"),
            Type::Angle => f.write_str("Angle"),
            Type::Duration => f.write_str("Duration"),
            Type::Speed => f.write_str("Speed"),
            Type::AngularSpeed => f.write_str("AngularSpeed"),
            Type::Vector3 => f.write_str("Vector3"),
            Type::Quaternion => f.write_str("Quaternion"),
            Type::Transform3D => f.write_str("Transform3D"),
            Type::Position => f.write_str("Position"),
            Type::Pose => f.write_str("Pose"),
            Type::Joints { dimension: Dimension::Known(n) } => write!(f, "Joints<{n}>"),
            Type::Joints { dimension: Dimension::Unknown } => f.write_str("Joints"),
            Type::Function(signature) => {
                let params = signature
                    .params
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(", ");
                write!(f, "({params}) -> {}", signature.return_type)
            }
            Type::Unit => f.write_str("Unit"),
            Type::Unknown => f.write_str("Unknown"),
            Type::Error => f.write_str("Error"),
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

    fn name(name: &str, start: usize) -> TypeExpr {
        TypeExpr::Name {
            name: name.to_string(),
            span: Span::new(start, start + name.len()),
        }
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

    #[test]
    fn resolves_plain_and_applied_annotations() {
        assert_eq!(resolve_type(&name("Position", 0)), Ok(Type::Position));
        assert_eq!(
            resolve_type(&name("Joints", 0)),
            Ok(Type::Joints {
                dimension: Dimension::Unknown
            })
        );

        let applied = TypeExpr::Applied {
            name: "Joints".to_string(),
            args: vec![TypeArg::Integer(4)],
            span: Span::new(0, 9),
        };
        assert_eq!(
            resolve_type(&applied),
            Ok(Type::Joints {
                dimension: Dimension::Known(4)
            })
        );
    }

    #[test]
    fn rejects_unknown_and_invalid_annotations() {
        let unknown = resolve_type(&name("Foo", 3)).expect_err("unknown type");
        assert_eq!(unknown.message, "Unknown type annotation 'Foo'");
        assert_eq!(unknown.span, Some(Span::new(3, 6)));

        let zero = TypeExpr::Applied {
            name: "Joints".to_string(),
            args: vec![TypeArg::Integer(0)],
            span: Span::new(0, 9),
        };
        assert!(resolve_type(&zero).is_err());

        let not_applicable = TypeExpr::Applied {
            name: "Position".to_string(),
            args: vec![TypeArg::Integer(4)],
            span: Span::new(0, 12),
        };
        assert!(resolve_type(&not_applicable).is_err());

        let non_integer = TypeExpr::Applied {
            name: "Joints".to_string(),
            args: vec![TypeArg::Type(name("Position", 7))],
            span: Span::new(0, 16),
        };
        assert!(resolve_type(&non_integer).is_err());
    }

    #[test]
    fn joints_compatibility_separates_boundary_from_knowledge() {
        let unknown = Type::Joints {
            dimension: Dimension::Unknown,
        };
        let four = Type::Joints {
            dimension: Dimension::Known(4),
        };
        let six = Type::Joints {
            dimension: Dimension::Known(6),
        };

        assert!(unknown.accepts(&four), "unknown accepts a known dimension at a boundary");
        assert!(four.accepts(&four));
        assert!(!four.accepts(&six));
        assert!(!four.accepts(&unknown), "a known dimension cannot be proven against unknown");
        assert!(Type::Unknown.accepts(&four));
        assert!(!four.accepts(&Type::Unknown));
    }

    #[test]
    fn numeric_widening_is_the_only_promotion() {
        assert!(Type::Float.accepts(&Type::Int));
        assert!(!Type::Int.accepts(&Type::Float));
        assert!(!Type::Length.accepts(&Type::Int));
    }

    #[test]
    fn display_is_the_language_surface() {
        assert_eq!(Type::Position.to_string(), "Position");
        assert_eq!(
            Type::Joints {
                dimension: Dimension::Known(4)
            }
            .to_string(),
            "Joints<4>"
        );
        assert_eq!(
            Type::Joints {
                dimension: Dimension::Unknown
            }
            .to_string(),
            "Joints"
        );
        assert_eq!(
            Type::Function(FunctionType::new(vec![Type::Length], Type::Position)).to_string(),
            "(Length) -> Position"
        );
    }

    #[test]
    fn dimension_keeps_the_legacy_wire_shape() {
        let known = serde_json::to_value(Dimension::Known(4)).expect("serialize");
        assert_eq!(known, serde_json::json!(4));
        let unknown = serde_json::to_value(Dimension::Unknown).expect("serialize");
        assert_eq!(unknown, serde_json::json!(null));

        assert_eq!(
            serde_json::from_value::<Dimension>(serde_json::json!(4)).expect("deserialize"),
            Dimension::Known(4)
        );
        assert_eq!(
            serde_json::from_value::<Dimension>(serde_json::json!(null)).expect("deserialize"),
            Dimension::Unknown
        );
    }
}
