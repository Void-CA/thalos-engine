//! Central registry of type constructors and member specifications.
//!
//! Fases 2–3 moved the *declaration* of constructors and members out of ad-hoc
//! string matches in the checker, evaluator and binder. The registry is the
//! single place that answers:
//!
//! - which constructors exist and with which signatures (`ConstructorSpec`);
//! - how a constructor binds its arguments (`ConstructorBinding`);
//! - which members a value type exposes (`MemberSpec`).
//!
//! Value construction itself still lives in the evaluator, dispatching on the
//! typed [`ConstructorId`]. That keeps the evaluator from re-deciding *types*
//! while avoiding a dynamic callback table that would hide the value semantics.

use std::collections::HashMap;
use std::sync::OnceLock;

use crate::member_schema::MemberSchema;
use crate::scope::SymbolTable;
use crate::symbols::{Symbol, SymbolKind};
use crate::types::{Dimension, FunctionType, Type};

/// The member specification of a type. `MemberSchema` is the concrete shape:
/// fields (readable data) plus operations (receiver-directed methods).
pub type MemberSpec = MemberSchema;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConstructorId {
    Vector3,
    Position,
    Pose,
    Euler,
    Quaternion,
    Joints,
    Offset,
}

/// How a constructor accepts its parameters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConstructorShape {
    /// Fixed arity/type parameters, matched by the signature.
    Fixed,
    /// One repeated parameter type (`joints(Angle, ...)`).
    Variadic { param: Type },
    /// A member operation whose shape is directed by the receiver
    /// (`offset(receiver, ...)`).
    ReceiverMethod,
}

/// How the binder normalizes named arguments for this constructor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstructorBinding {
    /// Ordinary positional/named binding: names resolve against `param_names`.
    Positional,
    /// `jN = value` slots (`joints`).
    JointIndex,
    /// Preserve named arguments until the receiver type is known (`offset`).
    ReceiverDirected,
}

#[derive(Debug, Clone)]
pub struct ConstructorSpec {
    pub id: ConstructorId,
    pub name: &'static str,
    pub params: Vec<Type>,
    pub param_names: Vec<String>,
    pub returns: Type,
    pub shape: ConstructorShape,
    pub binding: ConstructorBinding,
    /// Lowercase compatibility alias kept for one release.
    pub deprecated: bool,
}

impl ConstructorSpec {
    pub fn function_type(&self) -> FunctionType {
        FunctionType::with_names(
            self.params.clone(),
            self.param_names.clone(),
            self.returns.clone(),
        )
    }

    pub fn is_fixed(&self) -> bool {
        matches!(self.shape, ConstructorShape::Fixed)
    }
}

/// Metadata for one type (identity + presentation + members).
#[derive(Debug, Clone)]
pub struct TypeSpec {
    pub ty: Type,
    pub name: String,
    pub members: Option<MemberSpec>,
}

/// Constructor and member registry. Built once from the language's primitive
/// types; adding a constructor or a member type is a change in this module plus
/// its value builder, not in the checker/evaluator dispatch.
#[derive(Debug)]
pub struct TypeRegistry {
    constructors: HashMap<&'static str, Vec<ConstructorSpec>>,
}

impl TypeRegistry {
    pub fn builtin() -> Self {
        let mut constructors: HashMap<&'static str, Vec<ConstructorSpec>> = HashMap::new();
        let mut add = |spec: ConstructorSpec| {
            constructors.entry(spec.name).or_default().push(spec);
        };

        let fixed = |id, name, params, names: &[&str], returns, deprecated| ConstructorSpec {
            id,
            name,
            params,
            param_names: names.iter().map(|name| name.to_string()).collect(),
            returns,
            shape: ConstructorShape::Fixed,
            binding: ConstructorBinding::Positional,
            deprecated,
        };

        add(fixed(
            ConstructorId::Vector3,
            "Vector3",
            vec![Type::Length, Type::Length, Type::Length],
            &["x", "y", "z"],
            Type::Vector3,
            false,
        ));
        add(fixed(
            ConstructorId::Position,
            "Position",
            vec![Type::Length, Type::Length, Type::Length],
            &["x", "y", "z"],
            Type::Position,
            false,
        ));
        add(fixed(
            ConstructorId::Position,
            "Position",
            vec![Type::Vector3],
            &["vector"],
            Type::Position,
            false,
        ));
        add(fixed(
            ConstructorId::Pose,
            "Pose",
            vec![Type::Vector3, Type::Quaternion],
            &["position", "orientation"],
            Type::Pose,
            false,
        ));
        add(fixed(
            ConstructorId::Pose,
            "Pose",
            vec![Type::Position, Type::Quaternion],
            &["position", "orientation"],
            Type::Pose,
            false,
        ));
        add(fixed(
            ConstructorId::Quaternion,
            "Quaternion",
            vec![Type::Float, Type::Float, Type::Float, Type::Float],
            &["w", "x", "y", "z"],
            Type::Quaternion,
            false,
        ));
        add(fixed(
            ConstructorId::Euler,
            "Quaternion.euler",
            vec![Type::Angle, Type::Angle, Type::Angle],
            &["roll", "pitch", "yaw"],
            Type::Quaternion,
            false,
        ));
        add(ConstructorSpec {
            id: ConstructorId::Joints,
            name: "Joints",
            params: Vec::new(),
            param_names: Vec::new(),
            returns: Type::Joints {
                dimension: Dimension::Unknown,
            },
            shape: ConstructorShape::Variadic {
                param: Type::Angle,
            },
            binding: ConstructorBinding::JointIndex,
            deprecated: false,
        });

        // Lowercase compatibility aliases, deprecated for one release.
        add(fixed(
            ConstructorId::Position,
            "position",
            vec![Type::Vector3],
            &["vec"],
            Type::Position,
            true,
        ));
        add(fixed(
            ConstructorId::Pose,
            "pose",
            vec![Type::Vector3, Type::Quaternion],
            &["position", "orientation"],
            Type::Pose,
            true,
        ));
        add(fixed(
            ConstructorId::Pose,
            "pose",
            vec![Type::Position, Type::Quaternion],
            &["position", "orientation"],
            Type::Pose,
            true,
        ));
        add(fixed(
            ConstructorId::Euler,
            "euler",
            vec![Type::Angle, Type::Angle, Type::Angle],
            &["roll", "pitch", "yaw"],
            Type::Quaternion,
            true,
        ));
        add(fixed(
            ConstructorId::Quaternion,
            "quaternion",
            vec![Type::Float, Type::Float, Type::Float, Type::Float],
            &["w", "x", "y", "z"],
            Type::Quaternion,
            true,
        ));
        add(ConstructorSpec {
            id: ConstructorId::Joints,
            name: "joints",
            params: Vec::new(),
            param_names: Vec::new(),
            returns: Type::Joints {
                dimension: Dimension::Unknown,
            },
            shape: ConstructorShape::Variadic {
                param: Type::Angle,
            },
            binding: ConstructorBinding::JointIndex,
            deprecated: true,
        });
        add(ConstructorSpec {
            id: ConstructorId::Offset,
            name: "offset",
            params: Vec::new(),
            param_names: Vec::new(),
            // The receiver decides the result type; the checker resolves it.
            returns: Type::Unknown,
            shape: ConstructorShape::ReceiverMethod,
            binding: ConstructorBinding::ReceiverDirected,
            deprecated: false,
        });

        Self { constructors }
    }

    pub fn constructor_overloads(&self, name: &str) -> Option<&[ConstructorSpec]> {
        self.constructors.get(name).map(Vec::as_slice)
    }

    pub fn constructor(&self, name: &str) -> Option<&ConstructorSpec> {
        self.constructor_overloads(name).and_then(|specs| specs.first())
    }

    pub fn is_variadic(&self, name: &str) -> bool {
        self.constructor(name)
            .is_some_and(|spec| matches!(spec.shape, ConstructorShape::Variadic { .. }))
    }

    pub fn is_receiver_method(&self, name: &str) -> bool {
        self.constructor(name)
            .is_some_and(|spec| matches!(spec.shape, ConstructorShape::ReceiverMethod))
    }

    pub fn binding_of(&self, name: &str) -> Option<ConstructorBinding> {
        self.constructor(name).map(|spec| spec.binding)
    }

    /// Lowercase constructor names kept as compatibility aliases.
    pub fn legacy_constructor_names(&self) -> Vec<&'static str> {
        let mut names: Vec<&'static str> = self
            .constructors
            .iter()
            .filter(|(_, specs)| specs.iter().any(|spec| spec.deprecated))
            .map(|(name, _)| *name)
            .collect();
        names.sort_unstable();
        names
    }

    /// Canonical replacement for a deprecated constructor name.
    pub fn legacy_replacement(&self, name: &str) -> Option<&'static str> {
        let spec = self.constructor(name)?;
        if !spec.deprecated {
            return None;
        }
        Some(match spec.id {
            ConstructorId::Vector3 => "Vector3",
            ConstructorId::Position => "Position",
            ConstructorId::Pose => "Pose",
            ConstructorId::Euler => "Quaternion.euler",
            ConstructorId::Quaternion => "Quaternion",
            ConstructorId::Joints => "Joints",
            ConstructorId::Offset => "offset",
        })
    }

    /// Member specification for a checked type, if it has one.
    pub fn member_spec(&self, ty: &Type) -> Option<MemberSpec> {
        ty.member_schema()
    }

    /// Metadata for a checked type.
    pub fn type_spec(&self, ty: &Type) -> TypeSpec {
        TypeSpec {
            ty: ty.clone(),
            name: ty.to_string(),
            members: ty.member_schema(),
        }
    }

    /// Register the fixed-arity constructors in the symbol table so builtins
    /// are visible to tooling (hover, signatures, completion). Variadic and
    /// receiver-directed constructors are not `FunctionType` signatures.
    pub fn register_constructors(&self, table: &mut SymbolTable) {
        for overloads in self.constructors.values() {
            for spec in overloads.iter().filter(|spec| spec.is_fixed()) {
                let _ = table.declare_builtin(Symbol::new(
                    spec.name,
                    SymbolKind::Function,
                    Type::Function(spec.function_type()),
                    None,
                ));
            }
        }
    }
}

static REGISTRY: OnceLock<TypeRegistry> = OnceLock::new();

/// The process-wide builtin registry.
pub fn registry() -> &'static TypeRegistry {
    REGISTRY.get_or_init(TypeRegistry::builtin)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructors_have_their_declared_shapes() {
        let registry = registry();
        let position = registry.constructor_overloads("position").expect("position");
        assert_eq!(position.len(), 1);
        assert_eq!(position[0].params, vec![Type::Vector3]);
        assert_eq!(position[0].returns, Type::Position);

        let pose = registry.constructor_overloads("pose").expect("pose");
        assert_eq!(pose.len(), 2, "pose has Vector3 and Position overloads");
        assert!(pose.iter().all(|spec| spec.returns == Type::Pose));

        let joints = registry.constructor("joints").expect("joints");
        assert!(matches!(
            joints.shape,
            ConstructorShape::Variadic { ref param } if *param == Type::Angle
        ));
        assert_eq!(joints.binding, ConstructorBinding::JointIndex);

        let offset = registry.constructor("offset").expect("offset");
        assert!(matches!(offset.shape, ConstructorShape::ReceiverMethod));
        assert_eq!(offset.binding, ConstructorBinding::ReceiverDirected);
    }

    #[test]
    fn member_specs_are_centralized() {
        let registry = registry();
        let position = registry.type_spec(&Type::Position);
        assert_eq!(position.name, "Position");
        let members = position.members.expect("Position members");
        assert!(members.has("x"));
        assert!(members.has_operation("offset"));

        let unknown_joints = registry.type_spec(&Type::Joints {
            dimension: Dimension::Unknown,
        });
        assert!(unknown_joints.members.is_none());
    }

    #[test]
    fn fixed_constructors_are_registered_as_builtins() {
        let mut table = SymbolTable::new();
        registry().register_constructors(&mut table);
        assert!(table.lookup("position").is_some());
        assert!(table.lookup("pose").is_some());
        assert!(table.lookup("euler").is_some());
        assert!(table.lookup("quaternion").is_some());
        assert!(table.lookup("joints").is_none(), "variadic is not a FunctionType");
        assert!(table.lookup("offset").is_none(), "receiver method is not a FunctionType");
    }
}
