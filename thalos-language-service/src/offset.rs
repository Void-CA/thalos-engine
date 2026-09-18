//! `offset` — a component-wise delta on a typed value.
//!
//! `offset(receiver, ...)` produces a value of the **same type** as its receiver:
//!
//! - `Position` / `Vector3`: components `x`, `y`, `z` (`Length`)
//! - `Joints<N>`: components `j1..jN` (`Angle`)
//!
//! Two delta shapes are accepted, and they must not be mixed:
//!
//! - positional **full** delta: `offset(p, [dx, dy, dz])`, `offset(j, d1..dN)`
//! - named **partial** delta: `offset(p, x = -10mm)`, `offset(j, j2 = 5deg)`
//!
//! Omitted named components mean a **zero delta** (component unchanged), not a
//! generic default parameter.
//!
//! Argument-name resolution is receiver-type-directed: the receiver's type
//! decides both the valid names and the delta shape. That is why binding lives
//! here rather than in the syntactic [`crate::binder`], which runs before types
//! are known. The checker and the evaluator both call into this module, so the
//! semantics cannot diverge between them.

use thalos_lang::ast::{Arg, Expr};
use thalos_lang::units::{AngleRadians, LengthMeters};

use crate::evaluator::{CompileTimeValue, Position};
use crate::types::Type;

/// The shape of an `offset` delta after canonicalization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeltaShape {
    /// A single `Vector3` delta (`Position` / `Vector3`).
    Vector3,
    /// One scalar per joint (`Joints<N>`).
    Scalars,
}

/// Canonical (positional) delta arguments for an `offset` call.
#[derive(Debug, Clone, PartialEq)]
pub struct CanonicalDelta {
    pub shape: DeltaShape,
    pub args: Vec<Arg>,
}

/// Canonicalize the delta arguments of `offset(receiver, ...)` against the
/// receiver's type. `delta_args` excludes the receiver (the first argument).
pub fn canonicalize_delta(receiver: &Type, delta_args: &[Arg]) -> Result<CanonicalDelta, String> {
    match receiver {
        Type::Position | Type::Vector3 => canonicalize_xyz(delta_args),
        Type::Joints { dimension: Some(n) } => canonicalize_joints(*n, delta_args),
        Type::Joints { dimension: None } => {
            Err("offset requires a Joints receiver with a known dimension".to_string())
        }
        other => Err(format!(
            "offset expected a Position, Vector3, or Joints receiver, got {:?}",
            other
        )),
    }
}

fn canonicalize_xyz(delta_args: &[Arg]) -> Result<CanonicalDelta, String> {
    let schema = Type::Vector3
        .member_schema()
        .expect("Vector3 has a member schema");
    let (any_named, any_positional) = arg_kinds(delta_args);
    if any_named && any_positional {
        return Err("offset(...) cannot mix named and positional delta arguments".to_string());
    }

    if any_positional {
        if delta_args.len() != 1 {
            return Err(format!(
                "offset expected a single Vector3 delta, got {} arguments",
                delta_args.len()
            ));
        }
        return Ok(CanonicalDelta {
            shape: DeltaShape::Vector3,
            args: vec![delta_args[0].clone()],
        });
    }

    let mut components: [Option<Expr>; 3] = [None, None, None];
    for arg in delta_args {
        let name = arg.name.as_deref().unwrap_or_default();
        match schema.field_index_of(name) {
            Some(index) => {
                if components[index].is_some() {
                    return Err(format!("Duplicate offset component '{}'", name));
                }
                components[index] = Some(arg.value.clone());
            }
            None => {
                return Err(format!(
                    "Unknown offset component '{}'; expected x, y, z",
                    name
                ));
            }
        }
    }

    let [x, y, z] = components;
    let vector = Expr::Vector3([
        Box::new(x.unwrap_or_else(zero_length)),
        Box::new(y.unwrap_or_else(zero_length)),
        Box::new(z.unwrap_or_else(zero_length)),
    ]);
    Ok(CanonicalDelta {
        shape: DeltaShape::Vector3,
        args: vec![Arg::positional(vector)],
    })
}

fn canonicalize_joints(dimension: usize, delta_args: &[Arg]) -> Result<CanonicalDelta, String> {
    let schema = Type::Joints {
        dimension: Some(dimension),
    }
    .member_schema()
    .expect("Joints has a member schema");
    let (any_named, any_positional) = arg_kinds(delta_args);
    if any_named && any_positional {
        return Err("offset(...) cannot mix named and positional delta arguments".to_string());
    }

    if any_positional {
        if delta_args.len() != dimension {
            return Err(format!(
                "offset expected {} joint deltas, got {}",
                dimension,
                delta_args.len()
            ));
        }
        return Ok(CanonicalDelta {
            shape: DeltaShape::Scalars,
            args: delta_args.to_vec(),
        });
    }

    let mut components: Vec<Option<Expr>> = vec![None; dimension];
    for arg in delta_args {
        let name = arg.name.as_deref().unwrap_or_default();
        match schema.field_index_of(name) {
            Some(index) => {
                if components[index].is_some() {
                    return Err(format!("Duplicate offset component '{}'", name));
                }
                components[index] = Some(arg.value.clone());
            }
            None => {
                return Err(format!(
                    "Unknown offset component '{}'; expected j1..j{}",
                    name, dimension
                ));
            }
        }
    }

    let args = components
        .into_iter()
        .map(|component| Arg::positional(component.unwrap_or_else(zero_angle)))
        .collect();
    Ok(CanonicalDelta {
        shape: DeltaShape::Scalars,
        args,
    })
}

/// Apply a canonical delta to a receiver value.
pub fn apply(
    receiver: &CompileTimeValue,
    delta: &[CompileTimeValue],
) -> Result<CompileTimeValue, String> {
    match receiver {
        CompileTimeValue::Position(p) => match delta.first() {
            Some(CompileTimeValue::Vector3(v)) => Ok(CompileTimeValue::Position(Position {
                point: p.point + v.clone(),
            })),
            _ => Err("offset on Position expected a Vector3 delta".to_string()),
        },
        CompileTimeValue::Vector3(base) => match delta.first() {
            Some(CompileTimeValue::Vector3(v)) => Ok(CompileTimeValue::Vector3(base.clone() + v.clone())),
            _ => Err("offset on Vector3 expected a Vector3 delta".to_string()),
        },
        CompileTimeValue::Joints(values) => {
            if values.len() != delta.len() {
                return Err(format!(
                    "offset on Joints<{}> got {} delta components",
                    values.len(),
                    delta.len()
                ));
            }
            let mut out = Vec::with_capacity(values.len());
            for (base, component) in values.iter().zip(delta.iter()) {
                out.push(base + scalar_of(component)?);
            }
            Ok(CompileTimeValue::Joints(out))
        }
        _ => Err("offset is not supported for this receiver type".to_string()),
    }
}

fn arg_kinds(args: &[Arg]) -> (bool, bool) {
    (
        args.iter().any(Arg::is_named),
        args.iter().any(|arg| !arg.is_named()),
    )
}

fn scalar_of(value: &CompileTimeValue) -> Result<f64, String> {
    match value {
        CompileTimeValue::Angle(a) => Ok(*a),
        CompileTimeValue::Length(l) => Ok(*l),
        CompileTimeValue::Float(f) => Ok(*f),
        CompileTimeValue::Int(i) => Ok(*i as f64),
        _ => Err("offset component expected an Angle, Length, or number".to_string()),
    }
}

fn zero_length() -> Expr {
    Expr::Length(LengthMeters(0.0))
}

fn zero_angle() -> Expr {
    Expr::Angle(AngleRadians(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_lang::units::AngleRadians;

    fn length(mm: f64) -> Expr {
        Expr::Length(LengthMeters(mm / 1000.0))
    }

    fn angle(deg: f64) -> Expr {
        Expr::Angle(AngleRadians(deg.to_radians()))
    }

    #[test]
    fn positional_vector_delta_passes_through() {
        let delta = vec![Arg::positional(length(10.0))];
        let canonical = canonicalize_delta(&Type::Position, &delta).unwrap();
        assert_eq!(canonical.shape, DeltaShape::Vector3);
        assert_eq!(canonical.args, delta);
    }

    #[test]
    fn named_xyz_delta_fills_missing_components_with_zero() {
        let delta = vec![Arg::named("x", length(-10.0)), Arg::named("z", length(5.0))];
        let canonical = canonicalize_delta(&Type::Position, &delta).unwrap();
        assert_eq!(
            canonical.args,
            vec![Arg::positional(Expr::Vector3([
                Box::new(length(-10.0)),
                Box::new(zero_length()),
                Box::new(length(5.0)),
            ]))]
        );
    }

    #[test]
    fn named_joint_delta_preserves_dimension() {
        let delta = vec![Arg::named("j2", angle(5.0))];
        let canonical = canonicalize_delta(&Type::Joints { dimension: Some(3) }, &delta).unwrap();
        assert_eq!(canonical.shape, DeltaShape::Scalars);
        assert_eq!(
            canonical.args,
            vec![
                Arg::positional(zero_angle()),
                Arg::positional(angle(5.0)),
                Arg::positional(zero_angle()),
            ]
        );
    }

    #[test]
    fn positional_joint_delta_requires_full_arity() {
        let delta = vec![Arg::positional(angle(1.0))];
        let error = canonicalize_delta(&Type::Joints { dimension: Some(3) }, &delta).unwrap_err();
        assert!(error.contains("expected 3 joint deltas, got 1"));
    }

    #[test]
    fn mixing_named_and_positional_is_rejected() {
        let delta = vec![Arg::positional(length(1.0)), Arg::named("x", length(2.0))];
        let error = canonicalize_delta(&Type::Position, &delta).unwrap_err();
        assert!(error.contains("cannot mix named and positional"));
    }

    #[test]
    fn unknown_component_is_rejected() {
        let delta = vec![Arg::named("j2", angle(5.0))];
        let error = canonicalize_delta(&Type::Position, &delta).unwrap_err();
        assert!(error.contains("Unknown offset component 'j2'"));
    }
}
