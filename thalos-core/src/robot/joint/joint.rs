pub use thalos_models::JointLimits;

use crate::robot::joint::{
    fixed::FixedJoint, kind::JointKind, prismatic::PrismaticJoint, revolute::RevoluteJoint,
};
use thalos_math::Transform3D;
use thalos_math::UnitVector3;

pub use thalos_models::JointId;

#[derive(Debug, Clone)]
pub enum JointType {
    Revolute(RevoluteJoint),
    Prismatic(PrismaticJoint),
    Fixed(FixedJoint),
}

impl JointType {
    /// Número de grados de libertad que aporta este joint.
    ///
    /// - `Revolute`, `Prismatic` → 1
    /// - `Fixed` → 0
    pub fn dof(&self) -> usize {
        match self {
            JointType::Revolute(_) | JointType::Prismatic(_) => 1,
            JointType::Fixed(_) => 0,
        }
    }

    pub fn limits(&self) -> JointLimits {
        match self {
            JointType::Revolute(rev) => rev.limits,
            JointType::Prismatic(pris) => pris.distance_limits,
            JointType::Fixed(_) => JointLimits::new(0.0, 0.0),
        }
    }

    pub fn id(&self) -> JointId {
        match self {
            JointType::Revolute(rev) => rev.id,
            JointType::Prismatic(pris) => pris.id,
            JointType::Fixed(_) => 0,
        }
    }

    pub fn motion(&self, q: f64) -> Transform3D {
        match self {
            JointType::Revolute(j) => j.motion(q),
            JointType::Prismatic(j) => j.motion(q),
            JointType::Fixed(j) => j.motion(q),
        }
    }

    pub fn origin(&self) -> &Transform3D {
        match self {
            JointType::Revolute(j) => &j.origin,
            JointType::Prismatic(j) => &j.origin,
            JointType::Fixed(j) => &j.origin,
        }
    }

    pub fn axis(&self) -> UnitVector3 {
        match self {
            JointType::Revolute(j) => j.axis,
            JointType::Prismatic(j) => j.direction,
            JointType::Fixed(_) => UnitVector3::z_axis(),
        }
    }

    pub fn kind(&self) -> JointKind {
        match self {
            JointType::Revolute(_) => JointKind::Revolute,
            JointType::Prismatic(_) => JointKind::Prismatic,
            JointType::Fixed(_) => JointKind::Fixed,
        }
    }

    pub fn axis_world(&self, transform: &Transform3D) -> UnitVector3 {
        let axis_local = self.axis();

        let rotated = transform.rotation.rotate_vector(axis_local.into_inner());

        UnitVector3::new(rotated).unwrap()
    }
}
