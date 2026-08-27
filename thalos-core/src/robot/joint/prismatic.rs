use crate::robot::joint::joint::{JointId, JointLimits};
use thalos_math::{Transform3D, UnitVector3};

#[derive(Debug, Clone)]
pub struct PrismaticJoint {
    pub id: JointId,
    pub direction: UnitVector3,
    pub distance_limits: JointLimits,
    pub origin: Transform3D,
}

impl PrismaticJoint {
    pub fn new(
        id: JointId,
        direction: UnitVector3,
        distance_limits: JointLimits,
        origin: Transform3D,
    ) -> Self {
        Self {
            id,
            direction,
            distance_limits,
            origin,
        }
    }

    pub fn motion(&self, q: f64) -> Transform3D {
        Transform3D::from_translation(self.direction.into_inner() * q)
    }
}
