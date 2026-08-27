use crate::robot::joint::joint::{JointId, JointLimits};
use thalos_math::{Transform3D, UnitQuaternion, UnitVector3};
#[derive(Debug, Clone)]
pub struct RevoluteJoint {
    pub id: JointId,
    pub axis: UnitVector3,
    pub limits: JointLimits,
    pub origin: Transform3D,
}

impl RevoluteJoint {
    pub fn new(id: JointId, axis: UnitVector3, limits: JointLimits, origin: Transform3D) -> Self {
        Self {
            id,
            axis,
            limits,
            origin,
        }
    }

    pub fn motion(&self, q: f64) -> Transform3D {
        let rotation = UnitQuaternion::from_axis_angle(self.axis, q);
        Transform3D::from_rotation(rotation)
    }
}
