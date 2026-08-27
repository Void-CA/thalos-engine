use std::ops::Mul;

use crate::{UnitQuaternion, Vector3};

impl Mul for UnitQuaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self { q: self.q * rhs.q }
    }
}

impl Mul<Vector3> for UnitQuaternion {
    type Output = Vector3;

    fn mul(self, v: Vector3) -> Vector3 {
        self.rotate_vector(v)
    }
}
