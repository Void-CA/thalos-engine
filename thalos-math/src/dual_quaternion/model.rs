use crate::{MathError, Quaternion, Transform3D, UnitQuaternion, UnitVector3, Vector3};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, Mul, Sub};

/// Spatial velocity twist ξ = (ω, v) representing angular and linear velocity.
/// Used in screw-based differential kinematics.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    pub angular: Vector3,
    pub linear: Vector3,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DualNumber {
    pub real: f64,
    pub dual: f64,
}

impl DualNumber {
    pub fn new(real: f64, dual: f64) -> Self {
        Self { real, dual }
    }
}

impl Mul for DualNumber {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self {
            real: self.real * rhs.real,
            dual: self.real * rhs.dual + self.dual * rhs.real,
        }
    }
}

impl Add for DualNumber {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            real: self.real + rhs.real,
            dual: self.dual + rhs.dual,
        }
    }
}

impl Mul<f64> for DualNumber {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self {
            real: self.real * rhs,
            dual: self.dual * rhs,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DualQuaternion {
    q_r: Quaternion,
    q_d: Quaternion,
}

impl DualQuaternion {
    pub fn new(q_r: Quaternion, q_d: Quaternion) -> Result<Self, MathError> {
        if !q_r.is_unit() {
            return Err(MathError::QuaternionNotUnit {
                norm_sq: q_r.norm_squared(),
            });
        }
        Ok(Self { q_r, q_d })
    }

    pub fn identity() -> Self {
        Self {
            q_r: Quaternion::identity(),
            q_d: Quaternion::new(0.0, 0.0, 0.0, 0.0),
        }
    }

    pub fn from_rotation_translation(rotation: UnitQuaternion, translation: Vector3) -> Self {
        let q_r = rotation.into_inner();
        let q_t = Quaternion::new(0.0, translation.x, translation.y, translation.z) * q_r;
        let q_d = Quaternion::new(q_t.w * 0.5, q_t.x * 0.5, q_t.y * 0.5, q_t.z * 0.5);
        Self { q_r, q_d }
    }

    pub fn from_axis_angle_translation(
        axis: UnitVector3,
        angle: f64,
        translation: Vector3,
    ) -> Self {
        let rotation = UnitQuaternion::from_axis_angle(axis, angle);
        Self::from_rotation_translation(rotation, translation)
    }

    pub fn rotation(&self) -> Quaternion {
        self.q_r
    }

    pub fn dual_part(&self) -> Quaternion {
        self.q_d
    }

    pub fn translation(&self) -> Vector3 {
        let t = self.q_d * self.q_r.conjugate();
        Vector3::new(t.x * 2.0, t.y * 2.0, t.z * 2.0)
    }

    pub fn conjugate(&self) -> Self {
        Self {
            q_r: self.q_r.conjugate(),
            q_d: Quaternion::new(-self.q_d.w, -self.q_d.x, -self.q_d.y, -self.q_d.z),
        }
    }

    pub fn dual_conjugate(&self) -> Self {
        Self {
            q_r: self.q_r,
            q_d: Quaternion::new(-self.q_d.w, -self.q_d.x, -self.q_d.y, -self.q_d.z),
        }
    }

    pub fn norm(&self) -> f64 {
        self.q_r.norm()
    }

    pub fn normalize(&self) -> Self {
        let norm_r = self.q_r.norm();
        if norm_r < crate::constants::EPS {
            return *self;
        }
        let q_r = Quaternion::new(
            self.q_r.w / norm_r,
            self.q_r.x / norm_r,
            self.q_r.y / norm_r,
            self.q_r.z / norm_r,
        );
        let dot = self.q_r.w * self.q_d.w
            + self.q_r.x * self.q_d.x
            + self.q_r.y * self.q_d.y
            + self.q_r.z * self.q_d.z;
        let q_d = Quaternion::new(
            self.q_d.w - dot * self.q_r.w / (norm_r * norm_r),
            self.q_d.x - dot * self.q_r.x / (norm_r * norm_r),
            self.q_d.y - dot * self.q_r.y / (norm_r * norm_r),
            self.q_d.z - dot * self.q_r.z / (norm_r * norm_r),
        );
        Self { q_r, q_d }
    }

    pub fn to_screw_axis(&self) -> (Vector3, Vector3) {
        let omega = Vector3::new(self.q_r.x * 2.0, self.q_r.y * 2.0, self.q_r.z * 2.0);
        let v = Vector3::new(self.q_d.x * 2.0, self.q_d.y * 2.0, self.q_d.z * 2.0);
        (omega, v)
    }

    /// Convert to a Twist (spatial velocity) representation.
    pub fn to_twist(&self) -> Twist {
        let (angular, linear) = self.to_screw_axis();
        Twist { angular, linear }
    }

    pub fn to_transform(&self) -> Transform3D {
        let rotation = UnitQuaternion::new(self.q_r).unwrap_or_else(|_| UnitQuaternion::identity());
        let translation = self.translation();
        Transform3D::from_translation_rotation(translation, rotation)
    }
}

impl Mul for DualQuaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self {
            q_r: self.q_r * rhs.q_r,
            q_d: self.q_r * rhs.q_d + self.q_d * rhs.q_r,
        }
    }
}

impl Add for DualQuaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            q_r: self.q_r + rhs.q_r,
            q_d: self.q_d + rhs.q_d,
        }
    }
}

impl Sub for DualQuaternion {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            q_r: self.q_r - rhs.q_r,
            q_d: self.q_d - rhs.q_d,
        }
    }
}

impl From<DualQuaternion> for Transform3D {
    fn from(dq: DualQuaternion) -> Self {
        dq.to_transform()
    }
}

impl From<Transform3D> for DualQuaternion {
    fn from(t: Transform3D) -> Self {
        Self::from_rotation_translation(t.rotation, t.translation)
    }
}

impl fmt::Display for DualQuaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "DualQuaternion({}, {})", self.q_r, self.q_d)
    }
}
