use crate::traits::Cross;
use crate::{MathError, Quaternion, UnitVector3, Vector3, constants};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct UnitQuaternion {
    pub q: Quaternion,
}

impl UnitQuaternion {
    pub fn new(q: Quaternion) -> Result<Self, MathError> {
        if !q.is_unit() {
            return Err(MathError::QuaternionNotUnit {
                norm_sq: q.norm_squared(),
            });
        }
        Ok(Self { q })
    }

    pub fn from_quaternion_unchecked(q: Quaternion) -> Self {
        Self { q }
    }

    pub fn inner(&self) -> &Quaternion {
        &self.q
    }

    pub fn into_inner(self) -> Quaternion {
        self.q
    }

    pub fn identity() -> Self {
        Self {
            q: Quaternion::identity(),
        }
    }

    pub fn from_axis_angle(axis: UnitVector3, angle: f64) -> Self {
        let half = angle * 0.5;
        let s = half.sin();
        Self {
            q: Quaternion::new(half.cos(), axis.x * s, axis.y * s, axis.z * s),
        }
    }

    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self::from_euler_angles(roll, pitch, yaw)
    }

    pub fn from_euler_angles(roll: f64, pitch: f64, yaw: f64) -> Self {
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();

        Self {
            q: Quaternion::new(
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ),
        }
    }

    pub fn to_euler(&self) -> (f64, f64, f64) {
        self.to_euler_angles()
    }

    pub fn to_euler_angles(&self) -> (f64, f64, f64) {
        let (w, x, y, z) = (self.q.w, self.q.x, self.q.y, self.q.z);

        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let pitch = (2.0 * (w * y - z * x)).asin().clamp(-1.0, 1.0);
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));

        (roll, pitch, yaw)
    }

    pub fn rotate_vector(&self, v: Vector3) -> Vector3 {
        let q = self.q;
        let v_q = Quaternion::new(0.0, v.x, v.y, v.z);
        let q_inv = Quaternion {
            w: q.w,
            x: -q.x,
            y: -q.y,
            z: -q.z,
        };
        let rotated = q * v_q * q_inv;
        Vector3::new(rotated.x, rotated.y, rotated.z)
    }

    pub fn inverse(&self) -> Self {
        Self {
            q: Quaternion {
                w: self.q.w,
                x: -self.q.x,
                y: -self.q.y,
                z: -self.q.z,
            },
        }
    }

    pub fn rotation_between(a: Vector3, b: Vector3) -> Self {
        let dot = a.x * b.x + a.y * b.y + a.z * b.z;
        let norm_a = a.magnitude();
        let norm_b = b.magnitude();

        if norm_a < constants::EPS || norm_b < constants::EPS {
            return Self::identity();
        }

        let a_n = Vector3::new(a.x / norm_a, a.y / norm_a, a.z / norm_a);
        let b_n = Vector3::new(b.x / norm_b, b.y / norm_b, b.z / norm_b);

        let cos_theta = dot / (norm_a * norm_b);
        let cos_theta = cos_theta.clamp(-1.0, 1.0);

        if (cos_theta - 1.0).abs() < constants::EPS {
            return Self::identity();
        }

        if (cos_theta + 1.0).abs() < constants::EPS {
            // 180-degree rotation — need any perpendicular axis
            let orthogonal = if a_n.x.abs() < a_n.y.abs() {
                Vector3::new(1.0, 0.0, 0.0)
            } else {
                Vector3::new(0.0, 1.0, 0.0)
            };
            let axis_vec = a_n.cross(orthogonal);
            let axis_len = axis_vec.magnitude();
            if axis_len < constants::EPS {
                return Self::identity();
            }
            let axis = UnitVector3::new_normalize(axis_vec);
            return Self::from_axis_angle(axis, std::f64::consts::PI);
        }

        let axis_vec = a_n.cross(b_n);
        let axis_len = axis_vec.magnitude();
        if axis_len < constants::EPS {
            return Self::identity();
        }

        let half = (cos_theta * 0.5 + 0.5).sqrt();
        let s = (1.0 - half * half).sqrt() / axis_len;
        let axis = UnitVector3::new_normalize(axis_vec);

        Self {
            q: Quaternion::new(half, axis.x * s, axis.y * s, axis.z * s),
        }
    }

    pub fn from_rotation_matrix(m: &[[f64; 3]; 3]) -> Self {
        let trace = m[0][0] + m[1][1] + m[2][2];

        if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            Self {
                q: Quaternion::new(
                    s * 0.25,
                    (m[2][1] - m[1][2]) / s,
                    (m[0][2] - m[2][0]) / s,
                    (m[1][0] - m[0][1]) / s,
                ),
            }
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
            Self {
                q: Quaternion::new(
                    (m[2][1] - m[1][2]) / s,
                    s * 0.25,
                    (m[0][1] + m[1][0]) / s,
                    (m[0][2] + m[2][0]) / s,
                ),
            }
        } else if m[1][1] > m[2][2] {
            let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
            Self {
                q: Quaternion::new(
                    (m[0][2] - m[2][0]) / s,
                    (m[0][1] + m[1][0]) / s,
                    s * 0.25,
                    (m[1][2] + m[2][1]) / s,
                ),
            }
        } else {
            let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
            Self {
                q: Quaternion::new(
                    (m[1][0] - m[0][1]) / s,
                    (m[0][2] + m[2][0]) / s,
                    (m[1][2] + m[2][1]) / s,
                    s * 0.25,
                ),
            }
        }
    }

    pub fn to_rotation_matrix(&self) -> [[f64; 3]; 3] {
        let (w, x, y, z) = (self.q.w, self.q.x, self.q.y, self.q.z);
        let w2 = w * w;
        let x2 = x * x;
        let y2 = y * y;
        let z2 = z * z;
        [
            [
                w2 + x2 - y2 - z2,
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                w2 - x2 + y2 - z2,
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                w2 - x2 - y2 + z2,
            ],
        ]
    }

    pub fn slerp(&self, other: &Self, t: f64) -> Self {
        let dot = self.q.w * other.q.w
            + self.q.x * other.q.x
            + self.q.y * other.q.y
            + self.q.z * other.q.z;
        let dot = dot.clamp(-1.0, 1.0);

        if dot > 0.9999 {
            let q = Quaternion::new(
                self.q.w + t * (other.q.w - self.q.w),
                self.q.x + t * (other.q.x - self.q.x),
                self.q.y + t * (other.q.y - self.q.y),
                self.q.z + t * (other.q.z - self.q.z),
            );
            let norm = q.norm();
            return Self {
                q: Quaternion::new(q.w / norm, q.x / norm, q.y / norm, q.z / norm),
            };
        }

        let theta = dot.acos();
        let sin_theta = theta.sin();
        let a = ((1.0 - t) * theta).sin() / sin_theta;
        let b = (t * theta).sin() / sin_theta;

        Self {
            q: Quaternion::new(
                a * self.q.w + b * other.q.w,
                a * self.q.x + b * other.q.x,
                a * self.q.y + b * other.q.y,
                a * self.q.z + b * other.q.z,
            ),
        }
    }

    // ── SO(3) Lie algebra ─────────────────────────────────

    /// Returns the rotation angle in radians [0, π].
    pub fn angle(&self) -> f64 {
        2.0 * self.q.w.clamp(-1.0, 1.0).acos()
    }

    /// Logarithmic map: SO(3) → so(3).
    ///
    /// Returns the rotation vector ω = θ·axis, the so(3) tangent
    /// vector corresponding to this rotation.
    /// Returns the zero vector for the identity quaternion.
    pub fn log(&self) -> Vector3 {
        let w = self.q.w.clamp(-1.0, 1.0);
        let angle = 2.0 * w.acos();
        if angle.abs() < 1e-12 {
            return Vector3::zero();
        }
        let sin_ha = (angle / 2.0).sin();
        let scale = if sin_ha.abs() < 1e-12 {
            // Limit as θ → 0: θ / sin(θ/2) → 2
            2.0
        } else {
            angle / sin_ha
        };
        Vector3::new(self.q.x * scale, self.q.y * scale, self.q.z * scale)
    }

    /// Exponential map: so(3) → SO(3).
    ///
    /// Returns the unit quaternion corresponding to rotation
    /// vector ω = θ·axis. For ‖ω‖ < 1e-12 returns identity.
    pub fn exp_map(v: &Vector3) -> Self {
        let theta = v.norm();
        if theta < 1e-12 {
            return UnitQuaternion::identity();
        }
        let half = theta * 0.5;
        let s = half.sin() / theta;
        Self {
            q: Quaternion::new(half.cos(), v.x * s, v.y * s, v.z * s),
        }
    }
}

impl std::fmt::Display for UnitQuaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "UQ({}, {}, {}, {})",
            self.q.w, self.q.x, self.q.y, self.q.z
        )
    }
}
