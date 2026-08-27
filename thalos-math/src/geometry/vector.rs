use crate::{
    MathError, constants,
    traits::{Cross, Dot},
};
use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn norm(&self) -> f64 {
        self.magnitude()
    }

    pub fn normalized(&self) -> Result<Self, MathError> {
        let mag = self.magnitude();
        if mag.abs() < constants::EPS {
            return Err(MathError::ZeroVectorNormalization);
        }
        Ok(Self {
            x: self.x / mag,
            y: self.y / mag,
            z: self.z / mag,
        })
    }

    pub fn z_axis() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        }
    }

    pub fn y_axis() -> Self {
        Self {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        }
    }

    pub fn x_axis() -> Self {
        Self {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

// ─── Operator impls ─────────────────────────────────────────────

impl Dot for Vector3 {
    type Output = f64;

    fn dot(self, rhs: Vector3) -> Self::Output {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl Cross for Vector3 {
    type Output = Vector3;

    fn cross(self, rhs: Vector3) -> Self::Output {
        Vector3 {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }
}

impl Mul<f64> for Vector3 {
    type Output = Vector3;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl std::fmt::Display for Vector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(f, "(x: {:.4}, y: {:.4}, z: {:.4})", self.x, self.y, self.z);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::EPS;

    #[test]
    fn zero_vector_normalization_returns_error() {
        let v = Vector3::zero();
        let result = v.normalized();
        assert!(result.is_err(), "normalize of zero should error");
    }

    #[test]
    fn normalize_non_zero_vector() {
        let v = Vector3::new(3.0, 0.0, 0.0);
        let n = v.normalized().unwrap();
        assert!((n.x - 1.0).abs() < EPS);
        assert!((n.y - 0.0).abs() < EPS);
        assert!((n.z - 0.0).abs() < EPS);
    }

    #[test]
    fn dot_product_orthogonal() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        assert!(v1.dot(v2).abs() < EPS);
    }

    #[test]
    fn cross_product_orthogonal() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let c = v1.cross(v2);
        assert!((c.x - 0.0).abs() < EPS);
        assert!((c.y - 0.0).abs() < EPS);
        assert!((c.z - 1.0).abs() < EPS);
    }

    #[test]
    fn add_vectors() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        let r = v1 + v2;
        assert!((r.x - 5.0).abs() < EPS);
        assert!((r.y - 7.0).abs() < EPS);
        assert!((r.z - 9.0).abs() < EPS);
    }

    #[test]
    fn sub_vectors() {
        let v1 = Vector3::new(4.0, 5.0, 6.0);
        let v2 = Vector3::new(1.0, 2.0, 3.0);
        let r = v1 - v2;
        assert!((r.x - 3.0).abs() < EPS);
        assert!((r.y - 3.0).abs() < EPS);
        assert!((r.z - 3.0).abs() < EPS);
    }

    #[test]
    fn scale_vector() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let r = v * 2.0;
        assert!((r.x - 2.0).abs() < EPS);
        assert!((r.y - 4.0).abs() < EPS);
        assert!((r.z - 6.0).abs() < EPS);
    }

    #[test]
    fn axis_functions() {
        let x = Vector3::x_axis();
        assert!((x.x - 1.0).abs() < EPS);
        assert!((x.y - 0.0).abs() < EPS);
        assert!((x.z - 0.0).abs() < EPS);

        let y = Vector3::y_axis();
        assert!((y.x - 0.0).abs() < EPS);
        assert!((y.y - 1.0).abs() < EPS);
        assert!((y.z - 0.0).abs() < EPS);

        let z = Vector3::z_axis();
        assert!((z.x - 0.0).abs() < EPS);
        assert!((z.y - 0.0).abs() < EPS);
        assert!((z.z - 1.0).abs() < EPS);
    }
}
