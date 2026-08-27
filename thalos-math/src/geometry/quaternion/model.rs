use crate::{MathError, constants};
use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn norm_squared(&self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }

    pub fn is_unit(&self) -> bool {
        (self.norm_squared() - 1.0).abs() < constants::EPS
    }

    pub fn normalize(&self) -> Result<Self, MathError> {
        let norm = self.norm();

        if norm < constants::EPS {
            return Err(MathError::ZeroQuaternionNormalization);
        }

        Ok(Self {
            w: self.w / norm,
            x: self.x / norm,
            y: self.y / norm,
            z: self.z / norm,
        })
    }

    pub fn normalize_or_identity(&self) -> Self {
        self.normalize().unwrap_or_else(|_| Self::identity())
    }

    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn inverse(&self) -> Result<Self, MathError> {
        let norm_sq = self.norm_squared();

        if norm_sq < constants::EPS {
            return Err(MathError::ZeroQuaternionInverse { norm_sq });
        }

        let c = self.conjugate();

        Ok(Self {
            w: c.w / norm_sq,
            x: c.x / norm_sq,
            y: c.y / norm_sq,
            z: c.z / norm_sq,
        })
    }

    pub fn inverse_or_identity(&self) -> Self {
        self.inverse().unwrap_or_else(|_| Self::identity())
    }
}

// ─── Operator impls ─────────────────────────────────────────────

impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for Quaternion {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl std::fmt::Display for Quaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "({:.4}, {:.4}i, {:.4}j, {:.4}k)",
            self.w, self.x, self.y, self.z
        )
    }
}
