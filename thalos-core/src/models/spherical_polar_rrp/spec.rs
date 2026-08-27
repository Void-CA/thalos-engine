use crate::robot::joint::{JointInfo, JointKind, JointLimits};
use thalos_math::constants::{PI, PI_2};

/// Spec de un robot esférico-polar RRP (Revolute + Revolute + Prismatic).
///
/// - `l1`: altura fija de la base (offset en +Z desde el mundo al primer joint).
///
/// Cinemática directa:
///     p = ( r·cosφ·cosθ, r·cosφ·sinθ, -r·sinφ )
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphericalPolarRRPSpec {
    pub l1: f64,
    pub joint_limits: [JointLimits; 3],
}

impl SphericalPolarRRPSpec {
    pub const fn new(l1: f64, joint_limits: [JointLimits; 3]) -> Self {
        Self { l1, joint_limits }
    }

    /// Robot ideal: R_z libre, R_y ±π/2, prismático simétrico.
    pub const fn ideal() -> Self {
        Self {
            l1: 0.0,
            joint_limits: [
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI_2, PI_2),
                JointLimits::new(-1.0, 1.0),
            ],
        }
    }

    pub fn build(&self) -> crate::robot::serial_chain::SerialChain {
        let [jl1, jl2, jl3] = self.joint_limits;
        super::factory::create_spherical_polar_rrp(self.l1, jl1, jl2, jl3)
    }

    /// R(z) – R(y) – P(x).
    pub const fn joints(&self) -> [JointInfo; 3] {
        let [j1, j2, j3] = self.joint_limits;
        [
            JointInfo {
                name: "joint_1",
                kind: JointKind::Revolute,
                limits: Some(j1),
            },
            JointInfo {
                name: "joint_2",
                kind: JointKind::Revolute,
                limits: Some(j2),
            },
            JointInfo {
                name: "joint_3",
                kind: JointKind::Prismatic,
                limits: Some(j3),
            },
        ]
    }
}

pub const DEFAULT: SphericalPolarRRPSpec = SphericalPolarRRPSpec::ideal();

pub static JOINTS: &[JointInfo] = {
    const S: SphericalPolarRRPSpec = SphericalPolarRRPSpec::ideal();
    const J: [JointInfo; 3] = S.joints();
    &J
};
