use crate::robot::joint::{JointInfo, JointKind, JointLimits};
use thalos_math::constants::PI;

/// Spec de un robot cilíndrico RPP (Revolute + Prismatic + Prismatic).
///
/// - `l1`: altura fija de la base (offset en +Z desde el mundo al primer joint).
///
/// Cinemática directa:
///     p = ( r·cosθ, r·sinθ, l1 + z )
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylindricalRPPSpec {
    pub l1: f64,
    pub joint_limits: [JointLimits; 3],
}

impl CylindricalRPPSpec {
    pub const fn new(l1: f64, joint_limits: [JointLimits; 3]) -> Self {
        Self { l1, joint_limits }
    }

    /// Robot ideal: revolute libre, prismáticos simétricos.
    pub const fn ideal() -> Self {
        Self {
            l1: 0.0,
            joint_limits: [
                JointLimits::new(-PI, PI),
                JointLimits::new(-1.0, 1.0),
                JointLimits::new(-1.0, 1.0),
            ],
        }
    }

    pub fn build(&self) -> crate::robot::serial_chain::SerialChain {
        let [jl1, jl2, jl3] = self.joint_limits;
        super::factory::create_cylindrical_rpp(self.l1, jl1, jl2, jl3)
    }

    /// R(z) – P(z) – P(x).
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
                kind: JointKind::Prismatic,
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

pub const DEFAULT: CylindricalRPPSpec = CylindricalRPPSpec::ideal();

pub static JOINTS: &[JointInfo] = {
    const S: CylindricalRPPSpec = CylindricalRPPSpec::ideal();
    const J: [JointInfo; 3] = S.joints();
    &J
};
