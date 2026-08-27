use crate::robot::joint::{JointInfo, JointKind, JointLimits};
use thalos_math::constants::PI;

/// Spec de un robot planar 2R.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Planar2RSpec {
    pub l1: f64,
    pub l2: f64,
    pub joint_limits: [JointLimits; 2],
}

impl Planar2RSpec {
    pub const fn new(l1: f64, l2: f64, joint_limits: [JointLimits; 2]) -> Self {
        Self {
            l1,
            l2,
            joint_limits,
        }
    }

    /// Robot ideal: dos revolutos con rango completo.
    pub const fn ideal() -> Self {
        Self {
            l1: 1.0,
            l2: 1.0,
            joint_limits: [JointLimits::new(-PI, PI), JointLimits::new(-PI, PI)],
        }
    }

    pub fn build(&self) -> crate::robot::serial_chain::SerialChain {
        let [jl1, jl2] = self.joint_limits;
        super::factory::create_planar_2r(self.l1, self.l2, jl1, jl2)
    }

    pub const fn joints(&self) -> [JointInfo; 2] {
        let [j1, j2] = self.joint_limits;
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
        ]
    }
}

pub const DEFAULT: Planar2RSpec = Planar2RSpec::ideal();

pub static JOINTS: &[JointInfo] = {
    const S: Planar2RSpec = Planar2RSpec::ideal();
    const J: [JointInfo; 2] = S.joints();
    &J
};
