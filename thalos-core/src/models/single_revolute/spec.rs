use crate::robot::joint::{JointInfo, JointKind, JointLimits};
use thalos_math::constants::PI;

/// Spec de un robot SingleRevolute.
///
/// `l` es la longitud del único link (extendido en +X local del joint).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SingleRevoluteSpec {
    pub l: f64,
    pub joint_limits: [JointLimits; 1],
}

impl SingleRevoluteSpec {
    pub const fn new(l: f64, joint_limits: [JointLimits; 1]) -> Self {
        Self { l, joint_limits }
    }

    /// Robot ideal: un revolute con rango completo.
    pub const fn ideal() -> Self {
        Self {
            l: 1.0,
            joint_limits: [JointLimits::new(-PI, PI)],
        }
    }

    pub fn build(&self) -> crate::robot::serial_chain::SerialChain {
        let [jl1] = self.joint_limits;
        super::factory::create_single_revolute(self.l, jl1)
    }

    pub const fn joints(&self) -> [JointInfo; 1] {
        let [j1] = self.joint_limits;
        [JointInfo {
            name: "joint_1",
            kind: JointKind::Revolute,
            limits: Some(j1),
        }]
    }
}

pub const DEFAULT: SingleRevoluteSpec = SingleRevoluteSpec::ideal();

pub static JOINTS: &[JointInfo] = {
    const S: SingleRevoluteSpec = SingleRevoluteSpec::ideal();
    const J: [JointInfo; 1] = S.joints();
    &J
};
