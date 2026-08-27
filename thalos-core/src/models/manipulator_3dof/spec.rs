use crate::robot::joint::{JointInfo, JointKind, JointLimits};
use thalos_math::constants::PI;

/// Spec de un manipulador 3DOF estilo PUMA-base (columna vertical).
///
/// ADR-0001 (Z-up): joint 1 (yaw, eje Z vertical), joint 2 (hombro, eje Y
/// horizontal), joint 3 (codo, eje Y, paralelo a joint 2). Los links se
/// extienden en +X local.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Manipulator3DOFSpec {
    pub l1: f64,
    pub l2: f64,
    pub l3: f64,
    pub joint_limits: [JointLimits; 3],
}

impl Manipulator3DOFSpec {
    pub const fn new(l1: f64, l2: f64, l3: f64, joint_limits: [JointLimits; 3]) -> Self {
        Self {
            l1,
            l2,
            l3,
            joint_limits,
        }
    }

    /// Robot ideal: tres revolutos con rango completo.
    pub const fn ideal() -> Self {
        Self {
            l1: 1.0,
            l2: 1.0,
            l3: 1.0,
            joint_limits: [
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
            ],
        }
    }

    pub fn build(&self) -> crate::robot::serial_chain::SerialChain {
        let [jl1, jl2, jl3] = self.joint_limits;
        super::factory::create_manipulator_3dof(self.l1, self.l2, self.l3, jl1, jl2, jl3)
    }

    /// Z-Y-Y: joint 1 en Z (vertical), joints 2 y 3 en Y (horizontal pitch).
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
                kind: JointKind::Revolute,
                limits: Some(j3),
            },
        ]
    }
}

pub const DEFAULT: Manipulator3DOFSpec = Manipulator3DOFSpec::ideal();

pub static JOINTS: &[JointInfo] = {
    const S: Manipulator3DOFSpec = Manipulator3DOFSpec::ideal();
    const J: [JointInfo; 3] = S.joints();
    &J
};
