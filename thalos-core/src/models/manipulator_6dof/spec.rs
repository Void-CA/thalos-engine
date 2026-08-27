use crate::robot::joint::{JointInfo, JointKind, JointLimits};
use thalos_math::constants::PI;

/// Spec de un manipulador 6DOF (estilo PUMA / UR-like).
///
/// l1..l6 son longitudes de links. La convención cinemática concreta
/// (qué eje corresponde a cada joint) queda definida por `factory.rs`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Manipulator6DOFSpec {
    pub l1: f64,
    pub l2: f64,
    pub l3: f64,
    pub l4: f64,
    pub l5: f64,
    pub l6: f64,
    pub joint_limits: [JointLimits; 6],
}

impl Manipulator6DOFSpec {
    pub const fn new(
        l1: f64,
        l2: f64,
        l3: f64,
        l4: f64,
        l5: f64,
        l6: f64,
        joint_limits: [JointLimits; 6],
    ) -> Self {
        Self {
            l1,
            l2,
            l3,
            l4,
            l5,
            l6,
            joint_limits,
        }
    }

    /// Robot ideal: seis revolutos con rango completo.
    pub const fn ideal() -> Self {
        Self {
            l1: 1.0,
            l2: 1.0,
            l3: 1.0,
            l4: 1.0,
            l5: 1.0,
            l6: 1.0,
            joint_limits: [
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
            ],
        }
    }

    // build() no implementado — factory vacío.
    pub const fn joints(&self) -> [JointInfo; 6] {
        let [j1, j2, j3, j4, j5, j6] = self.joint_limits;
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
            JointInfo {
                name: "joint_4",
                kind: JointKind::Revolute,
                limits: Some(j4),
            },
            JointInfo {
                name: "joint_5",
                kind: JointKind::Revolute,
                limits: Some(j5),
            },
            JointInfo {
                name: "joint_6",
                kind: JointKind::Revolute,
                limits: Some(j6),
            },
        ]
    }
}

pub const DEFAULT: Manipulator6DOFSpec = Manipulator6DOFSpec::ideal();

pub static JOINTS: &[JointInfo] = {
    const S: Manipulator6DOFSpec = Manipulator6DOFSpec::ideal();
    const J: [JointInfo; 6] = S.joints();
    &J
};
