use crate::prelude::*;
use thalos_math::constants::*;

/// Spec completa de un robot SCARA.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScaraSpec {
    pub base_height: f64,
    pub a1: f64,
    pub a2: f64,
    /// Límites de los 4 joints actuados:
    /// `[joint_1 (revolute), joint_2 (revolute), joint_3 (prismatic), joint_4 (revolute)]`
    pub joint_limits: [JointLimits; 4],
}

impl ScaraSpec {
    pub const fn new(base_height: f64, a1: f64, a2: f64, joint_limits: [JointLimits; 4]) -> Self {
        Self {
            base_height,
            a1,
            a2,
            joint_limits,
        }
    }

    /// Robot ideal: rangos completos, geometría redonda.
    pub const fn ideal() -> Self {
        Self {
            base_height: 0.0,
            a1: 1.0,
            a2: 1.0,
            joint_limits: [
                JointLimits::new(-PI, PI),
                JointLimits::new(-PI, PI),
                JointLimits::new(-1.0, 1.0),
                JointLimits::new(-PI, PI),
            ],
        }
    }

    /// Robot canónico: parámetros realistas para un SCARA de escritorio.
    pub const fn canonical() -> Self {
        Self {
            base_height: 0.5,
            a1: 1.0,
            a2: 0.8,
            joint_limits: [
                JointLimits::new(-140.0_f64.to_radians(), 140.0_f64.to_radians()),
                JointLimits::new(-150.0_f64.to_radians(), 150.0_f64.to_radians()),
                JointLimits::new(-0.5, 0.0),
                JointLimits::new(-2.0 * PI, 2.0 * PI),
            ],
        }
    }

    /// Construye la `SerialChain` a partir de esta spec.
    pub fn build(&self) -> SerialChain {
        let [jl1, jl2, jl3, jl4] = self.joint_limits;
        super::factory::create_scara_robot(self.base_height, self.a1, self.a2, jl1, jl2, jl3, jl4)
    }

    /// Información de joints.
    pub const fn joints(&self) -> [JointInfo; 4] {
        let [j1, j2, j3, j4] = self.joint_limits;
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
            JointInfo {
                name: "joint_4",
                kind: JointKind::Revolute,
                limits: Some(j4),
            },
        ]
    }
}

/// Spec por defecto: [`ScaraSpec::canonical`].
pub const DEFAULT: ScaraSpec = ScaraSpec::canonical();

/// Joints del SCARA canonical (const para compatibilidad con API).
pub static JOINTS: &[JointInfo] = {
    const CANONICAL: ScaraSpec = ScaraSpec::canonical();
    const ARRAY: [JointInfo; 4] = CANONICAL.joints();
    &ARRAY
};
