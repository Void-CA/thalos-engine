use super::cylindrical_rpp::{self, CylindricalRPPSpec};
use super::error::RobotModelError;
use super::manipulator_3dof::{self, Manipulator3DOFSpec};
use super::manipulator_6dof::{self, Manipulator6DOFSpec};
use super::metadata::RobotMetadata;
use super::planar_2r::{self, Planar2RSpec};
use super::planar_3r::{self, Planar3RSpec};
use super::scara::{self, ScaraSpec};
use super::single_revolute::{self, SingleRevoluteSpec};
use super::spherical_polar_rrp::{self, SphericalPolarRRPSpec};
use crate::robot::serial_chain::SerialChain;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RobotModel {
    Planar2R,
    Planar3R,
    SingleRevolute,
    Scara,
    Manipulator3DOF,
    Manipulator6DOF,
    CylindricalRPP,
    SphericalPolarRRP,
}

impl RobotModel {
    pub fn metadata(&self) -> RobotMetadata {
        match self {
            RobotModel::Planar2R => RobotMetadata {
                id: "planar_2r",
                display_name: "Planar 2R",
                dof: 2,
                joints: planar_2r::JOINTS_PLANAR_2R,
            },
            RobotModel::Planar3R => RobotMetadata {
                id: "planar_3r",
                display_name: "Planar 3R",
                dof: 3,
                joints: planar_3r::JOINTS_PLANAR_3R,
            },
            RobotModel::SingleRevolute => RobotMetadata {
                id: "single_revolute",
                display_name: "Single Revolute",
                dof: 1,
                joints: single_revolute::JOINTS_SINGLE_REVOLUTE,
            },
            RobotModel::Scara => RobotMetadata {
                id: "scara",
                display_name: "SCARA",
                dof: 4,
                joints: scara::JOINTS_SCARA,
            },
            RobotModel::Manipulator3DOF => RobotMetadata {
                id: "manipulator_3dof",
                display_name: "Manipulator 3DOF",
                dof: 3,
                joints: manipulator_3dof::JOINTS_MANIPULATOR_3DOF,
            },
            RobotModel::Manipulator6DOF => RobotMetadata {
                id: "manipulator_6dof",
                display_name: "Manipulator 6DOF",
                dof: 6,
                joints: manipulator_6dof::JOINTS_MANIPULATOR_6DOF,
            },
            RobotModel::CylindricalRPP => RobotMetadata {
                id: "cylindrical_rpp",
                display_name: "Cylindrical RPP",
                dof: 3,
                joints: cylindrical_rpp::JOINTS_CYLINDRICAL_RPP,
            },
            RobotModel::SphericalPolarRRP => RobotMetadata {
                id: "spherical_polar_rrp",
                display_name: "Spherical-Polar RRP",
                dof: 3,
                joints: spherical_polar_rrp::JOINTS_SPHERICAL_POLAR_RRP,
            },
        }
    }

    pub fn default_spec(&self) -> RobotSpec {
        match self {
            RobotModel::Planar2R => RobotSpec::Planar2R(planar_2r::DEFAULT_SPEC),
            RobotModel::Planar3R => RobotSpec::Planar3R(planar_3r::DEFAULT_SPEC),
            RobotModel::SingleRevolute => RobotSpec::SingleRevolute(single_revolute::DEFAULT_SPEC),
            RobotModel::Scara => RobotSpec::Scara(scara::DEFAULT_SPEC),
            RobotModel::Manipulator3DOF => {
                RobotSpec::Manipulator3DOF(manipulator_3dof::DEFAULT_SPEC)
            }
            RobotModel::Manipulator6DOF => {
                RobotSpec::Manipulator6DOF(manipulator_6dof::DEFAULT_SPEC)
            }
            RobotModel::CylindricalRPP => RobotSpec::CylindricalRPP(cylindrical_rpp::DEFAULT_SPEC),
            RobotModel::SphericalPolarRRP => {
                RobotSpec::SphericalPolarRRP(spherical_polar_rrp::DEFAULT_SPEC)
            }
        }
    }

    pub fn from_id(id: &str) -> Result<RobotModel, RobotModelError> {
        match id {
            "planar_2r" => Ok(RobotModel::Planar2R),
            "planar_3r" => Ok(RobotModel::Planar3R),
            "single_revolute" => Ok(RobotModel::SingleRevolute),
            "scara" => Ok(RobotModel::Scara),
            "manipulator_3dof" => Ok(RobotModel::Manipulator3DOF),
            "manipulator_6dof" => Ok(RobotModel::Manipulator6DOF),
            "cylindrical_rpp" => Ok(RobotModel::CylindricalRPP),
            "spherical_polar_rrp" => Ok(RobotModel::SphericalPolarRRP),
            _ => Err(RobotModelError::InvalidRobotId { id: id.to_string() }),
        }
    }

    pub fn all() -> &'static [RobotModel] {
        &[
            RobotModel::Planar2R,
            RobotModel::Planar3R,
            RobotModel::SingleRevolute,
            RobotModel::Scara,
            RobotModel::Manipulator3DOF,
            RobotModel::Manipulator6DOF,
            RobotModel::CylindricalRPP,
            RobotModel::SphericalPolarRRP,
        ]
    }
}

/// Spec de geometría de un robot. Cada robot define su propio struct nominal
/// (ver `models/<robot>/spec.rs`); este enum es un wrapper discriminante que
/// permite despachar entre tipos heterogéneos preservando la API pública.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RobotSpec {
    Planar2R(Planar2RSpec),
    Planar3R(Planar3RSpec),
    SingleRevolute(SingleRevoluteSpec),
    Scara(ScaraSpec),
    Manipulator3DOF(Manipulator3DOFSpec),
    Manipulator6DOF(Manipulator6DOFSpec),
    CylindricalRPP(CylindricalRPPSpec),
    SphericalPolarRRP(SphericalPolarRRPSpec),
}

pub struct RobotRegistry;

impl RobotRegistry {
    /// Construye un robot validando consistencia model↔spec.
    pub fn create(model: RobotModel, spec: RobotSpec) -> Result<SerialChain, RobotModelError> {
        match (&model, &spec) {
            (RobotModel::Planar2R, RobotSpec::Planar2R(s)) => Ok(s.build()),
            (RobotModel::Planar3R, RobotSpec::Planar3R(s)) => Ok(s.build()),
            (RobotModel::SingleRevolute, RobotSpec::SingleRevolute(s)) => Ok(s.build()),
            (RobotModel::Scara, RobotSpec::Scara(s)) => Ok(s.build()),
            (RobotModel::Manipulator3DOF, RobotSpec::Manipulator3DOF(s)) => Ok(s.build()),
            (RobotModel::CylindricalRPP, RobotSpec::CylindricalRPP(s)) => Ok(s.build()),
            (RobotModel::SphericalPolarRRP, RobotSpec::SphericalPolarRRP(s)) => Ok(s.build()),
            // 6DOF factory todavía no implementado: caemos al mismatch.
            _ => Err(RobotModelError::ModelSpecMismatch { model, spec }),
        }
    }

    /// Construye un robot con parámetros por defecto para el modelo dado.
    pub fn create_default(model: RobotModel) -> SerialChain {
        let spec = model.default_spec();
        Self::create(model, spec).unwrap()
    }
}
