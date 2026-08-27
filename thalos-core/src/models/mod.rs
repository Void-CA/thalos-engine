pub mod factory;
pub mod metadata;

pub mod cylindrical_rpp;
pub mod manipulator_3dof;
pub mod manipulator_6dof;
pub mod planar_2r;
pub mod planar_3r;
pub mod scara;
pub mod single_revolute;
pub mod spherical_polar_rrp;

mod error;

pub use error::RobotModelError;
pub use factory::{RobotModel, RobotRegistry, RobotSpec};
pub use metadata::RobotMetadata;

pub mod factories {
    pub use crate::models::cylindrical_rpp::factory::create_cylindrical_rpp;
    pub use crate::models::manipulator_3dof::factory::create_manipulator_3dof;
    pub use crate::models::planar_2r::factory::create_planar_2r;
    pub use crate::models::planar_3r::factory::create_planar_3r;
    pub use crate::models::scara::factory::create_scara_robot;
    pub use crate::models::single_revolute::factory::create_single_revolute;
    pub use crate::models::spherical_polar_rrp::factory::create_spherical_polar_rrp;
}
