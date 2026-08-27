pub mod config;
pub mod error;
pub mod multi_start;
pub mod result;
pub mod seed_generator;
pub mod solver;
pub mod solvers;

pub use config::IKConfig;
pub use error::IkError;
pub use multi_start::MultiStartIKSolver;
pub use result::{IKResult, IKStatus};
pub use seed_generator::{ElbowAlternate, SeedConfig, SeedPolicy};
pub use solver::{IKGoal, IKSolver};
pub use solvers::{DampedLeastSquaresSolver, JacobianTransposeSolver};

// Re-export from kinematics::jacobian for backward compat
pub use crate::kinematics::jacobian::SingularityReport;

#[cfg(test)]
pub mod tests;
