//! Consumer scenario **B3 — inverse kinematics** — *resolved in slice D4*.
//!
//! # Intent
//! "Solve this target."
//!
//! # What the consumer must know now (after D4)
//! - `intent::{Manipulator, CartesianTarget}` — and, for advanced control,
//!   `intent::IkRequest` (Tier 2).
//!
//! # Mechanism now hidden
//! Before D4 the consumer built the solver and the goal by hand:
//! `ForwardKinematics`, `DampedLeastSquaresSolver`, `IKConfig`, `IKGoal`, the
//! `IKSolver` trait, and read `IKResult`/`IKStatus`. None of that is needed.
//!
//! # Gate (D4)
//! This file is **conceptually incapable** of naming `IKGoal`, `SerialChain`,
//! `ForwardKinematics`, `DampedLeastSquaresSolver`, `IKConfig`, `IKResult` or
//! `IKSolver`. It says *what* it wants and receives an `IkSolution`.
//!
//! # Tier 2 (still not Tier 3)
//! ```ignore
//! let solution = robot.inverse_with(
//!     IkRequest::new(CartesianTarget::position(1.0, 0.5, 0.0))
//!         .with_seed(vec![0.2, 0.1])
//!         .with_tolerance(1e-8),
//! )?;
//! ```

#[path = "common/mod.rs"]
mod common;

use thalos_api::intent::{CartesianTarget, Manipulator};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Manipulator::from_urdf(common::PLANAR_2R_URDF)?;

    let solution = robot.inverse(CartesianTarget::position(1.0, 0.5, 0.0))?;

    println!(
        "IK converged={} joints={:?} ({} iterations, error {:.2e})",
        solution.converged(),
        solution.joints(),
        solution.iterations(),
        solution.final_error()
    );
    Ok(())
}
