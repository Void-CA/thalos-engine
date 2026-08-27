//! Thalos Engine — unified robotics domain facade crate.
//!
//! `thalos-engine` provides a single, cohesive facade over the Thalos Engine crate ecosystem.
//! It re-exports all domain sub-crates as named modules and provides a curated [`prelude`] for ergonomic
//! access to the primary math, kinematics, robot model, planning, and semantic primitives.
//!
//! # Architecture
//!
//! The Engine is structured into eight domain crates:
//!
//! - [`math`]: Linear algebra, 3D transforms, quaternions, and math constants.
//! - [`models`]: Data-only robot specifications and URDF structures.
//! - [`core`]: Kinematics, trajectory representation, and spatial frame graphs.
//! - [`collision`]: Collision detection algorithms and distance queries.
//! - [`planning`]: Motion planning algorithms, instruction compilation, and trajectory materialization.
//! - [`optimization`]: Null-space, orientation relaxation, and sampling trajectory operators.
//! - [`semantic`]: Task-level semantic programs, operations, and lowering logic.
//! - [`intelligence`]: High-level decision making and reasoning components.
//!
//! # Quick Start
//!
//! ```rust
//! use thalos_engine::prelude::*;
//!
//! // Create 3D spatial transforms and vectors
//! let translation = Vector3::new(0.5, 0.0, 0.2);
//! let transform = Transform3D::from_translation(translation);
//! let pose = Pose::new(FrameId::World, FrameId::World, transform);
//!
//! assert_eq!(thalos_engine::version(), "0.1.0");
//! ```

pub use thalos_collision as collision;
pub use thalos_core as core;
pub use thalos_intelligence as intelligence;
pub use thalos_math as math;
pub use thalos_models as models;
pub use thalos_optimization as optimization;
pub use thalos_planning as planning;
pub use thalos_semantic as semantic;

/// Convenient re-exports of foundational types, math primitives, and traits across Thalos Engine.
pub mod prelude {
    // ── Math & Geometry Primitives ──
    pub use thalos_math::constants::*;
    pub use thalos_math::*;

    // ── Core Domain Primitives ──
    pub use thalos_core::prelude::*;

    // ── Robot Models & Specs ──
    pub use thalos_models::{Robot, RobotGraph};

    // ── Motion Planning & Compilers ──
    pub use thalos_planning::motion::compiler::PlanCompiler;

    // ── Semantic Task Model ──
    pub use thalos_semantic::operation::SemanticOperation;
    pub use thalos_semantic::program::SemanticProgram;
}

/// Returns the Engine version, inherited from the workspace package.
pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_compiles() {
        assert!(!version().is_empty());
    }

    #[test]
    fn reports_workspace_version() {
        assert_eq!(version(), "0.1.0");
    }

    #[test]
    fn prelude_reexports_fundamental_types() {
        use super::prelude::*;

        // Math
        let v = Vector3::new(1.0, 0.0, 0.0);
        let t = Transform3D::from_translation(v);
        assert!((t.translation.x - 1.0).abs() < 1e-9);

        // Core
        let pose = Pose::new(FrameId::World, FrameId::World, Transform3D::identity());
        assert_eq!(pose.reference_id(), FrameId::World);

        // Semantic
        let prog = SemanticProgram::new(vec![]);
        assert!(prog.operations.is_empty());
    }
}
