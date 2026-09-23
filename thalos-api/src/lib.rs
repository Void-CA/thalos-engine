//! # thalos-api
//!
//! Public boundary of **Thalos Engine**.
//!
//! This crate is a deliberately thin, *concept-curated* facade. It re-exports the
//! stable public surface of the engine so consumers (e.g. Thalos Industrial) do
//! not depend on the internal crate graph.
//!
//! ## Design rules
//!
//! - The namespace reflects **concepts**, not the internal filesystem layout.
//! - The boundary is **curated**, not an alias of whole crates. Internal modules
//!   that are not ready to be public (e.g. legacy/orphan types) are intentionally
//!   omitted; slimming the internal crates later must not break this surface.
//! - `thalos-api` **must not** depend on `thalos-runtime`: the application layer
//!   depends on the API, never the other way around.
//!
//! ## Target surface
//!
//! The engine owns domain + supervision **semantics** (not its
//! execution mechanism). The following public namespaces are planned:
//!
//! - `execution::supervision` — execution semantics and supervision contracts
//! - `telemetry` — execution evidence and observation semantics (**materialized**, F2.1b)
//! - `robot::controller` — robot controller contract and capabilities
//!
//! The remaining namespaces are intentionally **not materialized** until the
//! corresponding ownership move introduces their first concrete public items.
//! Each is added in the same change that brings the concepts across the boundary,
//! so every public path here has real semantic content behind it.

/// Curated core domain surface.
///
/// Only the fundamental domain modules are public here. Internal/stepping-stone
/// modules (`analysis`, `models` catalog, and the legacy inventory types)
/// are exposed through their own namespaces below or kept private.
///
/// Note: `core::station::Station` (engine, resource-binding) is a different
/// concept from `thalos_runtime::station::Station` (application, operational).
pub mod core {
    pub use thalos_core::{
        capability, collision, command, device, deviation, execution, ids, kinematics, motion,
        operation, program, resource, robot, skill, spatial, station, trajectory,
    };
}

/// Analysis layer: workspace/singularity/manipulability analyzers, the
/// canonical observation language, and the reusable analysis services.
pub mod analysis {
    pub use thalos_core::analysis::*;
    pub use thalos_analysis::{
        AlignedPair, Alignment, ComparePipeline, ComparePipelineError, ComparePipelineOutput,
        ComparisonMetrics, ConditionOutcome, Deviation, JointErrorMetrics, KinematicViolation,
        ManipulabilityService, PlanExecutionComparison, SignalComparison, SignalViolation,
        SingularityService, TickComparison, TickDeviation, TracePoint, WorkspaceService,
        compare_points, compare_tick, detect_deviations,
    };
}

/// Robot models: structural / URDF data types.
pub mod models {
    pub use thalos_models::*;
}

/// Built-in kinematic model catalog (`RobotModel`, `RobotRegistry`, …).
///
/// Distinct from [`models`]: this is the engine's preset kinematic catalog,
/// not the structural robot description.
pub mod catalog {
    pub use thalos_core::models::*;
}

/// Fundamental math types (`Vector3`, `Transform3D`, `Quaternion`, …).
pub mod math {
    pub use thalos_math::*;
}

/// Motion planning and interpolation.
pub mod planning {
    pub use thalos_planning::*;
}

/// Raw Thalos language front-end (AST and parser).
pub mod lang {
    pub use thalos_lang::*;
}

/// Semantic language service (diagnostics, symbols, compilation pipeline).
pub mod semantic {
    pub use thalos_language_service::*;
}

/// URDF / asset importing.
pub mod importer {
    pub use thalos_importer::*;
}

/// IO and transport contracts.
pub mod ports {
    pub use thalos_ports::*;
}

/// Document representations (programs, scenes, resources).
pub mod document {
    pub use thalos_document::*;
}

/// Visual scene representation (frames, links, primitives, trajectories).
pub mod visual {
    pub use thalos_visual::*;
}

/// Execution evidence and observation semantics (traces, samples, analyzer,
/// lifecycle events). Implemented by [`thalos_telemetry`]; the recording
/// mechanism and the live state stay in the application.
pub mod telemetry {
    pub use thalos_telemetry::*;
}
