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

/// Curated core domain surface.
///
/// Only the fundamental domain modules are public here. Internal/stepping-stone
/// modules (`analysis`, `models` catalog, and the legacy ADR-014 inventory types)
/// are exposed through their own namespaces below or kept private.
pub mod core {
    pub use thalos_core::{
        collision, command, device, deviation, ids, kinematics, motion, operation, program, robot,
        skill, spatial, trajectory,
    };
}

/// Analysis layer (workspace/singularity/manipulability analyzers and the
/// canonical observation language). Initially re-exported from `thalos_core`;
/// the analyzer services are being extracted into `thalos-analysis`.
pub mod analysis {
    pub use thalos_core::analysis::*;
}

/// Robot models: structural/URDF types plus the built-in kinematic model catalog.
pub mod models {
    pub use thalos_models::*;

    /// Built-in kinematic model catalog (`RobotModel`, `RobotRegistry`, …).
    pub mod catalog {
        pub use thalos_core::models::*;
    }
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
