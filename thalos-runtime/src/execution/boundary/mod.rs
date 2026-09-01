//! Execution Validation Boundary — plan→hardware manifest and trace types.
//!
//! This module owns the types shared with the ESP32 hardware backend. The
//! plan-based adapter (`prepare`/`execute`) and manifest builder (`from_plan`)
//! were removed with the parallel plan type (invariant I4 — the type no
//! longer exists); the canonical trajectory output is `CompiledPlan` (IR-3).
//!
//! # Ownership
//!
//! Lives in `thalos-runtime` because runtime already depends on `thalos-planning`
//! and owns `RobotController`. The reverse direction would create a circular dep.
//!
//! # Modules
//!
//! | Module | Contents | Phase |
//! |--------|----------|-------|
//! | `manifest` | `ExecutionManifest`, `ManifestMetadata`, `ManifestSegment`, `ManifestInstruction`, `TimedWaypoint` | PR 1 |
//! | `sample` | `ExecutionSample` | PR 1 |
//! | `manifest_builder` | `ExecutionManifestBuilder` — pure `ExecutionPlan → ExecutionManifest` builder + firmware-parity validator | PR 2; consumed by the deprecated `Esp32Backend::build_manifest` shim since PR 3 |

pub mod manifest;
pub mod manifest_builder;
pub mod safety_envelope;
pub mod sample;
pub mod velocity_retimer;

pub use manifest::{
    ExecutionManifest, ManifestInstruction, ManifestMetadata, ManifestSegment, TimedWaypoint,
};
pub use sample::ExecutionSample;
