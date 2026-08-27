//! Pipeline module for trajectory optimization.
//!
//! Contains the operator selection and iteration logic that drives
//! the optimization process across problem regions.
//!
//! - `operator_selector` — Ranks operators by composite score for a given region
//! - `optimization_pipeline` — Iterative per-region optimization loop
//! - `acceptance` — Evaluates operator candidates before accepting
//! - `trajectory_composer` — Blends modified segments with original trajectory

pub mod acceptance;
pub mod operator_selector;
pub mod optimization_pipeline;
pub mod trajectory_composer;

pub use acceptance::{AcceptanceEvaluation, AcceptancePolicy};
pub use operator_selector::OperatorSelector;
pub use optimization_pipeline::{OptimizationPipeline, OptimizationResult};
pub use trajectory_composer::{BlendPolicy, compose_trajectory};
