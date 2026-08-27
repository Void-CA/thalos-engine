//! Analysis modules over workspace datasets, trajectory configurations, and
//! the canonical observation language shared across all analyzers.
//!
//! # Layers
//!
//! - **Observation model** ([`attribute_value`], [`location`],
//!   [`observation`]): the machine-readable, artifact-anchored vocabulary every
//!   analyzer emits (see [`crate::analysis::observation`] and `README.md`).
//! - **Report container** ([`report`], [`action`], [`summary`]): the canonical
//!   [`AnalysisReport`] aggregating observations, actions, metrics and a summary,
//!   with structural validation (DAG over `causes[]`, spec I4/I5/I8).
//! - **Aggregation** ([`aggregator`], [`scoring`]): the source-agnostic
//!   `Vec<Observation> → AnalysisReport` step (design D2/D3). The
//!   [`DefaultAggregator`] is the only report constructor and composes a
//!   [`ScoringPolicy`] (penalties → `quality_index` → `grade`, design C2).
//! - **Domain analyzers**: [`singularity`], [`manipulability`], [`workspace`],
//!   [`constraints`], [`region`] — analyzers over the fundamental
//!   [`Workspace`](workspace::Workspace) dataset.
//!
//! Each submodule consumes the fundamental [`Workspace`](workspace::Workspace)
//! dataset and produces a derived analysis (singularity, manipulability, …).

pub mod action;
pub mod aggregator;
pub mod attribute_value;
pub mod constraints;
pub mod location;
pub mod manipulability;
pub mod observation;
pub mod region;
pub mod region_grouper;
pub mod report;
pub mod scoring;
pub mod singularity;
pub mod summary;
pub mod workspace;

pub use action::{Action, ActionId, ActionImpact, ActionKind, ActionPriority};
pub use aggregator::{Aggregator, DefaultAggregator};
pub use attribute_value::AttributeValue;
pub use location::Location;
pub use observation::{ArtifactRef, Observation, ObservationId, ObservationKind, Severity};
pub use region_grouper::{RegionGrouper, RegionGrouperConfig};
pub use report::{AnalysisReport, ReportError};
pub use scoring::{DefaultScoringPolicy, ScoringPolicy};
pub use summary::{AnalysisSummary, Grade};
