pub mod interconnection;
pub mod analysis;
pub mod backends;
pub mod commands;
pub mod comparison;
pub mod device;
pub mod error;
pub mod execution;
pub mod motion;
pub mod planning;
pub mod ports;
pub mod resources;
pub mod robot;
pub mod scene;
pub mod semantic;
pub mod services;
pub mod station;
pub mod telemetry;
pub mod test_support;
pub mod workspace;

// ── Application Facade: re-exports of domain & integration crates ──
pub mod engine {
    pub use thalos_engine::*;
}

pub mod document {
    pub use thalos_document::*;
}

pub mod visual {
    pub use thalos_visual::*;
}

pub mod language_service {
    pub use thalos_language_service::*;
}

/// Curated prelude for application-level consumption.
pub mod prelude {
    pub use crate::engine::prelude::*;
    pub use crate::execution::*;
    pub use crate::station::*;
    pub use crate::telemetry::*;
    pub use crate::workspace::*;
    pub use crate::robot::*;
    pub use crate::scene::service::SceneService;
    pub use crate::planning::{PlanningService, AnalysisService};
    pub use crate::semantic::service::SemanticService;
}

pub use execution::{
    analysis as execution_analysis, boundary as execution_boundary, plan, session,
};
pub use motion::{recorder as motion_recorder, trace as motion_trace};
pub use robot as state;
pub use scene as snapshots;

pub use analysis::manipulability::ManipulabilityService;
pub use analysis::singularity::SingularityService;
pub use analysis::workspace::WorkspaceService as WorkspaceAnalysisService;
pub use backends::controller::{BackendCapabilities, RobotController};
pub use commands::dispatch::Command;
pub use error::{ControllerError, RuntimeError};
pub use execution_analysis::ExecutionAnalyzer;
pub use execution_boundary::{
    ExecutionManifest, ManifestInstruction, ManifestMetadata, ManifestSegment, TimedWaypoint,
};
pub use motion_trace::{MotionSample, MotionTrace};
pub use plan::{ActiveMotionPlan, ExecutionSession, MotionType, PlanState, SessionStatus};
pub use planning::{
    AnalysisOutput, AnalysisService, MotionPlanRequest, PlanAnalysisResult, PlanAnalysisService,
    PlanningService,
};
pub use ports::{PersistenceError, ProgramRecord, ProgramRepository, RobotRecord, RobotRepository, RobotSource, WorkspaceRepository};
pub use robot::service::RobotService;
pub use robot::{RobotCatalog, RobotCatalogEntry, RobotCatalogError, RobotCatalogResolution};
pub use workspace::{ActiveWorkspace, OpenedWorkspace, RobotId, Workspace, WorkspaceConfiguration, WorkspaceId, WorkspaceService};
pub use scene::service::SceneService;
pub use scene::snapshot::{RuntimeSnapshot, TickDelta};
pub use semantic::service::{
    CompileMetadata, SemanticCompileOutput, SemanticRunOutput, SemanticService, ValidationSummary,
};
pub use commands::history::{AppliedCommand, CommandHistory, CommandMetrics, DEFAULT_HISTORY_CAP};
pub use session::{ExecutionSource, SessionData, SessionManager, SessionWithTrace};
pub use state::robot_state::{
    CartesianState, ConnectionState, DeviceState, Diagnostics, ExecutionState, JointState,
    MotionMode, MotionState, RobotError, RobotState,
};
pub use telemetry::{
    ExecutionEvent, ExecutionObserver, ExecutionRecorder, ExecutionSample, ExecutionStatistics,
    ExecutionTrace, TraceAnalyzer, TraceMetadata,
};
