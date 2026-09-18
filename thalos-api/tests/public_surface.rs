//! Boundary regression probe.
//!
//! Every path below MUST resolve using only `thalos-api`. This crate has no
//! dev-dependencies on internal engine crates on purpose: if a public concept
//! becomes reachable only through an internal crate, this test stops compiling
//! and the boundary violation is surfaced at the point of change.

#![allow(unused_imports)]

#[test]
fn public_surface_is_self_contained() {
    // Fundamental math.
    use thalos_api::math::{Quaternion, Transform3D, Vector3};

    // Robot structure (URDF-like data) and the built-in kinematic catalog.
    use thalos_api::catalog::{RobotModel, RobotRegistry};
    use thalos_api::models::{Joint, Link, Robot};

    // Core domain primitives and algorithms.
    use thalos_api::core::capability::{CapabilityRequirement, ResourceRequirement};
    use thalos_api::core::command::{Command, CommandSemantics, MotionCommand};
    use thalos_api::core::device::{ChannelId, ChannelObservation, ChannelValue, SignalQuality};
    use thalos_api::core::execution::{
        ExecutionMetadata, ExecutionPlan, ExecutionProgram, ExecutionSegment, ExecutionWaypoint,
        PlanInstruction, ProgramInstruction, RuntimeAction, RuntimeEvent, RuntimeProgram,
    };
    use thalos_api::core::ids::{ExecutionSessionId, OperationId, StationId};
    use thalos_api::core::kinematics::inverse::IKGoal;
    use thalos_api::core::resource::{Resource, ResourceKind, ResourceRef};
    use thalos_api::core::robot::state::{
        JointState, RobotState, StateDeviation, StateRequirement,
    };
    use thalos_api::core::robot::tool_frame::ToolFrame;
    use thalos_api::core::robot::serial_chain::SerialChain;
    use thalos_api::core::spatial::pose::Pose;

    // Analysis, planning and language front-end.
    use thalos_api::analysis::workspace::WorkspaceConfig;
    use thalos_api::analysis::{
        ComparisonMetrics, ManipulabilityService, PlanExecutionComparison, SingularityService,
        TracePoint, WorkspaceService,
    };
    use thalos_api::document::program_document::ProgramDocument;
    use thalos_api::importer::import_urdf;
    use thalos_api::lang::{parse_source, DEFAULT_PROGRAM};
    use thalos_api::planning::execution_plan_builder::ExecutionPlanBuilder;
    use thalos_api::semantic::DocumentAnalysis;
    use thalos_api::visual::{SceneBuilder, VisualScene};

    // IO / transport contracts.
    use thalos_api::ports::device::DeviceTransport;
    use thalos_api::ports::robot::RobotTransport;

    // A non-empty body proves the names resolve in-line, not only in `use`.
    let _ = std::any::type_name::<ExecutionPlan>();
    let _ = std::any::type_name::<RuntimeProgram>();
    let _ = std::any::type_name::<ResourceRef>();
    let _ = std::any::type_name::<CapabilityRequirement>();
    let _ = std::any::type_name::<RobotModel>();
    let _ = std::any::type_name::<Robot>();

    // Execution IR surface fixed for the ADR-020 extraction (Fase 2).
    let _ = std::any::type_name::<ExecutionSegment>();
    let _ = std::any::type_name::<ExecutionWaypoint>();
    let _ = std::any::type_name::<PlanInstruction>();
    let _ = std::any::type_name::<ExecutionMetadata>();
    let _ = std::any::type_name::<ExecutionProgram>();
    let _ = std::any::type_name::<ProgramInstruction>();
    let _ = std::any::type_name::<RuntimeAction>();
    let _ = std::any::type_name::<RuntimeEvent>();

    // Robot live state, device signals and the identities supervision will use.
    let _ = std::any::type_name::<JointState>();
    let _ = std::any::type_name::<StateDeviation>();
    let _ = std::any::type_name::<StateRequirement>();
    let _ = std::any::type_name::<ChannelId>();
    let _ = std::any::type_name::<ChannelObservation>();
    let _ = std::any::type_name::<ChannelValue>();
    let _ = std::any::type_name::<ExecutionSessionId>();
    let _ = std::any::type_name::<StationId>();

    // Analysis evidence core consumed by execution comparison.
    let _ = std::any::type_name::<TracePoint>();
    let _ = std::any::type_name::<PlanExecutionComparison>();
    let _ = std::any::type_name::<ComparisonMetrics>();
}
