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
    use thalos_api::core::device::SignalQuality;
    use thalos_api::core::execution::{ExecutionPlan, RuntimeProgram};
    use thalos_api::core::ids::OperationId;
    use thalos_api::core::kinematics::inverse::IKGoal;
    use thalos_api::core::resource::{Resource, ResourceKind, ResourceRef};
    use thalos_api::core::robot::state::RobotState;
    use thalos_api::core::robot::tool_frame::ToolFrame;
    use thalos_api::core::robot::serial_chain::SerialChain;
    use thalos_api::core::spatial::pose::Pose;

    // Analysis, planning and language front-end.
    use thalos_api::analysis::workspace::WorkspaceConfig;
    use thalos_api::analysis::{ManipulabilityService, SingularityService, WorkspaceService};
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
}
