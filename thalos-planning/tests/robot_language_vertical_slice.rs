//! Product Vertical Slice Integration Suite: Language to Robot Physical Execution Pipeline
//!
//! Validates the end-to-end user experience and product invariants:
//! L01 — Valid program compiles, plans, and produces a 3D trajectory preview.
//! L02 — DOF Mismatch is rejected with a categorized JointCountMismatch diagnostic.
//! L03 — Joint limit violation is rejected with a JointLimitViolation diagnostic.
//! L04 — Cartesian unreachable target is rejected with a Kinematic unreachability diagnostic.
//! L05 — Same program evaluated against different robot contexts (Language != Physical Robot).
//! L06 — Dynamic robot context switching in UI workbench changes analysis result without source mutation.

use thalos_core::kinematics::forward::ForwardKinematics;
use thalos_core::kinematics::inverse::DampedLeastSquaresSolver;
use thalos_core::models::planar_2r::Planar2RSpec;
use thalos_core::models::{RobotModel, RobotRegistry};
use thalos_core::robot::joint::JointLimits;
use thalos_core::robot::serial_chain::SerialChain;
use thalos_core::robot::state::RobotState;
use thalos_lang::parse_source;
use thalos_math::constants::PI;
use thalos_planning::error::{CompileError, PlanningError};
use thalos_planning::input::PlanningInput;
use thalos_planning::motion::compiler::{DefaultPlannerDispatcher, PlanCompiler};
use thalos_planning::motion::planner::SegmentPlanningContext;
use thalos_planning::motion::program::CompiledPlan;
use thalos_semantic::compiler::SemanticCompiler;
use thalos_semantic::resolver::SemanticResolver;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticSeverity {
    Error,
    Warning,
    Info,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticKind {
    Syntax,
    Type,
    Capability,
    Configuration,
    Kinematic,
    Constraint,
    Collision,
    Planning,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CategorizedDiagnostic {
    pub severity: DiagnosticSeverity,
    pub kind: DiagnosticKind,
    pub code: String,
    pub message: String,
}

#[derive(Debug)]
pub enum AnalysisResult {
    Valid {
        plan: CompiledPlan,
        waypoint_count: usize,
        duration_s: f64,
    },
    Invalid {
        diagnostics: Vec<CategorizedDiagnostic>,
    },
}

/// Helper function representing the UI Workbench "Analyze / Plan" service pipeline.
pub fn analyze_and_plan(source: &str, robot: &SerialChain) -> AnalysisResult {
    // 1. Parse .thalos source -> AST
    let ast = match parse_source(source) {
        Ok(ast) => ast,
        Err(parse_errors) => {
            let diagnostics = parse_errors
                .into_iter()
                .map(|e| CategorizedDiagnostic {
                    severity: DiagnosticSeverity::Error,
                    kind: DiagnosticKind::Syntax,
                    code: "THL_SYNTAX_ERROR".into(),
                    message: format!("{}", e),
                })
                .collect();
            return AnalysisResult::Invalid { diagnostics };
        }
    };

    // 2. Compile AST -> SemanticProgram
    let sem_program = match SemanticCompiler::compile(&ast) {
        Ok(sem) => sem,
        Err(sem_errors) => {
            let diagnostics = sem_errors
                .into_iter()
                .map(|e| CategorizedDiagnostic {
                    severity: DiagnosticSeverity::Error,
                    kind: DiagnosticKind::Type,
                    code: "THL_SEMANTIC_ERROR".into(),
                    message: e,
                })
                .collect();
            return AnalysisResult::Invalid { diagnostics };
        }
    };

    // 3. Resolve SemanticProgram -> ResolvedProgram
    let resolved = match SemanticResolver::resolve(&sem_program) {
        Ok(res) => res,
        Err(res_errors) => {
            let diagnostics = vec![CategorizedDiagnostic {
                severity: DiagnosticSeverity::Error,
                kind: DiagnosticKind::Configuration,
                code: "THL_RESOLVE_ERROR".into(),
                message: res_errors.join("; "),
            }];
            return AnalysisResult::Invalid { diagnostics };
        }
    };

    // 4. Lower ResolvedProgram -> PlanningInput
    let planning_input = PlanningInput::from_resolved(&resolved);

    // 5. Build SegmentPlanningContext for the SPECIFIED Robot
    let state = RobotState::zero(robot.dof_count());
    let fk = ForwardKinematics::new(robot.clone());
    let ik_solver = DampedLeastSquaresSolver::new(fk, *robot.end_effector(), 500, 1e-6, 0.1);
    let ctx = SegmentPlanningContext {
        robot,
        current_state: &state,
        ik_solver: &ik_solver,
        tcp: None,
    };

    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

    // 6. Compile Plan & Validate Kinematic/Physical Feasibility
    match compiler.compile(&planning_input.to_program(), &ctx) {
        Ok(compiled_plan) => AnalysisResult::Valid {
            waypoint_count: compiled_plan.waypoint_count,
            duration_s: compiled_plan.duration,
            plan: compiled_plan,
        },
        Err(CompileError {
            segment_index,
            source,
        }) => {
            let (kind, code) = match &source {
                PlanningError::JointCountMismatch { .. } => (
                    DiagnosticKind::Configuration,
                    "THL_JOINT_COUNT_MISMATCH".into(),
                ),
                PlanningError::JointLimitViolation { .. } => (
                    DiagnosticKind::Constraint,
                    "THL_JOINT_LIMIT_VIOLATION".into(),
                ),
                PlanningError::IkFailed { .. }
                | PlanningError::IkFailedPosition { .. }
                | PlanningError::Ik(_) => {
                    (DiagnosticKind::Kinematic, "THL_UNREACHABLE_TARGET".into())
                }
                _ => (DiagnosticKind::Planning, "THL_PLANNING_ERROR".into()),
            };

            let diagnostics = vec![CategorizedDiagnostic {
                severity: DiagnosticSeverity::Error,
                kind,
                code,
                message: format!("Segment {}: {}", segment_index + 1, source),
            }];

            AnalysisResult::Invalid { diagnostics }
        }
    }
}

/// L01 — Valid program compiles, plans, and produces a 3D trajectory preview.
#[test]
fn l01_valid_program_produces_trajectory_plan() {
    let source = "
    target home = joints(10deg, 20deg)
    fn main() {
        movej(home);
    }
    ";
    let robot = RobotRegistry::create_default(RobotModel::Planar2R);

    match analyze_and_plan(source, &robot) {
        AnalysisResult::Valid {
            plan,
            waypoint_count,
            duration_s,
        } => {
            assert!(waypoint_count > 0);
            assert!(duration_s > 0.0);
            assert_eq!(plan.segments.len(), 1);
            assert_eq!(plan.merged_trajectory.waypoints().len(), waypoint_count);
        }
        AnalysisResult::Invalid { diagnostics } => {
            panic!("Expected valid analysis result, got diagnostics: {:?}", diagnostics);
        }
    }
}

/// L02 — DOF Mismatch: Program requesting 4 joints on a 2 DOF Planar 2R is rejected with JointCountMismatch.
#[test]
fn l02_dof_mismatch_rejected_with_categorized_diagnostic() {
    let source = "
    target t4 = joints(10deg, 20deg, 30deg, 40deg)
    fn main() {
        movej(t4);
    }
    ";
    let robot = RobotRegistry::create_default(RobotModel::Planar2R); // 2 DOF

    match analyze_and_plan(source, &robot) {
        AnalysisResult::Valid { .. } => {
            panic!("Program with 4 joints MUST be rejected on a 2 DOF robot");
        }
        AnalysisResult::Invalid { diagnostics } => {
            assert_eq!(diagnostics.len(), 1);
            let diag = &diagnostics[0];
            assert_eq!(diag.kind, DiagnosticKind::Configuration);
            assert_eq!(diag.code, "THL_JOINT_COUNT_MISMATCH");
            assert!(diag.message.contains("expected 2 DOF for robot, got 4 joints"));
        }
    }
}

/// L03 — Joint Limit Violation: Target exceeding physical joint limits is rejected with JointLimitViolation.
#[test]
fn l03_joint_limit_violation_rejected() {
    // Program targets 80deg on Joint 2
    let source = "
    target t_limit = joints(10deg, 80deg)
    fn main() {
        movej(t_limit);
    }
    ";

    // Define Planar 2R with Joint 2 restricted to [-45deg, +45deg] (80deg = 1.396 rad exceeds limit)
    let spec = Planar2RSpec::new(
        1.0,
        1.0,
        [
            JointLimits::new(-PI, PI),
            JointLimits::new(-PI / 4.0, PI / 4.0),
        ],
    );
    let restricted_robot = spec.build();

    match analyze_and_plan(source, &restricted_robot) {
        AnalysisResult::Valid { .. } => {
            panic!("Target exceeding joint limits MUST be rejected");
        }
        AnalysisResult::Invalid { diagnostics } => {
            assert_eq!(diagnostics.len(), 1);
            let diag = &diagnostics[0];
            assert_eq!(diag.kind, DiagnosticKind::Constraint);
            assert_eq!(diag.code, "THL_JOINT_LIMIT_VIOLATION");
            assert!(diag.message.contains("Joint limit violation at joint 1"));
        }
    }
}

/// L04 — Cartesian Unreachable Target: Target outside robot reach is rejected with Kinematic diagnostic.
#[test]
fn l04_cartesian_unreachable_target_rejected() {
    let source = "
    target unreachable = position([5000mm, 5000mm, 5000mm])
    fn main() {
        movel(unreachable);
    }
    ";
    let robot = RobotRegistry::create_default(RobotModel::Planar2R);

    match analyze_and_plan(source, &robot) {
        AnalysisResult::Valid { .. } => {
            panic!("Unreachable target MUST be rejected");
        }
        AnalysisResult::Invalid { diagnostics } => {
            assert_eq!(diagnostics.len(), 1);
            let diag = &diagnostics[0];
            assert_eq!(diag.kind, DiagnosticKind::Kinematic);
            assert_eq!(diag.code, "THL_UNREACHABLE_TARGET");
        }
    }
}

/// L05 — Language != Physical Robot: Same program evaluated against 3 different robot contexts.
#[test]
fn l05_same_program_different_robot_contexts() {
    let source = "
    target t2 = joints(10deg, 20deg)
    fn main() {
        movej(t2);
    }
    ";

    let planar_2r = RobotRegistry::create_default(RobotModel::Planar2R); // 2 DOF
    let manipulator_3dof = RobotRegistry::create_default(RobotModel::Manipulator3DOF); // 3 DOF
    let single_revolute = RobotRegistry::create_default(RobotModel::SingleRevolute); // 1 DOF

    // 1. Evaluated on Planar 2R -> VALID
    assert!(matches!(
        analyze_and_plan(source, &planar_2r),
        AnalysisResult::Valid { .. }
    ));

    // 2. Evaluated on Manipulator 3Dof -> INVALID (expected 3 DOF, got 2)
    match analyze_and_plan(source, &manipulator_3dof) {
        AnalysisResult::Invalid { diagnostics } => {
            assert_eq!(diagnostics[0].code, "THL_JOINT_COUNT_MISMATCH");
            assert!(diagnostics[0].message.contains("expected 3 DOF for robot, got 2 joints"));
        }
        _ => panic!("Expected invalid result for Manipulator3Dof"),
    }

    // 3. Evaluated on Single Revolute -> INVALID (expected 1 DOF, got 2)
    match analyze_and_plan(source, &single_revolute) {
        AnalysisResult::Invalid { diagnostics } => {
            assert_eq!(diagnostics[0].code, "THL_JOINT_COUNT_MISMATCH");
            assert!(diagnostics[0].message.contains("expected 1 DOF for robot, got 2 joints"));
        }
        _ => panic!("Expected invalid result for SingleRevolute"),
    }
}

/// L06 — Dynamic Robot Context Switching: Swapping active robot in UI workbench changes analysis result dynamically.
#[test]
fn l06_dynamic_robot_context_switching_in_workbench() {
    let source_code = "
    target pick_target = joints(15deg, 30deg)
    fn main() {
        movej(pick_target);
    }
    ";

    // User selects Planar 2R in Workbench -> Analysis is VALID & PREVIEW READY
    let workbench_robot_1 = RobotRegistry::create_default(RobotModel::Planar2R);
    let result_1 = analyze_and_plan(source_code, &workbench_robot_1);
    assert!(matches!(result_1, AnalysisResult::Valid { .. }));

    // User switches Workbench Dropdown to SCARA (4 DOF) -> Analysis automatically turns INVALID with precise diagnostic
    let workbench_robot_2 = RobotRegistry::create_default(RobotModel::Scara);
    let result_2 = analyze_and_plan(source_code, &workbench_robot_2);

    if let AnalysisResult::Invalid { diagnostics } = result_2 {
        assert_eq!(diagnostics[0].kind, DiagnosticKind::Configuration);
        assert_eq!(diagnostics[0].code, "THL_JOINT_COUNT_MISMATCH");
        assert!(diagnostics[0].message.contains("expected 4 DOF for robot, got 2 joints"));
    } else {
        panic!("Workbench context switch to SCARA must yield invalid analysis");
    }
}
