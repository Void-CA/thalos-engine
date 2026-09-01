use serde::{Deserialize, Serialize};

use thalos_engine::core::execution::runtime::RuntimeProgram;
use thalos_engine::core::kinematics::{forward::ForwardKinematics, inverse::DampedLeastSquaresSolver};
use thalos_engine::core::motion::segment::MotionSegment;
use thalos_engine::core::robot::state::RobotState;
use thalos_engine::core::{
    ids::OperationId,
    operation::{Operation, OperationConstraints},
    spatial::{frame::FrameId, pose::Pose},
};
use thalos_engine::math::{Quaternion, Transform3D, UnitQuaternion, Vector3};
use thalos_engine::planning::error::CompileError;
use thalos_engine::planning::motion::compiler::{DefaultPlannerDispatcher, PlanCompiler};
use thalos_engine::planning::motion::planner::PlanningContext;
use thalos_engine::planning::motion::program::PlanningProgram;

use crate::error::RuntimeError;
use crate::services::scene::SceneService;
use crate::scene::RuntimeSnapshot;

const IK_MAX_ITERS: usize = 500;
const IK_TOLERANCE: f64 = 1e-6;
const IK_LAMBDA: f64 = 0.1;

// ── Motion plan request DTOs ──

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "kind", content = "value")]
pub enum RotationDto {
    Quaternion { w: f64, x: f64, y: f64, z: f64 },
    Ypr { roll: f64, pitch: f64, yaw: f64 },
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PoseTargetDto {
    pub translation: [f64; 3],
    #[serde(default)]
    pub rotation: Option<RotationDto>,
}

impl PoseTargetDto {
    pub fn to_pose(&self, target_frame: FrameId) -> Pose {
        let [tx, ty, tz] = self.translation;
        let translation = Vector3::new(tx, ty, tz);

        let rotation = match self.rotation {
            Some(RotationDto::Quaternion { w, x, y, z }) => {
                let q = Quaternion::new(w, x, y, z);
                UnitQuaternion::new(q.normalize_or_identity())
                    .unwrap_or_else(|_| UnitQuaternion::identity())
            }
            Some(RotationDto::Ypr { roll, pitch, yaw }) => {
                UnitQuaternion::from_euler(roll, pitch, yaw)
            }
            None => UnitQuaternion::identity(),
        };

        let transform = Transform3D {
            translation,
            rotation,
        };

        Pose::new(FrameId::World, target_frame, transform)
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type")]
pub enum MotionSegmentDto {
    #[serde(rename = "movej")]
    MoveJ {
        target: Vec<f64>,
        #[serde(default)]
        max_velocity: Option<f64>,
        #[serde(default)]
        max_acceleration: Option<f64>,
    },
    #[serde(rename = "movel")]
    MoveL {
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        max_velocity: Option<f64>,
    },
}

impl MotionSegmentDto {
    const MANUAL_ORIGIN: &'static str = "manual";

    fn into_segment(self, default_ee: FrameId) -> MotionSegment {
        match self {
            MotionSegmentDto::MoveJ {
                target,
                max_velocity,
                max_acceleration,
            } => MotionSegment::MoveJ {
                origin: OperationId(Self::MANUAL_ORIGIN.into()),
                target,
                max_velocity,
                max_acceleration,
            },
            MotionSegmentDto::MoveL {
                frame_id,
                target,
                max_velocity,
            } => {
                let frame = frame_id.map_or(default_ee, FrameId::Id);
                if target.rotation.is_some() {
                    let pose = target.to_pose(frame);
                    MotionSegment::MoveL {
                        origin: OperationId(Self::MANUAL_ORIGIN.into()),
                        frame,
                        target_pose: pose,
                        max_velocity,
                    }
                } else {
                    MotionSegment::MoveLPosition {
                        origin: OperationId(Self::MANUAL_ORIGIN.into()),
                        frame,
                        target_position: target.translation,
                        max_velocity,
                    }
                }
            }
        }
    }
}

#[derive(Debug, Default, Serialize, Deserialize, Clone)]
pub struct OperationConstraintsDto {
    #[serde(default)]
    pub position_tolerance: Option<f64>,
    #[serde(default)]
    pub orientation_tolerance: Option<f64>,
    #[serde(default)]
    pub velocity_limit: Option<f64>,
}

impl OperationConstraintsDto {
    fn into_constraints(self) -> OperationConstraints {
        OperationConstraints {
            position_tolerance: self.position_tolerance,
            orientation_tolerance: self.orientation_tolerance,
            joint_deviation_limit: None,
            velocity_limit: self.velocity_limit,
            approach_direction: None,
            retreat_direction: None,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type")]
pub enum OperationDto {
    #[serde(rename = "pick")]
    Pick {
        id: u64,
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        constraints: OperationConstraintsDto,
    },
    #[serde(rename = "place")]
    Place {
        id: u64,
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        constraints: OperationConstraintsDto,
    },
    #[serde(rename = "transit")]
    Transit {
        id: u64,
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        constraints: OperationConstraintsDto,
    },
}

impl OperationDto {
    pub fn into_operation(self, default_ee: FrameId) -> Operation {
        match self {
            OperationDto::Pick {
                id,
                frame_id,
                target,
                constraints,
            } => Operation::Pick {
                id: OperationId(id.to_string()),
                target_pose: target.to_pose(frame_id.map_or(default_ee, FrameId::Id)),
                constraints: constraints.into_constraints(),
            },
            OperationDto::Place {
                id,
                frame_id,
                target,
                constraints,
            } => Operation::Place {
                id: OperationId(id.to_string()),
                target_pose: target.to_pose(frame_id.map_or(default_ee, FrameId::Id)),
                constraints: constraints.into_constraints(),
            },
            OperationDto::Transit {
                id,
                frame_id,
                target,
                constraints,
            } => Operation::Transit {
                id: OperationId(id.to_string()),
                target_pose: target.to_pose(frame_id.map_or(default_ee, FrameId::Id)),
                constraints: constraints.into_constraints(),
            },
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MotionPlanRequest {
    #[serde(default)]
    pub segments: Vec<MotionSegmentDto>,
    #[serde(default)]
    pub operations: Option<Vec<OperationDto>>,
}

impl MotionPlanRequest {
    pub fn into_program(self, default_ee: FrameId) -> PlanningProgram {
        let segments = self
            .segments
            .into_iter()
            .map(|s| s.into_segment(default_ee))
            .collect();
        PlanningProgram::new(segments)
    }
}

use std::sync::Arc;

// ── Planning Application Service ──

pub struct PlanningService {
    scene: Arc<SceneService>,
}

impl PlanningService {
    pub fn new(scene: Arc<SceneService>) -> Self {
        Self { scene }
    }

    /// Compile a motion plan request and schedule it on the scene.
    /// Returns the updated runtime state. The robot does NOT move — this is strictly
    /// a "compile + preview" operation. Execution requires a subsequent
    /// call to `start_execution`.
    pub async fn preview_plan(
        &self,
        mut payload: MotionPlanRequest,
    ) -> Result<RuntimeSnapshot, RuntimeError> {
        let snapshot = self.scene.snapshot().await?;
        let default_frame = snapshot.resolve_default_frame();

        let fk = ForwardKinematics::new(snapshot.chain.clone());
        let solver = DampedLeastSquaresSolver::new(
            fk,
            default_frame,
            IK_MAX_ITERS,
            IK_TOLERANCE,
            IK_LAMBDA,
        );
        let robot_state = RobotState::from_positions(snapshot.joints.clone());
        let ctx = PlanningContext {
            robot: &snapshot.chain,
            current_state: &robot_state,
            ik_solver: &solver,
            tcp: snapshot.active_tcp.as_ref(),
        };
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let map_err = |err: CompileError| RuntimeError::CompileFailed {
            segment_index: err.segment_index,
            message: err.to_string(),
        };

        let compiled = if let Some(ops) = payload.operations.take() {
            if ops.is_empty() {
                return Ok(snapshot);
            }
            let operations: Vec<Operation> = ops
                .into_iter()
                .map(|op| op.into_operation(default_frame))
                .collect();
            compiler
                .compile_with_operations(&operations, &ctx)
                .map_err(map_err)?
                .plan
        } else {
            let program = payload.into_program(default_frame);
            if program.segments.is_empty() {
                return Ok(snapshot);
            }
            compiler.compile(&program, &ctx).map_err(map_err)?
        };

        let updated_snapshot = self
            .scene
            .schedule_program(compiled, RuntimeProgram::default())
            .await?;
        Ok(updated_snapshot)
    }
}
