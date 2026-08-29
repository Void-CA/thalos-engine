use thalos_semantic::model::{
    MotionKind, MotionTarget, Provenance, ResolvedProgram, ResolvedStatement,
};

#[derive(Debug, Clone, PartialEq)]
pub struct PlanningMotion {
    pub kind: MotionKind,
    pub target: MotionTarget,
    pub provenance: Provenance,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanningInput {
    pub motions: Vec<PlanningMotion>,
}

use thalos_core::ids::OperationId;
use thalos_core::motion::segment::MotionSegment;
use thalos_core::spatial::frame::FrameId;
use crate::motion::program::PlanningProgram;

impl PlanningInput {
    pub fn from_resolved(program: &ResolvedProgram) -> Self {
        let mut motions = Vec::new();
        for stmt in &program.statements {
            if let ResolvedStatement::Motion(m) = stmt {
                motions.push(PlanningMotion {
                    kind: m.kind.clone(),
                    target: m.target.clone(),
                    provenance: m.provenance.clone(),
                });
            }
        }
        Self { motions }
    }

    pub fn to_program(&self) -> PlanningProgram {
        let segments = self
            .motions
            .iter()
            .map(|m| {
                let origin = OperationId(
                    m.provenance
                        .source_name
                        .clone()
                        .unwrap_or_else(|| "anonymous".to_string()),
                );
                match &m.target {
                    MotionTarget::Joints(j) => MotionSegment::MoveJ {
                        origin,
                        target: j.values.clone(),
                        max_velocity: None,
                        max_acceleration: None,
                    },
                    MotionTarget::Position(p) => MotionSegment::MoveLPosition {
                        origin,
                        frame: FrameId::World,
                        target_position: [p.point.x, p.point.y, p.point.z],
                        max_velocity: None,
                    },
                    MotionTarget::Pose(pose) => MotionSegment::MoveL {
                        origin,
                        frame: FrameId::World,
                        target_pose: thalos_core::spatial::pose::Pose::new(
                            FrameId::World,
                            FrameId::World,
                            pose.transform.clone(),
                        ),
                        max_velocity: None,
                    },
                }
            })
            .collect();
        PlanningProgram::new(segments)
    }
}
