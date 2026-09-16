use thalos_semantic::model::{
    MotionKind, MotionTarget, Provenance, ResolvedProgram, ResolvedStatement,
};

#[derive(Debug, Clone, PartialEq)]
pub struct PlanningMotion {
    pub kind: MotionKind,
    pub target: MotionTarget,
    pub provenance: Provenance,
}

/// One ordered step of the program fed to planning.
///
/// The plan MUST preserve the program's order and MUST NOT silently drop
/// instructions that have no geometry (`Wait` / `SetOutput`). Motion keeps its
/// kind/target; temporal/operational steps keep their provenance so they remain
/// traceable through the plan.
#[derive(Debug, Clone, PartialEq)]
pub enum PlanningStep {
    Motion(PlanningMotion),
    Wait {
        seconds: f64,
        provenance: Provenance,
    },
    SetOutput {
        name: String,
        value: bool,
        provenance: Provenance,
    },
}

impl PlanningStep {
    pub fn provenance(&self) -> &Provenance {
        match self {
            PlanningStep::Motion(m) => &m.provenance,
            PlanningStep::Wait { provenance, .. } => provenance,
            PlanningStep::SetOutput { provenance, .. } => provenance,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanningInput {
    pub steps: Vec<PlanningStep>,
}

use thalos_core::ids::OperationId;
use thalos_core::motion::segment::MotionSegment;
use thalos_core::spatial::frame::FrameId;
use crate::motion::program::PlanningProgram;

impl PlanningInput {
    pub fn from_resolved(program: &ResolvedProgram) -> Self {
        let mut steps = Vec::new();
        for stmt in &program.statements {
            match stmt {
                ResolvedStatement::Motion(m) => steps.push(PlanningStep::Motion(PlanningMotion {
                    kind: m.kind.clone(),
                    target: m.target.clone(),
                    provenance: m.provenance.clone(),
                })),
                ResolvedStatement::Wait { seconds, provenance } => steps.push(PlanningStep::Wait {
                    seconds: *seconds,
                    provenance: provenance.clone(),
                }),
                ResolvedStatement::SetOutput {
                    name,
                    value,
                    provenance,
                } => steps.push(PlanningStep::SetOutput {
                    name: name.clone(),
                    value: *value,
                    provenance: provenance.clone(),
                }),
            }
        }
        Self { steps }
    }

    pub fn to_program(&self) -> PlanningProgram {
        let segments = self
            .steps
            .iter()
            .map(|step| match step {
                PlanningStep::Motion(m) => {
                    let origin = plan_origin(&m.provenance);
                    // `movec` is a circular move: it needs cartesian via + target
                    // (validated upstream). For any non-cartesian shape, fall back
                    // to the target-derived segment so this stays total.
                    if let MotionKind::MoveC { via } = &m.kind {
                        if let (Some(via_position), Some(target_position)) =
                            (target_position(via), target_position(&m.target))
                        {
                            return MotionSegment::MoveC {
                                origin,
                                frame: FrameId::World,
                                via_position,
                                target_position,
                                max_velocity: None,
                            };
                        }
                    }
                    target_segment(origin, &m.target)
                }
                PlanningStep::Wait { seconds, provenance } => MotionSegment::Delay {
                    origin: plan_origin(provenance),
                    seconds: *seconds,
                },
                PlanningStep::SetOutput {
                    name,
                    value,
                    provenance,
                } => MotionSegment::SetOutput {
                    origin: plan_origin(provenance),
                    channel: name.clone(),
                    value: *value,
                },
            })
            .collect();
        PlanningProgram::new(segments)
    }
}

fn plan_origin(provenance: &Provenance) -> OperationId {
    OperationId(
        provenance
            .source_name
            .clone()
            .unwrap_or_else(|| "anonymous".to_string()),
    )
}

/// Cartesian position of a motion target, if it has one (joints do not).
fn target_position(target: &MotionTarget) -> Option<[f64; 3]> {
    match target {
        MotionTarget::Position(p) => Some([p.point.x, p.point.y, p.point.z]),
        MotionTarget::Pose(p) => Some([
            p.transform.translation.x,
            p.transform.translation.y,
            p.transform.translation.z,
        ]),
        MotionTarget::Joints(_) => None,
    }
}

/// Map a resolved target to its segment, independent of the motion kind.
fn target_segment(origin: OperationId, target: &MotionTarget) -> MotionSegment {
    match target {
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
}
