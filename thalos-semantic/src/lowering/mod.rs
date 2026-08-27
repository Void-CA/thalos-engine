pub mod context;

use thalos_core::execution::program::{ExecutionMetadata, ExecutionProgram, ProgramInstruction};
use thalos_core::motion::MotionTarget;

use crate::{
    ir::SemanticIr,
    knowledge::{GraspPlan, LoweringError, PlacementPlan},
    operation::{HomeOp, MoveToOp, PickOp, PlaceOp, SemanticOperation, WaitOp},
    resource::ToolId,
};

use self::context::LoweringContext;

/// The lowering engine that converts a `SemanticIr` into an
/// `ExecutionProgram` by resolving semantic resource IDs through the
/// `KnowledgeProvider`.
///
/// Lowering is a pure function — same inputs always produce the same output.
/// All side effects (I/O, state mutation) are prohibited during lowering.
pub struct SemanticLowering;

impl SemanticLowering {
    /// Lower a `SemanticIr` into an `ExecutionProgram`.
    ///
    /// Iterates each `SemanticOperation` and emits the corresponding
    /// `ProgramInstruction` sequence:
    ///
    /// | Operation | Emitted Instructions | Source |
    /// |-----------|---------------------|--------|
    /// | Pick      | approach(MoveJ) → grasp(MoveL) → grip(SetOutput) → retract(MoveL) | grasp_plan(object) |
    /// | Place     | approach(MoveJ) → drop(MoveL) → ungrip(SetOutput) → retract(MoveL) | place_plan(object, location) |
    /// | MoveTo    | single MoveJ | location_pose(location) |
    /// | Wait      | single Delay | duration from operation |
    /// | Home      | single MoveJ | home_pose() |
    ///
    /// Returns `Err(LoweringError)` if the provider returns an error for any
    /// resource resolution. No partial `ExecutionProgram` is produced on error.
    pub fn lower(
        ir: &SemanticIr,
        ctx: &LoweringContext,
    ) -> Result<ExecutionProgram, LoweringError> {
        let mut instructions = Vec::new();

        for op in &ir.operations {
            match op {
                SemanticOperation::Skill(skill_op) => {
                    let skill_id = &skill_op.skill_call.skill;
                    let skill_resolved = if let Some(registry) = ctx.skills {
                        registry.get_for_robot(&ir.robot, skill_id)
                    } else {
                        None
                    };

                    let skill = skill_resolved.ok_or_else(|| LoweringError::UnknownSkill(skill_id.clone()))?;

                    match &skill.implementation {
                        thalos_core::skill::SkillImplementation::Program(fragment) => {
                            for inst in &fragment.instructions {
                                match inst {
                                    thalos_core::program::Instruction::Motion(_m) => {
                                        let pose = ctx.provider.home_pose()?;
                                        instructions.push(ProgramInstruction::MoveJ {
                                            origin: skill_op.origin.clone(),
                                            target: MotionTarget::Pose(pose),
                                            profile: ctx.default_profile.clone(),
                                        });
                                    }
                                    thalos_core::program::Instruction::Control(c) => match c {
                                        thalos_core::program::ControlInstruction::Wait { duration } => {
                                            instructions.push(ProgramInstruction::Delay {
                                                origin: skill_op.origin.clone(),
                                                duration: *duration,
                                            });
                                        }
                                        thalos_core::program::ControlInstruction::SetSignal { signal_id, value } => {
                                            instructions.push(ProgramInstruction::SetOutput {
                                                origin: skill_op.origin.clone(),
                                                channel: thalos_core::motion::OutputChannel {
                                                    name: signal_id.clone(),
                                                    channel_type: "digital".into(),
                                                },
                                                value: thalos_core::motion::OutputValue::Bool(*value),
                                            });
                                        }
                                        _ => {}
                                    },
                                    _ => {}
                                }
                            }
                        }
                        thalos_core::skill::SkillImplementation::Native(_) | thalos_core::skill::SkillImplementation::Planner(_) => {
                            let skill_name = skill_id.as_str();
                            if skill_name == "pick" {
                                let object_id = skill_op
                                    .skill_call
                                    .arguments
                                    .first()
                                    .and_then(|arg| match arg {
                                        thalos_core::program::Value::Target(t) => Some(crate::resource::ObjectId(t.as_str().to_string())),
                                        thalos_core::program::Value::String(s) => Some(crate::resource::ObjectId(s.clone())),
                                        _ => None,
                                    })
                                    .unwrap_or_else(|| crate::resource::ObjectId("default_object".to_string()));

                                let pick = PickOp {
                                    origin: skill_op.origin.clone(),
                                    object: object_id,
                                    tool: ctx.default_tool.clone(),
                                };
                                let plan = ctx.provider.grasp_plan(&pick.object)?;
                                Self::emit_pick(&mut instructions, &pick, &plan, ctx.default_tool.clone(), ctx);
                            } else if skill_name == "place" {
                                let object_id = skill_op
                                    .skill_call
                                    .arguments
                                    .first()
                                    .and_then(|arg| match arg {
                                        thalos_core::program::Value::Target(t) => Some(crate::resource::ObjectId(t.as_str().to_string())),
                                        thalos_core::program::Value::String(s) => Some(crate::resource::ObjectId(s.clone())),
                                        _ => None,
                                    })
                                    .unwrap_or_else(|| crate::resource::ObjectId("default_object".to_string()));

                                let dest_id = skill_op
                                    .skill_call
                                    .arguments
                                    .get(1)
                                    .and_then(|arg| match arg {
                                        thalos_core::program::Value::Target(t) => Some(crate::resource::LocationId(t.as_str().to_string())),
                                        thalos_core::program::Value::String(s) => Some(crate::resource::LocationId(s.clone())),
                                        _ => None,
                                    })
                                    .unwrap_or_else(|| crate::resource::LocationId("default_location".to_string()));

                                let place = PlaceOp {
                                    origin: skill_op.origin.clone(),
                                    object: object_id,
                                    destination: dest_id,
                                    tool: ctx.default_tool.clone(),
                                };
                                let plan = ctx.provider.place_plan(&place.object, &place.destination)?;
                                Self::emit_place(&mut instructions, &place, &plan, ctx.default_tool.clone(), ctx);
                            } else {
                                let pose = ctx.provider.home_pose()?;
                                let home = HomeOp { origin: skill_op.origin.clone() };
                                Self::emit_home(&mut instructions, &home, &pose, ctx);
                            }
                        }
                    }
                }
                SemanticOperation::Pick(pick) => {
                    let plan = ctx.provider.grasp_plan(&pick.object)?;
                    let tool = pick.tool.clone().or_else(|| ctx.default_tool.clone());
                    Self::emit_pick(&mut instructions, pick, &plan, tool, ctx);
                }
                SemanticOperation::Place(place) => {
                    let plan = ctx.provider.place_plan(&place.object, &place.destination)?;
                    let tool = place.tool.clone().or_else(|| ctx.default_tool.clone());
                    Self::emit_place(&mut instructions, place, &plan, tool, ctx);
                }
                SemanticOperation::MoveTo(mv) => {
                    let pose = ctx.provider.location_pose(&mv.destination)?;
                    Self::emit_move_to(&mut instructions, mv, &pose, ctx);
                }
                SemanticOperation::Wait(wait) => {
                    Self::emit_wait(&mut instructions, wait);
                }
                SemanticOperation::Home(home) => {
                    let pose = ctx.provider.home_pose()?;
                    Self::emit_home(&mut instructions, home, &pose, ctx);
                }
            }
        }

        Ok(ExecutionProgram {
            instructions,
            metadata: ExecutionMetadata {
                schema_version: 1,
                source_project: "thalos-semantic".into(),
            },
        })
    }

    fn emit_pick(
        instructions: &mut Vec<ProgramInstruction>,
        pick: &PickOp,
        plan: &GraspPlan,
        _tool: Option<ToolId>,
        ctx: &LoweringContext,
    ) {
        let origin = pick.origin.clone();

        // 1. Approach (MoveJ — joint-space: the JOINT profile in rad/s).
        instructions.push(ProgramInstruction::MoveJ {
            origin: origin.clone(),
            target: MotionTarget::Pose(plan.approach_frame.clone()),
            profile: ctx.default_profile.clone(),
        });

        // 2. Grasp (MoveL — cartesian: the CARTESIAN profile in m/s).
        instructions.push(ProgramInstruction::MoveL {
            origin: origin.clone(),
            target: MotionTarget::Pose(plan.grasp_frame.clone()),
            profile: ctx.cartesian_profile(),
        });

        // 3. Grip (SetOutput)
        instructions.push(ProgramInstruction::SetOutput {
            origin: origin.clone(),
            channel: crate::knowledge::gripper_channel(),
            value: thalos_core::motion::OutputValue::Bool(true),
        });

        // 4. Retract (MoveL — cartesian profile).
        instructions.push(ProgramInstruction::MoveL {
            origin,
            target: MotionTarget::Pose(plan.retreat_frame.clone()),
            profile: ctx.cartesian_profile(),
        });
    }

    fn emit_place(
        instructions: &mut Vec<ProgramInstruction>,
        place: &PlaceOp,
        plan: &PlacementPlan,
        _tool: Option<ToolId>,
        ctx: &LoweringContext,
    ) {
        let origin = place.origin.clone();

        // 1. Approach (MoveJ — JOINT profile).
        instructions.push(ProgramInstruction::MoveJ {
            origin: origin.clone(),
            target: MotionTarget::Pose(plan.approach_frame.clone()),
            profile: ctx.default_profile.clone(),
        });

        // 2. Drop (MoveL — CARTESIAN profile).
        instructions.push(ProgramInstruction::MoveL {
            origin: origin.clone(),
            target: MotionTarget::Pose(plan.drop_frame.clone()),
            profile: ctx.cartesian_profile(),
        });

        // 3. Ungrip (SetOutput)
        instructions.push(ProgramInstruction::SetOutput {
            origin: origin.clone(),
            channel: crate::knowledge::gripper_channel(),
            value: thalos_core::motion::OutputValue::Bool(false),
        });

        // 4. Retract (MoveL — cartesian profile).
        instructions.push(ProgramInstruction::MoveL {
            origin,
            target: MotionTarget::Pose(plan.retreat_frame.clone()),
            profile: ctx.cartesian_profile(),
        });
    }

    fn emit_move_to(
        instructions: &mut Vec<ProgramInstruction>,
        mv: &MoveToOp,
        pose: &thalos_core::motion::MotionPose,
        ctx: &LoweringContext,
    ) {
        instructions.push(ProgramInstruction::MoveJ {
            origin: mv.origin.clone(),
            target: MotionTarget::Pose(pose.clone()),
            profile: ctx.default_profile.clone(),
        });
    }

    fn emit_wait(instructions: &mut Vec<ProgramInstruction>, wait: &WaitOp) {
        instructions.push(ProgramInstruction::Delay {
            origin: wait.origin.clone(),
            duration: wait.duration,
        });
    }

    fn emit_home(
        instructions: &mut Vec<ProgramInstruction>,
        home: &HomeOp,
        pose: &thalos_core::motion::MotionPose,
        ctx: &LoweringContext,
    ) {
        instructions.push(ProgramInstruction::MoveJ {
            origin: home.origin.clone(),
            target: MotionTarget::Pose(pose.clone()),
            profile: ctx.default_profile.clone(),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use thalos_core::ids::OperationId;
    use thalos_core::motion::{MotionPose, MotionProfile};

    use crate::knowledge::MockKnowledgeProvider;
    use crate::resource::*;

    fn sample_pose(x: f64, y: f64, z: f64) -> MotionPose {
        MotionPose {
            position: [x, y, z],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        }
    }

    fn sample_profile() -> MotionProfile {
        MotionProfile {
            max_velocity: 500.0,
            max_acceleration: 1000.0,
            max_jerk: None,
        }
    }

    fn sample_grasp_plan() -> GraspPlan {
        GraspPlan {
            grasp_frame: sample_pose(1.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 1.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 1.0),
            preferred_tool: None,
        }
    }

    fn sample_placement_plan() -> PlacementPlan {
        PlacementPlan {
            drop_frame: sample_pose(2.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 2.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 2.0),
        }
    }

    fn sample_provider() -> MockKnowledgeProvider {
        let object = ObjectId("bolt-1".to_string());
        let location = LocationId("tray-1".to_string());

        MockKnowledgeProvider::new()
            .with_grasp_ok(object.clone(), sample_grasp_plan())
            .with_place_ok(object.clone(), location.clone(), sample_placement_plan())
            .with_location_ok(
                LocationId("shelf-a".to_string()),
                sample_pose(3.0, 0.0, 0.0),
            )
            .with_location_ok(LocationId("base".to_string()), sample_pose(0.0, 0.0, 0.0))
            .with_home_pose(Ok(sample_pose(0.0, 0.0, 0.5)))
    }

    fn sample_ctx(provider: &MockKnowledgeProvider) -> LoweringContext {
        LoweringContext {
            provider,
            skills: None,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        }
    }

    // ── Profile selection per instruction type (follow-up fix) ─────────────
    //
    // MoveJ plans in RADIANS: the cartesian demo default (0.1 m/s) must not
    // leak into joint-space MoveJ (0.1 rad/s ≈ 5.7°/s makes a 1.5 rad move
    // take ~15s). The lowering separates the profiles: approach/Home/MoveTo
    // (MoveJ) use the JOINT profile; grasp/drop/retract (MoveL) use the
    // CARTESIAN profile.

    fn joint_profile() -> MotionProfile {
        MotionProfile {
            max_velocity: 1.0,
            max_acceleration: 0.5,
            max_jerk: None,
        }
    }

    fn cartesian_profile() -> MotionProfile {
        MotionProfile {
            max_velocity: 0.1,
            max_acceleration: 0.5,
            max_jerk: None,
        }
    }

    fn split_profile_ctx(provider: &MockKnowledgeProvider) -> LoweringContext<'_> {
        LoweringContext {
            provider,
            skills: None,
            default_tool: None,
            default_profile: joint_profile(),
            default_cartesian_profile: Some(cartesian_profile()),
        }
    }

    #[test]
    fn pick_uses_joint_profile_for_approach_and_cartesian_for_movel() {
        let op = SemanticOperation::Pick(PickOp {
            origin: OperationId("pick-1".to_string()),
            object: ObjectId("bolt-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = split_profile_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        let insts = &ep.instructions;

        match &insts[0] {
            ProgramInstruction::MoveJ { profile, .. } => {
                assert_eq!(
                    *profile,
                    joint_profile(),
                    "approach MoveJ must use the JOINT profile (rad/s), got {profile:?}"
                );
            }
            other => panic!("first instruction must be MoveJ, got {other:?}"),
        }
        for (i, inst) in insts.iter().enumerate().skip(1) {
            match inst {
                ProgramInstruction::MoveL { profile, .. } => assert_eq!(
                    *profile,
                    cartesian_profile(),
                    "MoveL {i} must use the CARTESIAN profile (m/s), got {profile:?}"
                ),
                ProgramInstruction::SetOutput { .. } => {}
                other => panic!("unexpected instruction {other:?}"),
            }
        }
    }

    #[test]
    fn place_uses_joint_profile_for_approach_and_cartesian_for_movel() {
        let op = SemanticOperation::Place(PlaceOp {
            origin: OperationId("place-1".to_string()),
            object: ObjectId("bolt-1".to_string()),
            destination: LocationId("tray-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = split_profile_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        let insts = &ep.instructions;

        match &insts[0] {
            ProgramInstruction::MoveJ { profile, .. } => {
                assert_eq!(
                    *profile,
                    joint_profile(),
                    "approach MoveJ must use the JOINT profile"
                );
            }
            other => panic!("first instruction must be MoveJ, got {other:?}"),
        }
        for (i, inst) in insts.iter().enumerate().skip(1) {
            match inst {
                ProgramInstruction::MoveL { profile, .. } => assert_eq!(
                    *profile,
                    cartesian_profile(),
                    "MoveL {i} must use the CARTESIAN profile, got {profile:?}"
                ),
                ProgramInstruction::SetOutput { .. } => {}
                other => panic!("unexpected instruction {other:?}"),
            }
        }
    }

    #[test]
    fn move_to_and_home_use_the_joint_profile() {
        // MoveTo and Home are joint-space MoveJ — they must use the JOINT
        // profile even when a cartesian profile is configured.
        let provider = sample_provider();
        let ctx = split_profile_ctx(&provider);

        let program = SemanticIr::from_operations(vec![
            SemanticOperation::MoveTo(MoveToOp {
                origin: OperationId("move-1".to_string()),
                destination: LocationId("shelf-a".to_string()),
                tool: None,
            }),
            SemanticOperation::Home(HomeOp {
                origin: OperationId("home-1".to_string()),
            }),
        ]);
        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        for (i, inst) in ep.instructions.iter().enumerate() {
            match inst {
                ProgramInstruction::MoveJ { profile, .. } => assert_eq!(
                    *profile,
                    joint_profile(),
                    "MoveJ {i} must use the JOINT profile, got {profile:?}"
                ),
                other => panic!("expected MoveJ, got {other:?}"),
            }
        }
    }

    // ── Instruction count per operation ──────────────────────────────────

    #[test]
    fn pick_produces_four_instructions() {
        let op = SemanticOperation::Pick(PickOp {
            origin: OperationId("pick-1".to_string()),
            object: ObjectId("bolt-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_ok());
        let ep = result.unwrap();
        assert_eq!(ep.instructions.len(), 4);
    }

    #[test]
    fn place_produces_four_instructions() {
        let op = SemanticOperation::Place(PlaceOp {
            origin: OperationId("place-1".to_string()),
            object: ObjectId("bolt-1".to_string()),
            destination: LocationId("tray-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_ok());
        let ep = result.unwrap();
        assert_eq!(ep.instructions.len(), 4);
    }

    #[test]
    fn move_to_produces_one_instruction() {
        let op = SemanticOperation::MoveTo(MoveToOp {
            origin: OperationId("move-1".to_string()),
            destination: LocationId("shelf-a".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_ok());
        let ep = result.unwrap();
        assert_eq!(ep.instructions.len(), 1);
    }

    #[test]
    fn wait_produces_one_instruction() {
        let op = SemanticOperation::Wait(WaitOp {
            origin: OperationId("wait-1".to_string()),
            duration: Duration::from_secs(3),
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_ok());
        let ep = result.unwrap();
        assert_eq!(ep.instructions.len(), 1);
    }

    #[test]
    fn home_produces_one_instruction() {
        let op = SemanticOperation::Home(HomeOp {
            origin: OperationId("home-1".to_string()),
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_ok());
        let ep = result.unwrap();
        assert_eq!(ep.instructions.len(), 1);
    }

    // ── Instruction order per operation ─────────────────────────────────

    #[test]
    fn pick_instruction_order() {
        let op = SemanticOperation::Pick(PickOp {
            origin: OperationId("pick-1".to_string()),
            object: ObjectId("bolt-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        let insts = &ep.instructions;

        assert_eq!(insts.len(), 4);
        assert!(
            matches!(insts[0], ProgramInstruction::MoveJ { .. }),
            "First should be MoveJ (approach), got {:?}",
            insts[0]
        );
        assert!(
            matches!(insts[1], ProgramInstruction::MoveL { .. }),
            "Second should be MoveL (grasp), got {:?}",
            insts[1]
        );
        assert!(
            matches!(insts[2], ProgramInstruction::SetOutput { .. }),
            "Third should be SetOutput (grip), got {:?}",
            insts[2]
        );
        assert!(
            matches!(insts[3], ProgramInstruction::MoveL { .. }),
            "Fourth should be MoveL (retract), got {:?}",
            insts[3]
        );
    }

    #[test]
    fn place_instruction_order() {
        let op = SemanticOperation::Place(PlaceOp {
            origin: OperationId("place-1".to_string()),
            object: ObjectId("bolt-1".to_string()),
            destination: LocationId("tray-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        let insts = &ep.instructions;

        assert_eq!(insts.len(), 4);
        assert!(
            matches!(insts[0], ProgramInstruction::MoveJ { .. }),
            "First should be MoveJ (approach)"
        );
        assert!(
            matches!(insts[1], ProgramInstruction::MoveL { .. }),
            "Second should be MoveL (drop)"
        );
        assert!(
            matches!(insts[2], ProgramInstruction::SetOutput { .. }),
            "Third should be SetOutput (ungrip)"
        );
        assert!(
            matches!(insts[3], ProgramInstruction::MoveL { .. }),
            "Fourth should be MoveL (retract)"
        );
    }

    #[test]
    fn move_to_instruction_is_move_j() {
        let op = SemanticOperation::MoveTo(MoveToOp {
            origin: OperationId("move-1".to_string()),
            destination: LocationId("shelf-a".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        assert_eq!(ep.instructions.len(), 1);
        assert!(
            matches!(ep.instructions[0], ProgramInstruction::MoveJ { .. }),
            "MoveTo should produce MoveJ"
        );
    }

    #[test]
    fn wait_instruction_is_delay() {
        let op = SemanticOperation::Wait(WaitOp {
            origin: OperationId("wait-1".to_string()),
            duration: Duration::from_secs(5),
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        assert_eq!(ep.instructions.len(), 1);
        match &ep.instructions[0] {
            ProgramInstruction::Delay { duration, .. } => {
                assert_eq!(*duration, Duration::from_secs(5));
            }
            _ => panic!("Wait should produce Delay"),
        }
    }

    #[test]
    fn home_instruction_is_move_j() {
        let op = SemanticOperation::Home(HomeOp {
            origin: OperationId("home-1".to_string()),
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        assert_eq!(ep.instructions.len(), 1);
        assert!(
            matches!(ep.instructions[0], ProgramInstruction::MoveJ { .. }),
            "Home should produce MoveJ"
        );
    }

    // ── Origin propagation ──────────────────────────────────────────────

    #[test]
    fn pick_origin_propagates_to_all_four_instructions() {
        let origin = OperationId("pick-42".to_string());
        let op = SemanticOperation::Pick(PickOp {
            origin: origin.clone(),
            object: ObjectId("bolt-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        for (i, inst) in ep.instructions.iter().enumerate() {
            let inst_origin = match inst {
                ProgramInstruction::MoveJ { origin, .. } => origin,
                ProgramInstruction::MoveL { origin, .. } => origin,
                ProgramInstruction::SetOutput { origin, .. } => origin,
                ProgramInstruction::Delay { origin, .. } => origin,
            };
            assert_eq!(
                *inst_origin, origin,
                "Instruction {i} should carry origin '{origin}', got '{inst_origin}'"
            );
        }
    }

    #[test]
    fn place_origin_propagates_to_all_four_instructions() {
        let origin = OperationId("place-99".to_string());
        let op = SemanticOperation::Place(PlaceOp {
            origin: origin.clone(),
            object: ObjectId("bolt-1".to_string()),
            destination: LocationId("tray-1".to_string()),
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        for inst in &ep.instructions {
            let inst_origin = match inst {
                ProgramInstruction::MoveJ { origin, .. } => origin,
                ProgramInstruction::MoveL { origin, .. } => origin,
                ProgramInstruction::SetOutput { origin, .. } => origin,
                ProgramInstruction::Delay { origin, .. } => origin,
            };
            assert_eq!(*inst_origin, origin);
        }
    }

    #[test]
    fn home_origin_propagates() {
        let origin = OperationId("home-42".to_string());
        let op = SemanticOperation::Home(HomeOp {
            origin: origin.clone(),
        });
        let program = SemanticIr::from_operations(vec![op]);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        assert_eq!(ep.instructions.len(), 1);
        match &ep.instructions[0] {
            ProgramInstruction::MoveJ { origin: o, .. } => {
                assert_eq!(*o, origin, "Home MoveJ should carry the HomeOp origin");
            }
            other => panic!("Expected MoveJ, got {other:?}"),
        }
    }

    // ── Provider error propagation ───────────────────────────────────────

    #[test]
    fn unknown_object_in_pick_returns_error() {
        let unknown = ObjectId("unknown".to_string());
        let provider = MockKnowledgeProvider::new()
            .with_grasp_error(
                unknown.clone(),
                LoweringError::KnowledgeProvider("not found".into()),
            )
            .with_home_pose(Ok(sample_pose(0.0, 0.0, 0.0)));
        let ctx = LoweringContext {
            provider: &provider,
            skills: None,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };

        let op = SemanticOperation::Pick(PickOp {
            origin: OperationId("pick-1".to_string()),
            object: unknown,
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_err());
    }

    #[test]
    fn unknown_location_in_move_to_returns_error() {
        let unknown = LocationId("unknown".to_string());
        let provider = MockKnowledgeProvider::new()
            .with_location_error(
                unknown.clone(),
                LoweringError::KnowledgeProvider("not found".into()),
            )
            .with_home_pose(Ok(sample_pose(0.0, 0.0, 0.0)));
        let ctx = LoweringContext {
            provider: &provider,
            skills: None,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };

        let op = SemanticOperation::MoveTo(MoveToOp {
            origin: OperationId("move-1".to_string()),
            destination: unknown,
            tool: None,
        });
        let program = SemanticIr::from_operations(vec![op]);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_err());
    }

    #[test]
    fn missing_home_pose_returns_error() {
        let provider =
            MockKnowledgeProvider::new().with_home_pose(Err(LoweringError::MissingHomePose));
        let ctx = LoweringContext {
            provider: &provider,
            skills: None,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };

        let op = SemanticOperation::Home(HomeOp {
            origin: OperationId("home-1".to_string()),
        });
        let program = SemanticIr::from_operations(vec![op]);

        let result = SemanticLowering::lower(&program, &ctx);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), LoweringError::MissingHomePose);
    }

    // ── Determinism / purity ─────────────────────────────────────────────

    #[test]
    fn lower_is_deterministic() {
        let ops = vec![
            SemanticOperation::Pick(PickOp {
                origin: OperationId("op-1".to_string()),
                object: ObjectId("bolt-1".to_string()),
                tool: None,
            }),
            SemanticOperation::Place(PlaceOp {
                origin: OperationId("op-2".to_string()),
                object: ObjectId("bolt-1".to_string()),
                destination: LocationId("tray-1".to_string()),
                tool: None,
            }),
            SemanticOperation::MoveTo(MoveToOp {
                origin: OperationId("op-3".to_string()),
                destination: LocationId("shelf-a".to_string()),
                tool: None,
            }),
            SemanticOperation::Wait(WaitOp {
                origin: OperationId("op-4".to_string()),
                duration: Duration::from_secs(2),
            }),
            SemanticOperation::Home(HomeOp {
                origin: OperationId("op-5".to_string()),
            }),
        ];
        let program = SemanticIr::from_operations(ops);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let first = SemanticLowering::lower(&program, &ctx).unwrap();
        let second = SemanticLowering::lower(&program, &ctx).unwrap();

        assert_eq!(first, second);
    }

    // ── Full pipeline count ─────────────────────────────────────────────

    #[test]
    fn full_program_correct_instruction_count() {
        let ops = vec![
            SemanticOperation::Pick(PickOp {
                origin: OperationId("op-1".to_string()),
                object: ObjectId("bolt-1".to_string()),
                tool: None,
            }),
            SemanticOperation::Place(PlaceOp {
                origin: OperationId("op-2".to_string()),
                object: ObjectId("bolt-1".to_string()),
                destination: LocationId("tray-1".to_string()),
                tool: None,
            }),
            SemanticOperation::MoveTo(MoveToOp {
                origin: OperationId("op-3".to_string()),
                destination: LocationId("shelf-a".to_string()),
                tool: None,
            }),
            SemanticOperation::Wait(WaitOp {
                origin: OperationId("op-4".to_string()),
                duration: Duration::from_secs(2),
            }),
            SemanticOperation::Home(HomeOp {
                origin: OperationId("op-5".to_string()),
            }),
        ];
        let program = SemanticIr::from_operations(ops);
        let provider = sample_provider();
        let ctx = sample_ctx(&provider);

        let ep = SemanticLowering::lower(&program, &ctx).unwrap();
        // Pick(4) + Place(4) + MoveTo(1) + Wait(1) + Home(1) = 11
        assert_eq!(ep.instructions.len(), 11);
    }

    // ── SkillRegistry Resolution Vertical Slice Tests ──────────────────────

    #[test]
    fn registered_skill_resolves_and_lowers() {
        use thalos_core::ids::{ProgramName, RobotId, SkillId};
        use thalos_core::program::{Instruction, RobotProgram, SkillCall, Value};
        use thalos_core::skill::{NativeSkillId, RobotSkill, SkillImplementation, SkillRegistry};

        let mut registry = SkillRegistry::new();
        let pick_skill = RobotSkill {
            id: SkillId("pick".into()),
            name: "Pick Object".into(),
            parameters: vec![],
            preconditions: vec![],
            postconditions: vec![],
            implementation: SkillImplementation::Native(NativeSkillId("pick_native".into())),
        };
        registry.register(pick_skill);

        let program = RobotProgram {
            name: ProgramName("SkillDemo".into()),
            robot: RobotId("robot-1".into()),
            targets: vec![],
            body: vec![Instruction::Skill(SkillCall {
                skill: SkillId("pick".into()),
                arguments: vec![Value::String("bolt-1".into())],
            })],
        };

        let ir = crate::ir::normalize(&program).unwrap();
        let provider = sample_provider();
        let mut ctx = sample_ctx(&provider);
        ctx.skills = Some(&registry);

        let lowered = SemanticLowering::lower(&ir, &ctx);
        assert!(lowered.is_ok(), "Lowering registered skill should succeed: {:?}", lowered.err());
    }

    #[test]
    fn unknown_skill_is_rejected() {
        use thalos_core::ids::{ProgramName, RobotId, SkillId};
        use thalos_core::program::{Instruction, RobotProgram, SkillCall};
        use thalos_core::skill::SkillRegistry;

        let registry = SkillRegistry::new(); // Empty registry

        let program = RobotProgram {
            name: ProgramName("UnknownSkillDemo".into()),
            robot: RobotId("robot-1".into()),
            targets: vec![],
            body: vec![Instruction::Skill(SkillCall {
                skill: SkillId("weld".into()),
                arguments: vec![],
            })],
        };

        let ir = crate::ir::normalize(&program).unwrap();
        let provider = sample_provider();
        let mut ctx = sample_ctx(&provider);
        ctx.skills = Some(&registry);

        let result = SemanticLowering::lower(&ir, &ctx);
        assert_eq!(result.unwrap_err(), LoweringError::UnknownSkill(SkillId("weld".into())));
    }

    #[test]
    fn extensibility_custom_skill_without_compiler_edits() {
        use thalos_core::ids::{ProgramName, RobotId, SkillId, TargetId};
        use thalos_core::program::{ControlInstruction, Instruction, MotionInstruction, RobotProgram, SkillCall};
        use thalos_core::skill::{ProgramFragment, RobotSkill, SkillImplementation, SkillRegistry};

        let mut registry = SkillRegistry::new();
        let custom_skill = RobotSkill {
            id: SkillId("inspect_surface".into()),
            name: "Inspect Surface".into(),
            parameters: vec![],
            preconditions: vec![],
            postconditions: vec![],
            implementation: SkillImplementation::Program(ProgramFragment {
                instructions: vec![
                    Instruction::Motion(MotionInstruction::MoveJoint {
                        target: TargetId("home".into()),
                    }),
                    Instruction::Control(ControlInstruction::Wait {
                        duration: std::time::Duration::from_secs(3),
                    }),
                ],
            }),
        };
        registry.register(custom_skill);

        let program = RobotProgram {
            name: ProgramName("CustomSkillExtensibilityDemo".into()),
            robot: RobotId("robot-1".into()),
            targets: vec![],
            body: vec![Instruction::Skill(SkillCall {
                skill: SkillId("inspect_surface".into()),
                arguments: vec![],
            })],
        };

        // Prove normalize works without compiler changes
        let ir = crate::ir::normalize(&program).unwrap();
        assert_eq!(ir.operations.len(), 1);

        // Prove lowering expands ProgramFragment into instructions
        let provider = sample_provider();
        let mut ctx = sample_ctx(&provider);
        ctx.skills = Some(&registry);

        let lowered = SemanticLowering::lower(&ir, &ctx).unwrap();
        assert_eq!(lowered.instructions.len(), 2, "Custom skill inspect_surface expanded into 2 instructions");
    }

    #[test]
    fn robot_scoped_skill_resolution() {
        use thalos_core::ids::{ProgramName, RobotId, SkillId};
        use thalos_core::program::{Instruction, RobotProgram, SkillCall};
        use thalos_core::skill::{NativeSkillId, RobotSkill, SkillImplementation, SkillRegistry};

        let mut registry = SkillRegistry::new();
        let scara_pick = RobotSkill {
            id: SkillId("pick".into()),
            name: "SCARA Pick".into(),
            parameters: vec![],
            preconditions: vec![],
            postconditions: vec![],
            implementation: SkillImplementation::Native(NativeSkillId("scara_pick_driver".into())),
        };
        registry.register_for_robot(RobotId("scara_1".into()), scara_pick);

        let program = RobotProgram {
            name: ProgramName("ScaraProgram".into()),
            robot: RobotId("scara_1".into()),
            targets: vec![],
            body: vec![Instruction::Skill(SkillCall {
                skill: SkillId("pick".into()),
                arguments: vec![thalos_core::program::Value::String("bolt-1".into())],
            })],
        };

        let ir = crate::ir::normalize(&program).unwrap();
        let provider = sample_provider();
        let mut ctx = sample_ctx(&provider);
        ctx.skills = Some(&registry);

        let lowered = SemanticLowering::lower(&ir, &ctx);
        assert!(lowered.is_ok());
    }
}
