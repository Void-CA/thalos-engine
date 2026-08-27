use std::time::Duration;
use thalos_core::ids::{
    ObjectId, ProgramName, RobotId, SkillId, TargetId, TargetName,
};
use thalos_core::program::{
    ControlInstruction, Instruction, JointPosition, MotionInstruction, RobotProgram, SkillCall,
    Target, TargetReference, Value,
};
use thalos_core::skill::{
    Condition, NativeSkillId, Parameter, ProgramFragment, RobotSkill, SkillImplementation,
    SkillPlanner,
};
use thalos_core::spatial::frame::FrameId;
use thalos_core::spatial::pose::Pose;
use thalos_math::{Transform3D, Vector3};

fn sample_pose() -> Pose {
    Pose::new(
        FrameId::World,
        FrameId::Id(1),
        Transform3D::from_translation(Vector3::new(100.0, 50.0, 20.0)),
    )
}

#[test]
fn test_robot_program_construction_and_serde() {
    let target_1 = Target::new(
        TargetId("target-approach".to_string()),
        TargetName("pick_approach".to_string()),
        TargetReference::Cartesian { pose: sample_pose() },
    );

    let target_2 = Target::new(
        TargetId("target-joint-home".to_string()),
        TargetName("joint_home".to_string()),
        TargetReference::Joint {
            position: JointPosition::new(vec![0.0, 1.57, -1.57, 0.0]),
        },
    );

    let target_3 = Target::new(
        TargetId("target-relative".to_string()),
        TargetName("pick_offset".to_string()),
        TargetReference::Relative {
            reference: TargetId("target-approach".to_string()),
            transform: Transform3D::from_translation(Vector3::new(0.0, 0.0, -10.0)),
        },
    );

    let target_4 = Target::new(
        TargetId("target-object".to_string()),
        TargetName("part_target".to_string()),
        TargetReference::Object {
            object: ObjectId("part_01".to_string()),
            offset: Transform3D::identity(),
        },
    );

    let instructions = vec![
        Instruction::Motion(MotionInstruction::MoveJoint {
            target: TargetId("target-joint-home".to_string()),
        }),
        Instruction::Motion(MotionInstruction::Approach {
            target: TargetId("target-approach".to_string()),
            distance: 50.0,
        }),
        Instruction::Skill(SkillCall::new(
            SkillId("pick_skill".to_string()),
            vec![Value::Target(TargetId("target-object".to_string()))],
        )),
        Instruction::Control(ControlInstruction::Wait {
            duration: Duration::from_secs(1),
        }),
    ];

    let program = RobotProgram::new(
        ProgramName("pick_and_place_main".to_string()),
        RobotId("scara_robot_01".to_string()),
        vec![target_1, target_2, target_3, target_4],
        instructions,
    );

    assert_eq!(program.name.as_str(), "pick_and_place_main");
    assert_eq!(program.robot.as_str(), "scara_robot_01");
    assert_eq!(program.targets.len(), 4);
    assert_eq!(program.body.len(), 4);

    // Verify lossless serde round-trip
    let json = serde_json::to_string(&program).expect("serialize program");
    let decoded: RobotProgram = serde_json::from_str(&json).expect("deserialize program");
    assert_eq!(program, decoded);
}

#[test]
fn test_robot_skill_implementations_and_serde() {
    // 1. Program-backed skill
    let fragment = ProgramFragment {
        instructions: vec![
            Instruction::Motion(MotionInstruction::Approach {
                target: TargetId("target-grasp".to_string()),
                distance: 25.0,
            }),
            Instruction::Control(ControlInstruction::SetSignal {
                signal_id: "gripper_close".to_string(),
                value: true,
            }),
        ],
    };

    let skill_program = RobotSkill::new(
        SkillId("pick_composite".to_string()),
        "Pick Composite".to_string(),
        vec![Parameter {
            name: "workpiece".to_string(),
            param_type: "ObjectId".to_string(),
        }],
        vec![Condition::Custom {
            identifier: "gripper.open".to_string(),
            expected_value: "true".to_string(),
        }],
        vec![Condition::Custom {
            identifier: "gripper.closed".to_string(),
            expected_value: "true".to_string(),
        }],
        SkillImplementation::Program(fragment),
    );

    // 2. Planner-backed skill
    let skill_planner = RobotSkill::new(
        SkillId("pick_dynamic".to_string()),
        "Pick Dynamic Planner".to_string(),
        vec![],
        vec![],
        vec![],
        SkillImplementation::Planner(SkillPlanner {
            policy: "min_jerk_collision_free".to_string(),
        }),
    );

    // 3. Native hardware skill
    let skill_native = RobotSkill::new(
        SkillId("emergency_stop".to_string()),
        "Emergency Stop".to_string(),
        vec![],
        vec![],
        vec![],
        SkillImplementation::Native(NativeSkillId("hw_estop_v1".to_string())),
    );

    for skill in &[skill_program, skill_planner, skill_native] {
        let json = serde_json::to_string(skill).expect("serialize skill");
        let decoded: RobotSkill = serde_json::from_str(&json).expect("deserialize skill");
        assert_eq!(skill, &decoded);
    }
}
