/// Motion target types for the motion program.
///
/// Defines the target representation (`MotionTarget`, `MotionPose`), motion
/// constraints (`MotionProfile`), and output descriptors (`OutputChannel`,
/// `OutputValue`) that form the contract between lowering and motion backends.
///
/// All types derive `Serialize`/`Deserialize` for JSON round-tripping, and
/// `Debug`/`Clone`/`PartialEq` for testability and compiler-pass compatibility.
use serde::{Deserialize, Serialize};

/// A robot-independent pose suitable for motion targeting.
///
/// Mirrors the shape of `ResolvedPoseGoal` from `thalos-planning` but lives
/// in `thalos-core` so backends do not depend on the planning crate.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionPose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub frame: String,
}

/// A robot-independent position target — translation only, orientation is
/// left unconstrained (resolved from the robot's current configuration).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionPosition {
    pub position: [f64; 3],
    pub frame: String,
}

/// An extensible motion target.
///
/// `Pose` constrains position **and** orientation; `Position` constrains
/// only the translation — the planner drives IK with `IKGoal::Position`.
/// New variants (e.g. `JointConfiguration`, `ExternalAxis`) can be added
/// without breaking execution instructions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MotionTarget {
    Pose(MotionPose),
    Position(MotionPosition),
}

/// Concrete motion limits for an instruction.
///
/// Carries the resolved numeric values — no symbolic profile names. `max_jerk`
/// is optional because not all backends support jerk limiting.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionProfile {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_jerk: Option<f64>,
}

/// A resolved output channel descriptor.
///
/// Mirrors the document-level channel type with both a human-readable `name`
/// and a `channel_type` string that describes the electrical/logical interface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OutputChannel {
    pub name: String,
    pub channel_type: String,
}

/// A typed output value matching the document's type system.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum OutputValue {
    Bool(bool),
    Integer(i32),
    Float(f64),
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Task 1.1: MotionTarget::Pose holds correct MotionPose inner values ──

    #[test]
    fn pose_target_holds_supplied_pose() {
        let pose = MotionPose {
            position: [1.0, 2.0, 3.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        };

        let target = MotionTarget::Pose(pose.clone());

        match target {
            MotionTarget::Pose(inner) => {
                assert_eq!(inner.position, [1.0, 2.0, 3.0]);
                assert_eq!(inner.orientation, [0.0, 0.0, 0.0, 1.0]);
                assert_eq!(inner.frame, "world");
                assert_eq!(inner, pose);
            }
            MotionTarget::Position(_) => panic!("expected Pose"),
        }
    }

    #[test]
    fn pose_target_with_different_values() {
        let pose = MotionPose {
            position: [10.5, -20.0, 0.0],
            orientation: [0.707, 0.0, 0.0, 0.707],
            frame: "flange".into(),
        };

        let target = MotionTarget::Pose(pose.clone());

        match target {
            MotionTarget::Pose(inner) => {
                assert_eq!(inner.position, [10.5, -20.0, 0.0]);
                assert_eq!(inner.orientation, [0.707, 0.0, 0.0, 0.707]);
                assert_eq!(inner.frame, "flange");
                assert_eq!(inner, pose);
            }
            MotionTarget::Position(_) => panic!("expected Pose"),
        }
    }

    #[test]
    fn pose_target_round_trips_through_debug() {
        let pose = MotionPose {
            position: [0.0, 0.0, 0.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            frame: "tool0".into(),
        };
        let target = MotionTarget::Pose(pose);
        let debug = format!("{target:?}");
        assert!(debug.contains("Pose"), "Debug should contain variant name");
        assert!(debug.contains("tool0"), "Debug should contain frame");
    }

    // ── Task 1.2: MotionProfile construction ──

    #[test]
    fn motion_profile_without_jerk() {
        let profile = MotionProfile {
            max_velocity: 500.0,
            max_acceleration: 1000.0,
            max_jerk: None,
        };

        assert_eq!(profile.max_velocity, 500.0);
        assert_eq!(profile.max_acceleration, 1000.0);
        assert_eq!(profile.max_jerk, None);
    }

    #[test]
    fn motion_profile_with_jerk() {
        let profile = MotionProfile {
            max_velocity: 250.0,
            max_acceleration: 500.0,
            max_jerk: Some(750.0),
        };

        assert_eq!(profile.max_velocity, 250.0);
        assert_eq!(profile.max_acceleration, 500.0);
        assert_eq!(profile.max_jerk, Some(750.0));
    }

    #[test]
    fn motion_profile_edge_values() {
        // Zero velocities are valid (e.g. for testing or hold-in-place)
        let profile = MotionProfile {
            max_velocity: 0.0,
            max_acceleration: 0.0,
            max_jerk: Some(0.0),
        };

        assert_eq!(profile.max_velocity, 0.0);
        assert_eq!(profile.max_acceleration, 0.0);
        assert_eq!(profile.max_jerk, Some(0.0));
    }

    #[test]
    fn motion_profile_exact_equality() {
        let a = MotionProfile {
            max_velocity: 100.0,
            max_acceleration: 200.0,
            max_jerk: None,
        };

        let b = MotionProfile {
            max_velocity: 100.0,
            max_acceleration: 200.0,
            max_jerk: None,
        };

        assert_eq!(a, b, "Identical profiles should be equal");
    }

    #[test]
    fn motion_profile_inequality() {
        let a = MotionProfile {
            max_velocity: 100.0,
            max_acceleration: 200.0,
            max_jerk: None,
        };

        let b = MotionProfile {
            max_velocity: 300.0,
            max_acceleration: 200.0,
            max_jerk: None,
        };

        assert_ne!(a, b, "Different velocities should not be equal");
    }

    #[test]
    fn motion_profile_jerk_some_vs_none_inequality() {
        let with = MotionProfile {
            max_velocity: 100.0,
            max_acceleration: 200.0,
            max_jerk: Some(300.0),
        };

        let without = MotionProfile {
            max_velocity: 100.0,
            max_acceleration: 200.0,
            max_jerk: None,
        };

        assert_ne!(with, without, "Some vs None jerk should differ");
    }

    // ── Task 1.3: OutputChannel and OutputValue ──

    #[test]
    fn output_channel_holds_supplied_values() {
        let channel = OutputChannel {
            name: "gripper".into(),
            channel_type: "digital".into(),
        };

        assert_eq!(channel.name, "gripper");
        assert_eq!(channel.channel_type, "digital");
    }

    #[test]
    fn output_channel_different_values() {
        let channel = OutputChannel {
            name: "vacuum".into(),
            channel_type: "analog".into(),
        };

        assert_eq!(channel.name, "vacuum");
        assert_eq!(channel.channel_type, "analog");
    }

    #[test]
    fn output_channel_equality() {
        let a = OutputChannel {
            name: "gripper".into(),
            channel_type: "digital".into(),
        };
        let b = OutputChannel {
            name: "gripper".into(),
            channel_type: "digital".into(),
        };
        assert_eq!(a, b);
    }

    #[test]
    fn output_channel_inequality() {
        let a = OutputChannel {
            name: "gripper".into(),
            channel_type: "digital".into(),
        };
        let b = OutputChannel {
            name: "gripper".into(),
            channel_type: "analog".into(),
        };
        assert_ne!(a, b);
    }

    #[test]
    fn output_value_bool() {
        let v = OutputValue::Bool(true);
        assert_eq!(v, OutputValue::Bool(true));
        assert_ne!(v, OutputValue::Bool(false));
    }

    #[test]
    fn output_value_integer() {
        let v = OutputValue::Integer(42);
        assert_eq!(v, OutputValue::Integer(42));
        assert_ne!(v, OutputValue::Integer(0));
    }

    #[test]
    fn output_value_float() {
        let v = OutputValue::Float(3.14);
        assert_eq!(v, OutputValue::Float(3.14));
        assert_ne!(v, OutputValue::Float(0.0));
    }

    #[test]
    fn output_value_variant_inequality() {
        // Different variants should never be equal
        assert_ne!(OutputValue::Bool(true), OutputValue::Integer(1));
        assert_ne!(OutputValue::Integer(42), OutputValue::Float(42.0));
        assert_ne!(OutputValue::Bool(false), OutputValue::Float(0.0));
    }

    #[test]
    fn output_value_debug_representation() {
        assert_eq!(format!("{:?}", OutputValue::Bool(true)), "Bool(true)");
        assert_eq!(format!("{:?}", OutputValue::Integer(-5)), "Integer(-5)");
        assert!(format!("{:?}", OutputValue::Float(2.5)).starts_with("Float"));
    }

    // ── Serde round-trip (basic smoke for target types) ──

    #[test]
    fn motion_pose_serde_round_trip() {
        let pose = MotionPose {
            position: [1.0, 2.0, 3.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        };

        let json = serde_json::to_string(&pose).expect("serialize");
        let decoded: MotionPose = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(pose, decoded);
    }

    #[test]
    fn motion_profile_serde_round_trip() {
        let profile = MotionProfile {
            max_velocity: 500.0,
            max_acceleration: 1000.0,
            max_jerk: Some(2000.0),
        };

        let json = serde_json::to_string(&profile).expect("serialize");
        let decoded: MotionProfile = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(profile, decoded);
    }

    #[test]
    fn output_channel_serde_round_trip() {
        let channel = OutputChannel {
            name: "gripper".into(),
            channel_type: "digital".into(),
        };

        let json = serde_json::to_string(&channel).expect("serialize");
        let decoded: OutputChannel = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(channel, decoded);
    }

    #[test]
    fn motion_target_serde_round_trip() {
        let target = MotionTarget::Pose(MotionPose {
            position: [1.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "base".into(),
        });

        let json = serde_json::to_string(&target).expect("serialize");
        assert!(
            json.contains(r#""type":"pose""#),
            "JSON should use internally-tagged type: {json}"
        );
        let decoded: MotionTarget = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(target, decoded);
    }

    #[test]
    fn motion_target_serde_forward_compat() {
        // JSON with extra unknown field (forward compatibility test)
        let json = r#"{"type":"pose","position":[1.0,0.0,0.0],"orientation":[0.0,0.0,0.0,1.0],"frame":"base","unknown_field":42}"#;
        let decoded: Result<MotionTarget, _> = serde_json::from_str(json);
        assert!(
            decoded.is_ok(),
            "Should tolerate unknown fields for forward compatibility"
        );
    }

    #[test]
    fn position_target_holds_supplied_position() {
        let pos = MotionPosition {
            position: [0.4, 0.3, 0.2],
            frame: "world".into(),
        };
        let target = MotionTarget::Position(pos.clone());
        match target {
            MotionTarget::Position(inner) => {
                assert_eq!(inner.position, [0.4, 0.3, 0.2]);
                assert_eq!(inner.frame, "world");
                assert_eq!(inner, pos);
            }
            other => panic!("expected Position, got {other:?}"),
        }
    }

    #[test]
    fn position_target_serde_round_trip() {
        let target = MotionTarget::Position(MotionPosition {
            position: [0.4, 0.3, 0.2],
            frame: "base".into(),
        });
        let json = serde_json::to_string(&target).expect("serialize");
        assert!(
            json.contains(r#""type":"position""#),
            "JSON should use internally-tagged type: {json}"
        );
        let decoded: MotionTarget = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(target, decoded);
    }

    #[test]
    fn output_value_serde_round_trip() {
        let cases = vec![
            OutputValue::Bool(true),
            OutputValue::Integer(99),
            OutputValue::Float(-1.5),
        ];

        for value in cases {
            let json = serde_json::to_string(&value).expect("serialize");
            let decoded: OutputValue = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(value, decoded);
        }
    }
}
