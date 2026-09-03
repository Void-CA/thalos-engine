use std::collections::HashMap;
use thalos_importer::import_urdf;
use thalos_models::Robot as ModelRobot;

use crate::robot::joint::{FixedJoint, JointLimits, JointType, PrismaticJoint, RevoluteJoint};
use crate::robot::link::Link as CoreLink;
use crate::robot::segment::Segment;
use crate::robot::serial_chain::SerialChain;
use crate::spatial::frame::{FrameId, FrameRegistry};
use thalos_math::Transform3D;

// ─── Error ──────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub enum AdapterError {
    EmptyRobot,
    NoJoints,
    MissingLink { joint: String, link: String },
    UnsupportedJointKind { joint: String, kind: String },
    MissingAxis { joint: String },
    MissingLimits { joint: String },
    Parse(String),
}

impl std::fmt::Display for AdapterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AdapterError::EmptyRobot => write!(f, "robot has no links"),
            AdapterError::NoJoints => write!(f, "robot has no joints"),
            AdapterError::MissingLink { joint, link } => {
                write!(f, "joint `{joint}` references missing link `{link}`")
            }
            AdapterError::UnsupportedJointKind { joint, kind } => {
                write!(f, "joint `{joint}` has unsupported kind `{kind}`")
            }
            AdapterError::MissingAxis { joint } => {
                write!(f, "joint `{joint}` is missing required axis")
            }
            AdapterError::MissingLimits { joint } => {
                write!(f, "joint `{joint}` is missing limits")
            }
            AdapterError::Parse(msg) => write!(f, "URDF parse error: {msg}"),
        }
    }
}

impl std::error::Error for AdapterError {}

// ─── Public API ─────────────────────────────────────────────────

pub fn from_urdf(source: &str) -> Result<SerialChain, AdapterError> {
    let robot = import_urdf(source).map_err(|e| AdapterError::Parse(e.to_string()))?;
    auto(&robot)
}

pub fn from_robot(robot: &ModelRobot) -> Result<SerialChain, AdapterError> {
    if robot.links.is_empty() {
        return Err(AdapterError::EmptyRobot);
    }

    let ordered = robot
        .bfs_joints()
        .ok_or_else(|| AdapterError::Parse("cycle or dangling link in robot graph".into()))?;

    if ordered.is_empty() {
        return Err(AdapterError::NoJoints);
    }

    // Build reverse map: link_name → sequential id
    let mut link_ids: HashMap<&str, u32> = HashMap::new();
    for (i, (name, _)) in robot.links.iter().enumerate() {
        link_ids.insert(name.as_str(), i as u32);
    }

    let mut registry = FrameRegistry::new();
    let mut joint_counter: u32 = 0;
    let mut segments = Vec::new();

    // Ensure all link frames exist in the registry.
    for link_name in robot.links.keys() {
        registry.create(link_name);
    }

    for (idx, joint) in ordered.iter().enumerate() {
        let parent_id = link_ids
            .get(joint.parent.as_str())
            .copied()
            .ok_or_else(|| AdapterError::MissingLink {
                joint: joint.name.clone(),
                link: joint.parent.clone(),
            })?;
        let child_id = link_ids.get(joint.child.as_str()).copied().ok_or_else(|| {
            AdapterError::MissingLink {
                joint: joint.name.clone(),
                link: joint.child.clone(),
            }
        })?;

        let parent_frame = if idx == 0 {
            FrameId::World
        } else {
            FrameId::new(parent_id as u64)
        };
        let child_frame = FrameId::new(child_id as u64);

        let joint_type = build_joint_type(joint, &mut joint_counter)?;

        let core_link = CoreLink::new(child_id, Transform3D::identity());

        segments.push(Segment::new(
            parent_frame,
            child_frame,
            joint_type,
            core_link,
        ));
    }

    let last_joint = ordered.last().unwrap();
    let ee_id = link_ids.get(last_joint.child.as_str()).copied().unwrap();
    let end_effector = FrameId::new(ee_id as u64);

    Ok(SerialChain {
        segments,
        frames: registry,
        end_effector,
    })
}

pub fn from_tip(robot: &ModelRobot, target_name: &str) -> Result<SerialChain, AdapterError> {
    use thalos_models::graph::RobotGraph;

    let graph = RobotGraph::from_robot(robot);

    let target_id = graph
        .link_id(target_name)
        .ok_or_else(|| AdapterError::MissingLink {
            joint: "(path)".into(),
            link: target_name.into(),
        })?;

    let path = graph.path(graph.root, target_id).ok_or_else(|| {
        AdapterError::Parse(format!(
            "no path from root '{}' to target '{}'",
            robot.root_link, target_name
        ))
    })?;

    let mut registry = FrameRegistry::new();
    let mut joint_counter: u32 = 0;
    let mut segments = Vec::new();

    for &link_id in &path.links {
        let name = graph.link_name(link_id).unwrap_or("unknown");
        registry.create(name);
    }

    for (i, &joint_id) in path.joints.iter().enumerate() {
        let parent_link = path.links[i];
        let child_link = path.links[i + 1];

        let parent_frame = if i == 0 {
            FrameId::World
        } else {
            FrameId::new(parent_link as u64)
        };
        let child_frame = FrameId::new(child_link as u64);

        let j_name = graph.joint_name(joint_id).unwrap_or("unknown");
        let model_joint = robot
            .joints
            .get(j_name)
            .ok_or_else(|| AdapterError::Parse(format!("joint '{}' not found in robot", j_name)))?;

        let joint_type = build_joint_type(model_joint, &mut joint_counter)?;

        let core_link = CoreLink::new(child_link, Transform3D::identity());

        segments.push(Segment::new(
            parent_frame,
            child_frame,
            joint_type,
            core_link,
        ));
    }

    let end_effector = FrameId::new(target_id as u64);

    Ok(SerialChain {
        segments,
        frames: registry,
        end_effector,
    })
}

pub fn auto(robot: &ModelRobot) -> Result<SerialChain, AdapterError> {
    use thalos_models::graph::RobotGraph;

    let graph = RobotGraph::from_robot(robot);
    let leaves = graph.leaves();

    if leaves.is_empty() {
        return Err(AdapterError::EmptyRobot);
    }

    let best = {
        let named = leaves.iter().find(|&&leaf| {
            graph
                .link_name(leaf)
                .map_or(false, |name| name == "tool0" || name == "tcp")
        });
        match named {
            Some(&id) => id,
            None => leaves
                .iter()
                .max_by_key(|&&leaf| {
                    graph
                        .path(graph.root, leaf)
                        .map(|p| (graph.actuated_count(&p, robot), p.joints.len()))
                        .unwrap_or((0, 0))
                })
                .copied()
                .unwrap(),
        }
    };

    let target_name = graph.link_name(best).unwrap_or("unknown");
    from_tip(robot, target_name)
}

pub(crate) fn build_joint_type(
    joint: &thalos_models::Joint,
    counter: &mut u32,
) -> Result<JointType, AdapterError> {
    let id = *counter;
    *counter += 1;

    let origin = joint.origin.clone();
    let limits = joint.limits.unwrap_or(JointLimits::new(0.0, 0.0));

    match joint.kind {
        thalos_models::JointKind::Revolute | thalos_models::JointKind::Continuous => {
            let axis = joint.axis.ok_or_else(|| AdapterError::MissingAxis {
                joint: joint.name.clone(),
            })?;
            let limits = if matches!(joint.kind, thalos_models::JointKind::Continuous)
                && joint.limits.is_none()
            {
                JointLimits::unlimited()
            } else {
                limits
            };
            Ok(JointType::Revolute(RevoluteJoint::new(
                id, axis, limits, origin,
            )))
        }
        thalos_models::JointKind::Prismatic => {
            let axis = joint.axis.ok_or_else(|| AdapterError::MissingAxis {
                joint: joint.name.clone(),
            })?;
            let dir = axis;
            Ok(JointType::Prismatic(PrismaticJoint::new(
                id, dir, limits, origin,
            )))
        }
        thalos_models::JointKind::Fixed => Ok(JointType::Fixed(FixedJoint::new(origin))),
        ref other => Err(AdapterError::UnsupportedJointKind {
            joint: joint.name.clone(),
            kind: other.to_string(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PLANAR_2R_URDF: &str = r#"
        <robot name="planar_2r">
            <link name="base"/>
            <link name="link1"/>
            <link name="link2"/>
            <joint name="j1" type="revolute">
                <parent link="base"/>
                <child  link="link1"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <axis   xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
            </joint>
            <joint name="j2" type="revolute">
                <parent link="link1"/>
                <child  link="link2"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
                <axis   xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
            </joint>
        </robot>
    "#;

    #[test]
    fn planar_2r_from_urdf() {
        let chain = from_urdf(PLANAR_2R_URDF).unwrap();
        assert_eq!(chain.segments.len(), 2);
        assert!(chain.frames.get(&chain.end_effector).is_some());
    }

    #[test]
    fn rejects_empty_robot() {
        let robot = ModelRobot::new("empty", "base");
        let err = from_robot(&robot).unwrap_err();
        assert!(
            matches!(err, AdapterError::EmptyRobot),
            "expected EmptyRobot, got {err}"
        );
    }

    #[test]
    fn rejects_floating_joint() {
        let source = r#"
            <robot name="bad">
                <link name="a"/><link name="b"/>
                <joint name="j" type="floating">
                    <parent link="a"/><child link="b"/>
                </joint>
            </robot>
        "#;
        let err = from_urdf(source).unwrap_err();
        assert!(
            matches!(err, AdapterError::Parse(_)),
            "expected Parse error for unsupported joint type, got {err}"
        );
    }

    #[test]
    fn requires_axis_for_revolute() {
        let source = r#"
            <robot name="bad">
                <link name="a"/><link name="b"/>
                <joint name="j" type="revolute">
                    <parent link="a"/><child link="b"/>
                    <limit lower="-1" upper="1" effort="1" velocity="1"/>
                </joint>
            </robot>
        "#;
        let err = from_urdf(source).unwrap_err();
        assert!(
            matches!(err, AdapterError::MissingAxis { .. }),
            "expected MissingAxis, got {err}"
        );
    }

    #[test]
    fn scara_like_with_prismatic() {
        let source = r#"
            <robot name="scara">
                <link name="base"/>
                <link name="arm1"/>
                <link name="arm2"/>
                <link name="tool"/>
                <joint name="j1" type="revolute">
                    <parent link="base"/><child link="arm1"/>
                    <axis xyz="0 0 1"/>
                    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
                </joint>
                <joint name="j2" type="revolute">
                    <parent link="arm1"/><child link="arm2"/>
                    <axis xyz="0 0 1"/>
                    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
                </joint>
                <joint name="j3" type="prismatic">
                    <parent link="arm2"/><child link="tool"/>
                    <axis xyz="0 0 1"/>
                    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
                </joint>
            </robot>
        "#;
        let chain = from_urdf(source).unwrap();
        assert_eq!(chain.segments.len(), 3);

        assert!(matches!(chain.segments[0].joint, JointType::Revolute(_)));
        assert!(matches!(chain.segments[1].joint, JointType::Revolute(_)));
        assert!(matches!(chain.segments[2].joint, JointType::Prismatic(_)));
    }
}
