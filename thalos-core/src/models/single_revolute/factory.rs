use crate::prelude::*;
use thalos_math::*;

pub fn create_single_revolute(l: f64, limits_j1: JointLimits) -> SerialChain {
    let mut builder = SerialChainBuilder::new();
    let link_1_frame = builder.create_frame("link_1");

    let joint1 = JointType::Revolute(RevoluteJoint::new(
        0,
        UnitVector3::z_axis(),
        limits_j1,
        Transform3D::identity(),
    ));
    let link1 = Link {
        id: 0,
        transform: Transform3D::from_translation(Vector3::new(l, 0.0, 0.0)),
        collision_geometry: None,
    };
    builder.add_segment(Segment {
        parent: FrameId::World,
        child: link_1_frame,
        joint: joint1,
        link: link1,
    });

    builder.set_end_effector(link_1_frame);
    builder.build().unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::single_revolute::SingleRevoluteSpec;
    use thalos_math::constants::EPS;

    fn build_robot() -> SerialChain {
        SingleRevoluteSpec::ideal().build()
    }

    #[test]
    fn has_one_segment() {
        let robot = build_robot();
        assert_eq!(robot.segments.len(), 1, "Should have exactly one segment");
    }

    #[test]
    fn parent_is_world() {
        let robot = build_robot();
        assert_eq!(
            robot.segments[0].parent,
            FrameId::World,
            "Segment parent should be World"
        );
    }

    #[test]
    fn child_frame_exists_in_registry() {
        let robot = build_robot();
        assert!(
            robot.frames.get(&robot.segments[0].child).is_some(),
            "Child frame should exist"
        );
    }

    #[test]
    fn joint_is_revolute_with_z_axis() {
        let robot = build_robot();
        let joint = &robot.segments[0].joint;
        match joint {
            JointType::Revolute(revolute) => {
                let axis = &revolute.axis;
                assert!(
                    (axis.x.abs() < EPS) && (axis.y.abs() < EPS) && (axis.z - 1.0).abs() < EPS,
                    "Joint axis should be Z (0,0,1), got {:?}",
                    (axis.x, axis.y, axis.z)
                );
            }
            _ => panic!("Expected Revolute joint"),
        }
    }

    #[test]
    fn link_has_x_translation() {
        let robot = build_robot();
        let t = &robot.segments[0].link.transform.translation;
        assert!(
            (t.x - 1.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
            "Link translation should be (1,0,0), got ({},{},{})",
            t.x,
            t.y,
            t.z
        );
    }

    #[test]
    fn joint_has_correct_id() {
        let robot = build_robot();
        let joint = &robot.segments[0].joint;
        match joint {
            JointType::Revolute(revolute) => assert_eq!(revolute.id, 0, "Joint ID should be 0"),
            _ => panic!("Expected Revolute joint"),
        }
    }
}
