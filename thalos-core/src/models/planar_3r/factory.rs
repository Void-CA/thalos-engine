use crate::prelude::*;
use thalos_math::*;

pub fn create_planar_3r(
    l1: f64,
    l2: f64,
    l3: f64,
    limits_j1: JointLimits,
    limits_j2: JointLimits,
    limits_j3: JointLimits,
) -> SerialChain {
    let mut builder = SerialChainBuilder::new();
    let link_1_frame = builder.create_frame("link_1");
    let link_2_frame = builder.create_frame("link_2");
    let link_3_frame = builder.create_frame("link_3");

    // Joint 1
    let joint1 = JointType::Revolute(RevoluteJoint::new(
        0,
        UnitVector3::z_axis(),
        limits_j1,
        Transform3D::identity(),
    ));
    let link1 = Link {
        id: 0,
        transform: Transform3D::from_translation(Vector3::new(l1, 0.0, 0.0)),
        collision_geometry: None,
    };
    builder.add_segment(Segment {
        parent: FrameId::World,
        child: link_1_frame.clone(),
        joint: joint1,
        link: link1,
    });

    // Joint 2
    let joint2 = JointType::Revolute(RevoluteJoint::new(
        1,
        UnitVector3::z_axis(),
        limits_j2,
        Transform3D::identity(),
    ));
    let link2 = Link {
        id: 1,
        transform: Transform3D::from_translation(Vector3::new(l2, 0.0, 0.0)),
        collision_geometry: None,
    };
    builder.add_segment(Segment {
        parent: link_1_frame,
        child: link_2_frame.clone(),
        joint: joint2,
        link: link2,
    });

    // Joint 3
    let joint3 = JointType::Revolute(RevoluteJoint::new(
        2,
        UnitVector3::z_axis(),
        limits_j3,
        Transform3D::identity(),
    ));
    let link3 = Link {
        id: 2,
        transform: Transform3D::from_translation(Vector3::new(l3, 0.0, 0.0)),
        collision_geometry: None,
    };
    builder.add_segment(Segment {
        parent: link_2_frame,
        child: link_3_frame.clone(),
        joint: joint3,
        link: link3,
    });

    builder.set_end_effector(link_3_frame);
    builder.build().unwrap()
}
