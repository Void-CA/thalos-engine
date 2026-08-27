use crate::prelude::*;

/// Construye un robot esférico-polar RRP (R en Z, R en Y, P en X).
///
/// FK del efector:
///     p = ( r·cosφ·cosθ, r·cosφ·sinθ, -r·sinφ )
pub fn create_spherical_polar_rrp(
    l1: f64,
    limits_j1: JointLimits,
    limits_j2: JointLimits,
    limits_j3: JointLimits,
) -> SerialChain {
    let mut builder = SerialChainBuilder::new();

    let link_1_frame = builder.create_frame("link_1");
    let link_2_frame = builder.create_frame("link_2");
    let ee_frame = builder.create_frame("ee");

    // Joint 1 — Revolute en Z (azimut θ)
    let joint1 = JointType::Revolute(RevoluteJoint::new(
        0,
        UnitVector3::z_axis(),
        limits_j1,
        Transform3D::identity(),
    ));
    let link1 = Link {
        id: 0,
        transform: Transform3D::from_translation(Vector3::new(0.0, 0.0, l1)),
        collision_geometry: None,
    };
    builder.add_segment(Segment {
        parent: FrameId::World,
        child: link_1_frame.clone(),
        joint: joint1,
        link: link1,
    });

    // Joint 2 — Revolute en Y (polar φ)
    let joint2 = JointType::Revolute(RevoluteJoint::new(
        1,
        UnitVector3::y_axis(),
        limits_j2,
        Transform3D::identity(),
    ));
    let link2 = Link {
        id: 1,
        transform: Transform3D::identity(),
        collision_geometry: None,
    };
    builder.add_segment(Segment {
        parent: link_1_frame,
        child: link_2_frame.clone(),
        joint: joint2,
        link: link2,
    });

    // Joint 3 — Prismatic en X (extensión radial r)
    let joint3 = JointType::Prismatic(PrismaticJoint::new(
        2,
        UnitVector3::x_axis(),
        limits_j3,
        Transform3D::identity(),
    ));
    let link3 = Link {
        id: 2,
        transform: Transform3D::identity(),
        collision_geometry: None,
    };
    builder.add_segment(Segment {
        parent: link_2_frame,
        child: ee_frame.clone(),
        joint: joint3,
        link: link3,
    });

    builder.set_end_effector(ee_frame);
    builder.build().unwrap()
}
