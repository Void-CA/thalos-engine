use crate::prelude::*;

/// Construye un robot SCARA a partir de geometría y límites.
///
/// Normalmente no se llama directamente — usar [`ScaraSpec::build`] en su lugar.
pub fn create_scara_robot(
    base_height: f64,
    l1: f64,
    l2: f64,
    limits_j1: JointLimits,
    limits_j2: JointLimits,
    limits_j3: JointLimits,
    limits_j4: JointLimits,
) -> SerialChain {
    let mut builder = SerialChainBuilder::new();

    let base_frame = builder.create_frame("base");
    let link_1_frame = builder.create_frame("link_1");
    let link_2_frame = builder.create_frame("link_2");
    let prismatic_frame = builder.create_frame("prismatic_joint");
    let wrist_frame = builder.create_frame("wrist");

    // ── Segmento 0: Base fija ──────────────────────────────────
    let base_joint = JointType::Fixed(FixedJoint::new(Transform3D::from_translation(
        Vector3::new(0.0, 0.0, base_height),
    )));

    let base_link = Link {
        id: 0,
        transform: Transform3D::identity(),
        collision_geometry: None,
    };

    builder.add_segment(Segment {
        parent: FrameId::World,
        child: base_frame.clone(),
        joint: base_joint,
        link: base_link,
    });

    // ── Segmento 1: Revoluta en Z (base rotacional) ────────────
    let joint1 = JointType::Revolute(RevoluteJoint::new(
        0,
        UnitVector3::z_axis(),
        limits_j1,
        Transform3D::identity(),
    ));

    let link1 = Link {
        id: 1,
        transform: Transform3D::from_translation(Vector3::new(l1, 0.0, 0.0)),
        collision_geometry: None,
    };

    builder.add_segment(Segment {
        parent: base_frame,
        child: link_1_frame.clone(),
        joint: joint1,
        link: link1,
    });

    // ── Segmento 2: Revoluta en Z (codo) ───────────────────────
    let joint2 = JointType::Revolute(RevoluteJoint::new(
        1,
        UnitVector3::z_axis(),
        limits_j2,
        Transform3D::identity(),
    ));

    let link2 = Link {
        id: 2,
        transform: Transform3D::from_translation(Vector3::new(l2, 0.0, 0.0)),
        collision_geometry: None,
    };

    builder.add_segment(Segment {
        parent: link_1_frame,
        child: link_2_frame.clone(),
        joint: joint2,
        link: link2,
    });

    // ── Segmento 3: Prismática en Z (vertical) ────────────────
    let joint3 = JointType::Prismatic(PrismaticJoint::new(
        2,
        UnitVector3::z_axis(),
        limits_j3,
        Transform3D::identity(),
    ));

    let link3 = Link {
        id: 3,
        transform: Transform3D::identity(),
        collision_geometry: None,
    };

    builder.add_segment(Segment {
        parent: link_2_frame,
        child: prismatic_frame.clone(),
        joint: joint3,
        link: link3,
    });

    // ── Segmento 4: Revoluta en Z (muñeca) ─────────────────────
    let joint4 = JointType::Revolute(RevoluteJoint::new(
        3,
        UnitVector3::z_axis(),
        limits_j4,
        Transform3D::identity(),
    ));

    let link4 = Link {
        id: 4,
        transform: Transform3D::from_translation(Vector3::new(0.0, 0.0, 0.0)),
        collision_geometry: None,
    };

    builder.add_segment(Segment {
        parent: prismatic_frame,
        child: wrist_frame.clone(),
        joint: joint4,
        link: link4,
    });

    builder.set_end_effector(wrist_frame);

    builder.build().unwrap()
}
