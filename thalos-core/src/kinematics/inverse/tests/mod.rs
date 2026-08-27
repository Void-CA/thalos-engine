pub mod bench;
pub mod dls;
pub mod jt;
pub mod pose;
pub mod singularity;

use crate::kinematics::inverse::result::IKStatus;
use crate::kinematics::inverse::solvers::JacobianTransposeSolver;
use crate::prelude::*;
use std::f64::consts::PI;
use thalos_math::constants::*;
use thalos_math::*;

// ─── Helpers ───────────────────────────────────────────────────────────

/// Brazo planar de 1-DOF: un único revolute con link de longitud `L`.
/// Joint en el origen, rotación alrededor de Z, link sobre X.
fn build_1dof_arm(length: f64) -> (ForwardKinematics, FrameId) {
    let mut builder = SerialChainBuilder::new();

    let ee = builder.create_frame("ee");

    let joint = JointType::Revolute(RevoluteJoint::new(
        0,
        UnitVector3::z_axis(),
        JointLimits::new(-PI, PI),
        Transform3D::identity(),
    ));

    let link = Link::new(
        0,
        Transform3D::from_translation(Vector3::new(length, 0.0, 0.0)),
    );

    builder.add_segment(Segment::new(FrameId::World, ee.clone(), joint, link));
    builder.set_end_effector(ee.clone());

    let chain = builder.build().expect("1-DOF arm: builder failed");
    (ForwardKinematics::new(chain), ee)
}

/// Brazo planar de 2-DOF: L1 = L2 = 1, ambos revolute alrededor de Z.
/// q = [0, 0] → efector en (2, 0, 0).
fn build_2dof_planar_arm() -> (ForwardKinematics, FrameId) {
    let mut builder = SerialChainBuilder::new();

    let shoulder = builder.create_frame("shoulder");
    let ee = builder.create_frame("ee");

    // Segmento 1: World → shoulder
    let joint1 = JointType::Revolute(RevoluteJoint::new(
        0,
        UnitVector3::z_axis(),
        JointLimits::new(-PI, PI),
        Transform3D::identity(),
    ));
    let link1 = Link::new(
        0,
        Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
    );
    builder.add_segment(Segment::new(
        FrameId::World,
        shoulder.clone(),
        joint1,
        link1,
    ));

    // Segmento 2: shoulder → ee
    let joint2 = JointType::Revolute(RevoluteJoint::new(
        1,
        UnitVector3::z_axis(),
        JointLimits::new(-PI, PI),
        Transform3D::identity(),
    ));
    let link2 = Link::new(
        1,
        Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
    );
    builder.add_segment(Segment::new(shoulder, ee.clone(), joint2, link2));

    builder.set_end_effector(ee.clone());

    let chain = builder.build().expect("2-DOF arm: builder failed");
    (ForwardKinematics::new(chain), ee)
}
