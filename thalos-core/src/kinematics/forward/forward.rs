use crate::{
    kinematics::forward::result::FKResult,
    robot::serial_chain::SerialChain,
    spatial::{frame::FrameId, pose::Pose},
};
use std::collections::HashMap;
use thalos_math::Transform3D;

#[derive(Clone)]
pub struct ForwardKinematics {
    chain: SerialChain,
}

impl ForwardKinematics {
    pub fn new(chain: SerialChain) -> Self {
        Self { chain }
    }

    pub fn robot(&self) -> &SerialChain {
        &self.chain
    }

    pub fn evaluate(&self, q: &[f64]) -> FKResult {
        assert_eq!(
            q.len(),
            self.chain.dof_count(),
            "ForwardKinematics::evaluate: q has {} elements but chain has {} DOF ({} segments, {} with dof>0)",
            q.len(),
            self.chain.dof_count(),
            self.chain.segments.len(),
            self.chain
                .segments
                .iter()
                .filter(|s| s.joint.dof() > 0)
                .count(),
        );
        let mut t = Transform3D::identity();

        let mut poses = HashMap::new();

        let world = FrameId::World;

        // World pose explícita
        poses.insert(
            world.clone(),
            Pose::new(world.clone(), world.clone(), Transform3D::identity()),
        );

        let mut q_idx = 0;

        for segment in &self.chain.segments {
            // joint local origin
            t = t.compose(segment.joint.origin());

            // joint motion (solo si aporta DOF; Fixed no consume q)
            if segment.joint.dof() > 0 {
                let q_i = q[q_idx];
                let joint_motion = segment.joint.motion(q_i);
                t = t.compose(&joint_motion);
                q_idx += 1;
            }

            // rigid link transform
            t = t.compose(&segment.link.transform);

            // store global pose of child frame
            poses.insert(
                segment.child.clone(),
                Pose::new(world.clone(), segment.child.clone(), t.clone()),
            );
        }

        let end_effector = *self.chain.end_effector();
        FKResult::new(poses, end_effector)
    }
}
