use crate::kinematics::forward::ForwardKinematics;
use crate::kinematics::jacobian::{Jacobian, JacobianSolver};
use crate::robot::joint::JointKind;
use crate::spatial::frame::FrameId;
use thalos_math::{Cross, DynamicMatrix, Vector3};

pub struct ScrewJacobian {
    fk: ForwardKinematics,
    end_effector: FrameId,
}

impl ScrewJacobian {
    pub fn new(fk: ForwardKinematics, end_effector: FrameId) -> Self {
        Self { fk, end_effector }
    }
}

impl JacobianSolver for ScrewJacobian {
    fn evaluate(&self, q: &[f64]) -> Jacobian {
        let result = self.fk.evaluate(q);
        let robot = self.fk.robot();

        let n_dof: usize = robot.segments.iter().map(|s| s.joint.dof()).sum();

        let mut linear = DynamicMatrix::zeros(3, n_dof);
        let mut angular = DynamicMatrix::zeros(3, n_dof);

        let tcp = robot
            .segments
            .iter()
            .find(|s| s.child == self.end_effector)
            .map(|s| &s.child);

        let ee_pose = result
            .pose(tcp.unwrap_or(&self.end_effector))
            .expect("End effector pose not found");
        let p_e = ee_pose.transform().translation;

        let mut col = 0;

        let max_segments = robot
            .segments
            .iter()
            .position(|s| s.child == self.end_effector)
            .map(|i| i + 1)
            .unwrap_or(0);

        for segment in robot.segments.iter().take(max_segments) {
            if segment.joint.dof() == 0 {
                continue;
            }

            let parent_pose = result.pose(&segment.parent).expect("Parent pose not found");
            let joint_transform = parent_pose.transform().compose(segment.joint.origin());
            let p_i = joint_transform.translation;

            let z_i = segment.joint.axis_world(&joint_transform);

            match segment.joint.kind() {
                JointKind::Revolute | JointKind::Continuous => {
                    let omega_i = Vector3::new(z_i.x, z_i.y, z_i.z);
                    let v_i = omega_i.cross(p_e - p_i);

                    angular[(0, col)] = omega_i.x;
                    angular[(1, col)] = omega_i.y;
                    angular[(2, col)] = omega_i.z;

                    linear[(0, col)] = v_i.x;
                    linear[(1, col)] = v_i.y;
                    linear[(2, col)] = v_i.z;
                }

                JointKind::Prismatic => {
                    // ω = 0, v = z_i
                    linear[(0, col)] = z_i.x;
                    linear[(1, col)] = z_i.y;
                    linear[(2, col)] = z_i.z;
                }

                JointKind::Fixed | JointKind::Floating | JointKind::Planar => {}
            }

            col += 1;
        }

        Jacobian::new(linear, angular)
    }
}
