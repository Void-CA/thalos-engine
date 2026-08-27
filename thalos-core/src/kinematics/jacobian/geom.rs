use crate::kinematics::forward::ForwardKinematics;
use crate::kinematics::jacobian::{Jacobian, JacobianSolver};

use thalos_math::DynamicMatrix;

use crate::robot::joint::JointKind;
use thalos_math::Cross;

use crate::robot::tool_frame::ToolFrame;
use crate::spatial::frame::FrameId;

pub struct GeometricJacobian {
    fk: ForwardKinematics,
    end_effector: FrameId,
    tcp: Option<ToolFrame>,
}

impl GeometricJacobian {
    /// Create a Jacobian that references a specific frame (with optional offset).
    ///
    /// This is the canonical constructor. Both `new()` and `with_tcp()` delegate to it.
    ///
    /// The `reference_frame` defines which frame's pose is used as the reference point
    /// for the Jacobian calculation. If `tcp_offset` is provided, it is composed with
    /// the frame's pose to compute the actual reference point.
    fn build(
        fk: ForwardKinematics,
        reference_frame: FrameId,
        tcp_offset: Option<ToolFrame>,
    ) -> Self {
        Self {
            fk,
            end_effector: reference_frame,
            tcp: tcp_offset,
        }
    }

    /// Create a Jacobian that references the end effector frame.
    ///
    /// This is equivalent to `with_tcp(fk, ToolFrame::identity(end_effector))`.
    pub fn new(fk: ForwardKinematics, end_effector: FrameId) -> Self {
        Self::build(
            fk,
            end_effector.clone(),
            Some(ToolFrame::identity(end_effector)),
        )
    }

    /// Create a Jacobian that references a TCP frame instead of the end effector.
    ///
    /// The TCP can have an offset from its base frame. The Jacobian will compute
    /// the linear and angular velocity of the TCP point, not the base frame.
    pub fn with_tcp(fk: ForwardKinematics, tcp: ToolFrame) -> Self {
        Self::build(fk, tcp.base_frame.clone(), Some(tcp))
    }
}

impl JacobianSolver for GeometricJacobian {
    fn evaluate(&self, q: &[f64]) -> Jacobian {
        let result = self.fk.evaluate(q);

        let robot = self.fk.robot();

        let n_dof: usize = robot.segments.iter().map(|s| s.joint.dof()).sum();

        let mut linear = DynamicMatrix::zeros(3, n_dof);

        let mut angular = DynamicMatrix::zeros(3, n_dof);

        // Pose global del punto de referencia (end-effector o TCP con offset)
        // self.tcp is always Some (new() creates identity, with_tcp() creates the actual TCP)
        let tcp = self.tcp.as_ref().expect("TCP must be set");
        let base_pose = result
            .pose(&tcp.base_frame)
            .expect("TCP base frame pose not found");
        let reference_pose = base_pose.transform().compose(&tcp.transform);
        let p_e = reference_pose.translation;

        let mut col = 0;

        // Solo considerar segmentos hasta (e incluyendo) el que tiene como
        // hijo al frame objetivo. Joints debajo del frame objetivo en la
        // cadena cinemática NO afectan su Jacobiano.
        let max_segments = robot
            .segments
            .iter()
            .position(|s| s.child == self.end_effector)
            .map(|i| i + 1)
            .unwrap_or(0);

        for segment in robot.segments.iter().take(max_segments) {
            // Fixed joint: no contribuye al Jacobiano
            if segment.joint.dof() == 0 {
                continue;
            }

            // Pose global del parent
            let parent_pose = result.pose(&segment.parent).expect("Parent pose not found");

            // Frame real del joint:
            // parent * origin
            let joint_transform = parent_pose.transform().compose(segment.joint.origin());

            let p_i = joint_transform.translation;

            let z_i = segment.joint.axis_world(&joint_transform);

            match segment.joint.kind() {
                JointKind::Revolute | JointKind::Continuous => {
                    let linear_part = z_i.cross(p_e - p_i);

                    linear[(0, col)] = linear_part.x;

                    linear[(1, col)] = linear_part.y;

                    linear[(2, col)] = linear_part.z;

                    angular[(0, col)] = z_i.x;

                    angular[(1, col)] = z_i.y;

                    angular[(2, col)] = z_i.z;
                }

                JointKind::Prismatic => {
                    linear[(0, col)] = z_i.x;

                    linear[(1, col)] = z_i.y;

                    linear[(2, col)] = z_i.z;

                    // angular = 0
                }

                JointKind::Fixed | JointKind::Floating | JointKind::Planar => {
                    // no debe llegar acá (filtrado arriba)
                }
            }

            col += 1;
        }

        Jacobian::new(linear, angular)
    }
}
