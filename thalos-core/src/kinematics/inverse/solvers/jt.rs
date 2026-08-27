use crate::kinematics::forward::ForwardKinematics;
use crate::kinematics::inverse::{
    IKSolver,
    error::IkError,
    result::IKResult,
    solver::{IKGoal, compute_pose_error},
};
use crate::kinematics::jacobian::{GeometricJacobian, JacobianSolver};
use crate::robot::joint::{JointKind, JointLimits};
use crate::robot::serial_chain::SerialChain;
use crate::spatial::frame::FrameId;
use thalos_math::algebra::{DynamicVector, vector_to_dynamic};

pub struct JacobianTransposeSolver {
    jacobian: GeometricJacobian,
    fk: ForwardKinematics,
    end_effector: FrameId,
    max_iters: usize,
    tolerance: f64,
    alpha: f64,
    track_history: bool,
}

impl JacobianTransposeSolver {
    pub fn new(
        fk: ForwardKinematics,
        end_effector: FrameId,
        max_iters: usize,
        tolerance: f64,
        alpha: f64,
    ) -> Self {
        let jacobian = GeometricJacobian::new(fk.clone(), end_effector);
        Self {
            jacobian,
            fk,
            end_effector,
            max_iters,
            tolerance,
            alpha,
            track_history: false,
        }
    }

    /// Habilita el registro del historial de error por iteración.
    pub fn with_history(mut self, enabled: bool) -> Self {
        self.track_history = enabled;
        self
    }
}

impl IKSolver for JacobianTransposeSolver {
    fn robot(&self) -> Option<&SerialChain> {
        Some(self.fk.robot())
    }

    fn solve(&self, q0: &[f64], goal: IKGoal) -> Result<IKResult, IkError> {
        let mut q = DynamicVector::from_column_slice(q0);
        let mut error_history = if self.track_history {
            Some(Vec::with_capacity(self.max_iters))
        } else {
            None
        };

        // Extraer límites articulares para clamping post-iteración
        // Solo joints actuados (Fixed no consume q, no tiene límites activos)
        let n_joints: usize = self.fk.robot().segments.iter().map(|s| s.joint.dof()).sum();
        let joint_limits: Vec<JointLimits> = self
            .fk
            .robot()
            .segments
            .iter()
            .filter(|s| s.joint.dof() > 0)
            .map(|s| s.joint.limits())
            .collect();
        let joint_kinds: Vec<JointKind> = self
            .fk
            .robot()
            .segments
            .iter()
            .filter(|s| s.joint.dof() > 0)
            .map(|s| s.joint.kind())
            .collect();

        for iteration in 0..self.max_iters {
            let fk_result = self.fk.evaluate(q.as_slice());
            let jacobian = self.jacobian.evaluate(q.as_slice());

            let ee_pose = fk_result
                .pose(&self.end_effector)
                .expect("target frame not found in FK result");
            let (error_vec, magnitude) = match &goal {
                IKGoal::Position(target_pos) => {
                    let p = ee_pose.translation();
                    let error = *target_pos - p;
                    let mag = error.magnitude();
                    (vector_to_dynamic(error), mag)
                }
                IKGoal::Pose(target_pose) => {
                    let error = compute_pose_error(ee_pose, target_pose);
                    let mag = error.magnitude();
                    (error, mag)
                }
            };

            if let Some(ref mut history) = error_history {
                history.push(magnitude);
            }

            if magnitude < self.tolerance {
                return Ok(IKResult::converged(
                    q.as_slice().to_vec(),
                    iteration + 1,
                    magnitude,
                    error_history,
                ));
            }

            let dq = match &goal {
                IKGoal::Position(_) => {
                    let j_lin = jacobian.linear();
                    self.alpha * (j_lin.transpose() * error_vec)
                }
                IKGoal::Pose(_) => {
                    let j_full = jacobian.full();
                    self.alpha * (j_full.transpose() * error_vec)
                }
            };
            q += dq;

            // Aplicar límites articulares: clamp para revolutes con rango finito,
            // wrap para continuous (rotación infinita), clamp para prismáticos.
            // Fixed no debería llegar acá (filtrado por dof() > 0)
            for i in 0..n_joints {
                let kind = joint_kinds[i];
                q[i] = match kind {
                    JointKind::Continuous => joint_limits[i].wrap(q[i]),
                    JointKind::Revolute | JointKind::Prismatic => joint_limits[i].clamp(q[i]),
                    JointKind::Fixed | JointKind::Floating | JointKind::Planar => {
                        return Err(IkError::UnsupportedJointType(kind));
                    }
                };
            }
        }

        // Último error después de agotar iteraciones
        let fk_result = self.fk.evaluate(q.as_slice());
        let final_error = match &goal {
            IKGoal::Position(target_pos) => {
                let p = fk_result
                    .pose(&self.end_effector)
                    .expect("target frame not found")
                    .translation();
                (*target_pos - p).magnitude()
            }
            IKGoal::Pose(target_pose) => {
                let current = fk_result
                    .pose(&self.end_effector)
                    .expect("target frame not found");
                compute_pose_error(current, target_pose).magnitude()
            }
        };

        Ok(IKResult::max_iterations(
            q.as_slice().to_vec(),
            self.max_iters,
            final_error,
            error_history,
        ))
    }
}
