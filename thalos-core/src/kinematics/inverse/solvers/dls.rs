use crate::kinematics::forward::ForwardKinematics;
use crate::kinematics::inverse::{
    IKConfig, IKSolver,
    error::IkError,
    result::IKResult,
    solver::{IKGoal, compute_pose_error},
};
use crate::kinematics::jacobian::{GeometricJacobian, JacobianSolver};
use crate::robot::joint::{JointKind, JointLimits};
use crate::robot::serial_chain::SerialChain;
use crate::spatial::frame::FrameId;
use thalos_math::algebra::{DynamicMatrix, DynamicVector, vector_to_dynamic};

pub struct DampedLeastSquaresSolver {
    jacobian: GeometricJacobian,
    fk: ForwardKinematics,
    end_effector: FrameId,
    max_iters: usize,
    tolerance: f64,
    lambda: f64,
    track_history: bool,
}

impl DampedLeastSquaresSolver {
    /// Build the solver from an explicit shared [`IKConfig`] (spec `ik-config`).
    ///
    /// All construction sites (semantic compilation, plan analysis, runtime
    /// execution) pass their `IKConfig` here — one explicit type, no hidden
    /// global default.
    pub fn from_config(fk: ForwardKinematics, end_effector: FrameId, config: IKConfig) -> Self {
        let jacobian = GeometricJacobian::new(fk.clone(), end_effector);
        Self {
            jacobian,
            fk,
            end_effector,
            max_iters: config.max_iterations,
            tolerance: config.tolerance,
            lambda: config.lambda,
            track_history: false,
        }
    }

    /// Legacy positional constructor — delegates to [`Self::from_config`].
    pub fn new(
        fk: ForwardKinematics,
        end_effector: FrameId,
        max_iters: usize,
        tolerance: f64,
        lambda: f64,
    ) -> Self {
        Self::from_config(
            fk,
            end_effector,
            IKConfig {
                max_iterations: max_iters,
                tolerance,
                lambda,
            },
        )
    }

    /// Habilita el registro del historial de error por iteración.
    pub fn with_history(mut self, enabled: bool) -> Self {
        self.track_history = enabled;
        self
    }
}

impl IKSolver for DampedLeastSquaresSolver {
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

        let lambda_sq = self.lambda * self.lambda;

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

            // Extract error vector and Jacobian matrix based on goal type
            let ee_pose = fk_result
                .pose(&self.end_effector)
                .expect("target frame not found in FK result");
            let (error_vec, j_mat, magnitude) = match &goal {
                IKGoal::Position(target_pos) => {
                    let p = ee_pose.translation();
                    let error = *target_pos - p;
                    let mag = error.magnitude();
                    let j_lin = jacobian.linear().clone_owned();
                    (vector_to_dynamic(error), j_lin, mag)
                }
                IKGoal::Pose(target_pose) => {
                    let error = compute_pose_error(ee_pose, target_pose);
                    let mag = error.magnitude();
                    let j_full = jacobian.full();
                    (error, j_full, mag)
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

            // A = J · Jᵀ  (n_dof × n_dof)
            // A_damped = A + λ² · I
            let n_dof = j_mat.nrows();
            let identity = DynamicMatrix::identity(n_dof, n_dof);
            let a = &j_mat * j_mat.transpose();
            let a_damped = a + lambda_sq * identity;

            let inv = match a_damped.try_inverse() {
                Some(inv) => inv,
                None => {
                    return Ok(IKResult::max_iterations(
                        q.as_slice().to_vec(),
                        iteration + 1,
                        magnitude,
                        error_history,
                    ));
                }
            };

            // Δq = Jᵀ · inv(A_damped) · e
            let dq = j_mat.transpose() * (inv * error_vec);
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
