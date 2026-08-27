use crate::kinematics::forward::ForwardKinematics;
use crate::kinematics::jacobian::{Jacobian, JacobianSolver};

use thalos_math::DynamicMatrix;
/// Perturbation step for numerical differentiation.
const JACOBIAN_EPS: f64 = 1e-6;
use crate::spatial::frame::FrameId;

pub struct NumericalJacobian {
    fk: ForwardKinematics,
    end_effector: FrameId,
    epsilon: f64,
}

impl NumericalJacobian {
    pub fn new(fk: ForwardKinematics, end_effector: FrameId) -> Self {
        Self {
            fk,
            end_effector,
            epsilon: JACOBIAN_EPS,
        }
    }

    pub fn with_epsilon(fk: ForwardKinematics, end_effector: FrameId, epsilon: f64) -> Self {
        Self {
            fk,
            end_effector,
            epsilon,
        }
    }

    fn end_effector_position(&self, q: &[f64]) -> [f64; 3] {
        let result = self.fk.evaluate(q);

        let pose = result
            .pose(&self.end_effector)
            .expect("End effector pose not found");

        let t = &pose.transform().translation;

        [t.x, t.y, t.z]
    }
}

impl JacobianSolver for NumericalJacobian {
    fn evaluate(&self, q: &[f64]) -> Jacobian {
        let n = q.len();

        let mut linear = DynamicMatrix::zeros(3, n);

        let angular = DynamicMatrix::zeros(3, n);

        for i in 0..n {
            let mut q_plus = q.to_vec();
            let mut q_minus = q.to_vec();

            q_plus[i] += self.epsilon;
            q_minus[i] -= self.epsilon;

            let p_plus = self.end_effector_position(&q_plus);

            let p_minus = self.end_effector_position(&q_minus);

            for j in 0..3 {
                linear[(j, i)] = (p_plus[j] - p_minus[j]) / (2.0 * self.epsilon);
            }
        }

        Jacobian::new(linear, angular)
    }
}
