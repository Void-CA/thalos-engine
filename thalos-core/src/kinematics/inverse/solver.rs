use crate::spatial::pose::Pose;
use crate::robot::serial_chain::SerialChain;
use thalos_math::DynamicVector;
use thalos_math::{UnitQuaternion, Vector3, orientation_error};

use super::error::IkError;
use super::result::IKResult;

// ─── IK Goal ──────────────────────────────────────────────────────────

/// Objetivo del solucionador de cinemática inversa.
///
/// - [`Position`](IKGoal::Position): solo posición del end-effector.
/// - [`Pose`](IKGoal::Pose): posición **y** orientación completas.
#[derive(Debug, Clone)]
pub enum IKGoal {
    Position(Vector3),
    Pose(Pose),
}

/// Error de orientación 3-DOF a partir de la rotación relativa entre la
/// orientación actual y la deseada.
///
/// Delega a [`orientation_error`] de `thalos-math`, que usa la exponencial
/// logarítmica exacta `log(q_target · q_current⁻¹)` en lugar de la
/// aproximación `2·imag(q_rel)`. Para errores pequeños (típicos en IK
/// iterativo) ambas son equivalentes.
fn orientation_error_3d(target_rot: &UnitQuaternion, current_rot: &UnitQuaternion) -> Vector3 {
    orientation_error(target_rot, current_rot)
}

/// Error completo 6-DOF para pose: [error_posición (3), error_orientación (3)].
pub fn compute_pose_error(current: &Pose, target: &Pose) -> DynamicVector {
    let pos_error = target.translation() - current.translation();

    let r_cur = current.transform().rotation;
    let r_target = target.transform().rotation;
    let orient_error = orientation_error_3d(&r_target, &r_cur);

    let mut error = DynamicVector::zeros(6);
    error[0] = pos_error.x;
    error[1] = pos_error.y;
    error[2] = pos_error.z;
    error[3] = orient_error.x;
    error[4] = orient_error.y;
    error[5] = orient_error.z;
    error
}

// ─── IKSolver trait ───────────────────────────────────────────────────

pub trait IKSolver: Send + Sync {
    fn solve(&self, q0: &[f64], goal: IKGoal) -> Result<IKResult, IkError>;

    /// Optional access to the robot chain the solver operates on.
    ///
    /// Solvers built over a [`ForwardKinematics`] (e.g.
    /// `DampedLeastSquaresSolver`) expose their chain so callers — like the
    /// availability verifier in the planner — can recompile and re-analyze
    /// edited programs with the SAME kinematic model. The default is `None`
    /// (mock solvers without a robot); additive and non-breaking.
    fn robot(&self) -> Option<&SerialChain> {
        None
    }
}
