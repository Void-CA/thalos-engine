//! Tier-1 kinematics intent: cartesian targets and inverse kinematics.
//!
//! The consumer expresses "what target" and receives a [`IkSolution`]; the
//! solver (`DampedLeastSquaresSolver`), the chain, the forward kinematics and
//! the config never appear. Advanced control is Tier 2 via [`IkRequest`].

use thalos_core::spatial::pose::Pose;
use thalos_math::{UnitQuaternion, Vector3};

/// A cartesian target: a position, or a full pose.
///
/// This is the Tier-1 goal type, shared by linear motion and inverse
/// kinematics. It is deliberately **not** a re-export of the Tier 3 `IKGoal`, so
/// the mechanism beneath it can evolve without breaking this surface.
///
/// Both forms are expressed the same way — no `FrameId` is needed to build a
/// target:
///
/// ```
/// # use thalos_api::intent::CartesianTarget;
/// # use thalos_api::math::UnitQuaternion;
/// let position = CartesianTarget::position(0.4, 0.2, 0.3);
/// let pose = CartesianTarget::pose(0.4, 0.2, 0.3, UnitQuaternion::identity());
/// ```
#[derive(Debug, Clone)]
pub enum CartesianTarget {
    /// End-effector position in world coordinates.
    Position(Vector3),
    /// Full end-effector pose (position + orientation).
    Pose(Pose),
}

impl CartesianTarget {
    /// World position target.
    pub fn position(x: f64, y: f64, z: f64) -> Self {
        Self::Position(Vector3::new(x, y, z))
    }

    /// World pose target. Orientation is a unit quaternion; no frame bookkeeping
    /// is required at this level.
    pub fn pose(x: f64, y: f64, z: f64, orientation: UnitQuaternion) -> Self {
        Self::Pose(Pose::from_translation_rotation(
            Vector3::new(x, y, z),
            orientation,
        ))
    }
}

impl From<Vector3> for CartesianTarget {
    fn from(position: Vector3) -> Self {
        Self::Position(position)
    }
}

impl From<Pose> for CartesianTarget {
    fn from(pose: Pose) -> Self {
        Self::Pose(pose)
    }
}

/// The result of an inverse-kinematics request (Tier 1).
///
/// A useful value, not a renamed `IKResult`: it carries the joints and the
/// solver's own verdict, while hiding the Tier 3 `IKResult`/`IKStatus` types.
/// A solution that did not converge is still returned (with `converged() ==
/// false` and the best-effort joints) — IK is the question, not a precondition.
#[derive(Debug, Clone)]
pub struct IkSolution {
    joints: Vec<f64>,
    converged: bool,
    iterations: usize,
    final_error: f64,
}

impl IkSolution {
    pub(crate) fn new(
        joints: Vec<f64>,
        converged: bool,
        iterations: usize,
        final_error: f64,
    ) -> Self {
        Self {
            joints,
            converged,
            iterations,
            final_error,
        }
    }

    /// The solution joint configuration (best effort when not converged).
    pub fn joints(&self) -> &[f64] {
        &self.joints
    }

    /// Consume the solution, yielding the joint configuration.
    pub fn into_joints(self) -> Vec<f64> {
        self.joints
    }

    /// Whether the solver reached the tolerance.
    pub fn converged(&self) -> bool {
        self.converged
    }

    /// Iterations the solver performed.
    pub fn iterations(&self) -> usize {
        self.iterations
    }

    /// Final residual error magnitude.
    pub fn final_error(&self) -> f64 {
        self.final_error
    }
}

/// A typed inverse-kinematics request (Tier 2): a target plus optional seed and
/// solver tolerances. Field overrides apply on top of the engine defaults, so
/// setting one never resets the others.
#[derive(Debug, Clone)]
pub struct IkRequest {
    target: CartesianTarget,
    seed: Option<Vec<f64>>,
    tolerance: Option<f64>,
    max_iterations: Option<usize>,
    lambda: Option<f64>,
}

impl IkRequest {
    /// Start a request for a cartesian target.
    pub fn new(target: impl Into<CartesianTarget>) -> Self {
        Self {
            target: target.into(),
            seed: None,
            tolerance: None,
            max_iterations: None,
            lambda: None,
        }
    }

    /// Initial joint configuration (warm start). Defaults to zeros.
    pub fn with_seed(mut self, seed: Vec<f64>) -> Self {
        self.seed = Some(seed);
        self
    }

    /// Override only the convergence tolerance.
    pub fn with_tolerance(mut self, tolerance: f64) -> Self {
        self.tolerance = Some(tolerance);
        self
    }

    /// Override only the maximum iteration count.
    pub fn with_max_iterations(mut self, max_iterations: usize) -> Self {
        self.max_iterations = Some(max_iterations);
        self
    }

    /// Override only the damping factor (DLS λ).
    pub fn with_lambda(mut self, lambda: f64) -> Self {
        self.lambda = Some(lambda);
        self
    }

    pub(crate) fn target(&self) -> &CartesianTarget {
        &self.target
    }

    pub(crate) fn seed(&self) -> Option<&[f64]> {
        self.seed.as_deref()
    }

    pub(crate) fn tolerance(&self) -> Option<f64> {
        self.tolerance
    }

    pub(crate) fn max_iterations(&self) -> Option<usize> {
        self.max_iterations
    }

    pub(crate) fn lambda(&self) -> Option<f64> {
        self.lambda
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_and_pose_are_two_forms_of_the_same_target() {
        let position = CartesianTarget::position(0.4, 0.2, 0.3);
        let pose = CartesianTarget::pose(0.4, 0.2, 0.3, UnitQuaternion::identity());
        assert!(matches!(position, CartesianTarget::Position(_)));
        assert!(matches!(pose, CartesianTarget::Pose(_)));
    }

    #[test]
    fn a_pose_target_needs_no_frame_id() {
        // Builds a full pose with only numbers + a quaternion.
        let _ = CartesianTarget::pose(1.0, 2.0, 3.0, UnitQuaternion::identity());
    }
}
