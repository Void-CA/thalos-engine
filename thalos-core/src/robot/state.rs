/// The instantaneous kinematic state of a robot: its joint configuration.
///
/// This type exists so that subsystems (planning, simulation, control,
/// temporal analysis) can express "I only need the current joint values"
/// without pulling in the full robot description.
#[derive(Debug, Clone)]
pub struct RobotState {
    pub joints: Vec<f64>,
}

impl RobotState {
    pub fn new(joints: Vec<f64>) -> Self {
        Self { joints }
    }

    /// Convenience constructor: all joints set to zero.
    pub fn zero(dof: usize) -> Self {
        Self {
            joints: vec![0.0; dof],
        }
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.joints
    }
}
