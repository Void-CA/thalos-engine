/// A single snapshot of a trajectory: joint positions at a given time.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct TrajectoryPoint {
    pub(crate) joints: Vec<f64>,
    pub(crate) timestamp: f64,
}

impl TrajectoryPoint {
    pub fn new(joints: Vec<f64>, timestamp: f64) -> Self {
        Self { joints, timestamp }
    }

    pub fn joints(&self) -> &[f64] {
        &self.joints
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp
    }

    pub fn into_joints(self) -> Vec<f64> {
        self.joints
    }
}
