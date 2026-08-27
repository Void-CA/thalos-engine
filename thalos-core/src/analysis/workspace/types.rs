use crate::models::RobotModel;
use thalos_math::Vector3;

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceSample {
    pub q: Vec<f64>,
    pub position: Vector3,
}

/// Axis-aligned bounding box enclosing all reachable positions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox {
    pub min: Vector3,
    pub max: Vector3,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceMetrics {
    /// Volume of the AABB (`(max - min).x * y * z`). NOT the workspace shape volume.
    pub bounding_volume: f64,
    /// Max Euclidean distance from the origin to any sample position.
    pub max_reach: f64,
    /// Min Euclidean distance from the origin to any sample position.
    pub min_reach: f64,
    /// Arithmetic mean of sample positions. NOT a mass centroid, NOT geometric.
    pub centroid: Vector3,
    /// Number of samples (== `Workspace::samples().len()`).
    pub sample_count: usize,
}

/// Cache key for `Workspace` instances.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct WorkspaceKey {
    pub robot_id: RobotModel,
    pub samples: usize,
    pub seed: u64,
}
