//! `Workspace`: an immutable value object derived from joint-config samples.
//!
//! A `Workspace` is constructed once from a `Vec<WorkspaceSample>` via
//! [`Workspace::from_samples`] and exposes only getters. The derived
//! `BoundingBox` and `WorkspaceMetrics` are computed in a single O(n) pass
//! at construction time, then cached in the private fields for O(1) access.

use thalos_math::Vector3;

use super::error::WorkspaceError;
use super::reachability::Reachability;
use super::types::{BoundingBox, WorkspaceMetrics, WorkspaceSample};

/// Immutable collection of joint-config samples with derived bounds and
/// metrics. The `Workspace` is the **fundamental dataset** for downstream
/// analysis (singularity map, manipulability map, trajectory validation):
/// it stores both the input (q) and the output (position) of each sample.
///
/// Once constructed, fields are private and never mutate. To share across
/// threads or services, wrap in `Arc<Workspace>`.
///
/// ```compile_fail
/// use thalos_core::analysis::workspace::Workspace;
/// let ws = Workspace::from_samples(vec![]).unwrap();
/// let _ = ws.samples; // private field — must not compile
/// ```
#[derive(Debug, Clone)]
pub struct Workspace {
    samples: Vec<WorkspaceSample>,
    bounds: BoundingBox,
    metrics: WorkspaceMetrics,
}

impl Workspace {
    /// Construct a `Workspace` from a vector of samples, deriving bounds and
    /// metrics (centroid, max/min reach, bounding_volume) in a single O(n)
    /// sequential pass. Rejects empty input.
    pub fn from_samples(samples: Vec<WorkspaceSample>) -> Result<Self, WorkspaceError> {
        if samples.is_empty() {
            return Err(WorkspaceError::EmptyWorkspace);
        }

        // Single-pass: min, max, sum (for centroid), max_reach, min_reach.
        let mut min = samples[0].position;
        let mut max = samples[0].position;
        let mut sum = Vector3::new(0.0, 0.0, 0.0);
        let mut max_reach = 0.0_f64;
        let mut min_reach = f64::INFINITY;

        for s in &samples {
            let p = s.position;
            // min / max
            if p.x < min.x {
                min.x = p.x;
            }
            if p.y < min.y {
                min.y = p.y;
            }
            if p.z < min.z {
                min.z = p.z;
            }
            if p.x > max.x {
                max.x = p.x;
            }
            if p.y > max.y {
                max.y = p.y;
            }
            if p.z > max.z {
                max.z = p.z;
            }
            // sum for centroid
            sum.x += p.x;
            sum.y += p.y;
            sum.z += p.z;
            // reach
            let r = p.magnitude();
            if r > max_reach {
                max_reach = r;
            }
            if r < min_reach {
                min_reach = r;
            }
        }

        let n = samples.len() as f64;
        let centroid = Vector3::new(sum.x / n, sum.y / n, sum.z / n);
        let bounding_volume = (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
        let sample_count = samples.len();

        Ok(Self {
            samples,
            bounds: BoundingBox { min, max },
            metrics: WorkspaceMetrics {
                bounding_volume,
                max_reach,
                min_reach,
                centroid,
                sample_count,
            },
        })
    }

    /// View of the underlying samples (joint configs + positions).
    pub fn samples(&self) -> &[WorkspaceSample] {
        &self.samples
    }

    /// Axis-aligned bounding box enclosing all positions.
    pub fn bounds(&self) -> &BoundingBox {
        &self.bounds
    }

    /// Metrics derived from the position set.
    pub fn metrics(&self) -> &WorkspaceMetrics {
        &self.metrics
    }

    // ─── Reachability query ───────────────────────────────────────────

    /// Check whether a point is reachable (within `tolerance` of any sample).
    ///
    /// Uses linear scan (D1). Returns:
    /// - `Ok(Reachable)` if `min_distance ≤ tolerance`
    /// - `Ok(OutOfWorkspace { nearest_distance })` otherwise
    /// - `Err(InvalidPoint)` if any coordinate is NaN or Infinity
    /// - `Err(InvalidTolerance)` if tolerance is NaN or negative
    pub fn is_reachable(
        &self,
        point: &Vector3,
        tolerance: f64,
    ) -> Result<Reachability, WorkspaceError> {
        // R4: Validate inputs
        validate_point(point)?;
        validate_tolerance(tolerance)?;

        let min_sq = self
            .samples
            .iter()
            .map(|s| {
                let dx = s.position.x - point.x;
                let dy = s.position.y - point.y;
                let dz = s.position.z - point.z;
                dx * dx + dy * dy + dz * dz
            })
            .fold(f64::INFINITY, f64::min);

        let min_dist = min_sq.sqrt();

        if min_dist <= tolerance {
            Ok(Reachability::Reachable)
        } else {
            Ok(Reachability::OutOfWorkspace {
                nearest_distance: min_dist,
            })
        }
    }
}

fn validate_point(point: &Vector3) -> Result<(), WorkspaceError> {
    if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
        return Err(WorkspaceError::InvalidPoint(format!(
            "({}, {}, {})",
            point.x, point.y, point.z
        )));
    }
    Ok(())
}

fn validate_tolerance(tolerance: f64) -> Result<(), WorkspaceError> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(WorkspaceError::InvalidTolerance(tolerance));
    }
    Ok(())
}
