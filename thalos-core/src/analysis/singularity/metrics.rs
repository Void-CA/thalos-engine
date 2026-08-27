//! Aggregated statistics over a workspace singularity analysis.

/// Summary metrics computed from all per-sample singularity results.
#[derive(Debug, Clone, Copy)]
pub struct SingularityMetrics {
    pub total_samples: usize,
    pub singular_count: usize,
    pub near_singular_count: usize,
    pub normal_count: usize,
    pub avg_condition_number: f64,
    pub min_condition_number: f64,
    pub max_condition_number: f64,
    pub avg_sigma_min: f64,
}
