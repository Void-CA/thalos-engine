//! Configuration thresholds for singularity classification.

/// Thresholds that control when a sample is classified as
/// `NearSingular` vs `Normal` based on its condition number.
#[derive(Debug, Clone, Copy)]
pub struct SingularityConfig {
    pub near_singular_condition_threshold: f64,
}

impl Default for SingularityConfig {
    fn default() -> Self {
        Self {
            near_singular_condition_threshold: 100.0,
        }
    }
}
