/// Aggregated manipulability statistics over a workspace.
#[derive(Debug, Clone, Copy)]
pub struct ManipulabilityMetrics {
    pub total_samples: usize,
    pub avg_yoshikawa: f64,
    pub min_yoshikawa: f64,
    pub max_yoshikawa: f64,
    pub avg_isotropy: f64,
    pub min_isotropy: f64,
    pub max_isotropy: f64,
    /// Chain-side canonical robot-scale normalization factor (`L_ref`,
    /// meters) — the reference dimension the normalized measure was
    /// computed against (spec analysis-report-contract "Additive Reference
    /// Dimension on Metrics").
    pub reference_dimension: f64,
    /// Percentile P05 of `normalized_yoshikawa` over the workspace samples
    /// (design "relative_manipulability": the absolute reference floor of
    /// THIS robot's own distribution).
    pub p05: f64,
    /// Median (P50) of `normalized_yoshikawa` over the workspace samples.
    pub p50: f64,
    /// Percentile P95 of `normalized_yoshikawa` over the workspace samples
    /// (the absolute reference ceiling of the distribution).
    pub p95: f64,
    /// Mean of the per-sample `relative_manipulability` scores — the
    /// percentile rank of each configuration within the robot's own
    /// P05–P95 window, clamped to [0, 1].
    pub avg_relative: f64,
}
