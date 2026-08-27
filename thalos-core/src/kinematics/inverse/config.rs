//! Shared IK solver configuration (spec `ik-config`).
//!
//! Pure data — no runtime dependencies. Semantic compilation, plan analysis,
//! and runtime execution each construct their solver through an explicit
//! [`IKConfig`]; there is no global overridable default that one site could
//! mutate and another observe. Unifying the TYPE does not uniform the VALUES:
//! each site preserves its current configuration (semantic 1000/1e-4/0.1,
//! analysis+runtime 500/1e-6/0.1). Whether values should converge is a
//! separate follow-up decision, out of scope for this change.

/// Explicit solver configuration for `DampedLeastSquaresSolver` (and any
/// future IK solver).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IKConfig {
    /// Maximum iterations before the solver reports `MaxIterations`.
    pub max_iterations: usize,
    /// Convergence threshold: the solver stops when the pose error magnitude
    /// drops below `tolerance`.
    pub tolerance: f64,
    /// Damping factor (the DLS λ — the λ²·I regularization of `J·Jᵀ`).
    pub lambda: f64,
}

impl Default for IKConfig {
    /// Runtime/analysis default (500, 1e-6, 0.1). The semantic site passes its
    /// own preserved values (1000, 1e-4, 0.1) — `Default` is only a
    /// convenience for sites whose values match the runtime/analysis set; it
    /// is never a hidden global that other sites depend on.
    fn default() -> Self {
        Self {
            max_iterations: 500,
            tolerance: 1e-6,
            lambda: 0.1,
        }
    }
}
