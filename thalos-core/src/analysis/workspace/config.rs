//! Configuration for workspace sampling and queries.

/// User-facing configuration for `WorkspaceSampler::sample` and
/// `Workspace::is_reachable` validation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorkspaceConfig {
    /// Number of joint configurations to sample. MUST be > 0.
    pub samples: usize,
    /// Seed for the deterministic RNG (D4: `StdRng::seed_from_u64`).
    pub seed: u64,
    /// Distance tolerance for reachability queries (in metres).
    /// MUST be `>= 0`. Used in `is_reachable` validation.
    pub tolerance: f64,
}

impl Default for WorkspaceConfig {
    fn default() -> Self {
        Self {
            samples: 10_000,
            seed: 0xDEAD_BEEF,
            tolerance: 1e-3,
        }
    }
}
