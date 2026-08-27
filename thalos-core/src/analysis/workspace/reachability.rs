use std::fmt;

/// Outcome of a reachability query.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reachability {
    /// The point is within the specified tolerance of at least one sample.
    Reachable,
    /// The point is farther than tolerance from every sample.
    OutOfWorkspace {
        /// Minimum Euclidean distance from the query point to any sample position.
        nearest_distance: f64,
    },
}

impl fmt::Display for Reachability {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Reachable => write!(f, "reachable"),
            Self::OutOfWorkspace { nearest_distance } => {
                write!(
                    f,
                    "out of workspace (nearest distance: {})",
                    nearest_distance
                )
            }
        }
    }
}
