use std::cmp::Ordering;

/// Qualitative precision level for operator decisions.
///
/// Coexists with float tolerances in `OperationConstraints`:
/// - `PrecisionLevel` for qualitative operator decisions (allow/disallow behavior)
/// - `f64` tolerances for physical constraint enforcement
///
/// Ordering (from most to least precise):
/// Critical > High > Normal > None
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PrecisionLevel {
    /// Sub-mm tolerance (Pick, Inspection).
    Critical,
    /// Mm-level (Place, Process).
    High,
    /// Cm-level (Transit, free motion).
    Normal,
    /// Unconstrained.
    None,
}

impl PrecisionLevel {
    /// Returns `true` if this level is at least as strict as `other`.
    pub fn is_at_least(&self, other: PrecisionLevel) -> bool {
        self.partial_cmp(&other) == Some(Ordering::Greater) || self == &other
    }
}

impl PartialOrd for PrecisionLevel {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PrecisionLevel {
    fn cmp(&self, other: &Self) -> Ordering {
        fn rank(level: PrecisionLevel) -> u8 {
            match level {
                PrecisionLevel::Critical => 0,
                PrecisionLevel::High => 1,
                PrecisionLevel::Normal => 2,
                PrecisionLevel::None => 3,
            }
        }
        rank(*self).cmp(&rank(*other)).reverse()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Ordering tests ────────────────────────────────────

    #[test]
    fn critical_greater_than_high() {
        assert!(PrecisionLevel::Critical > PrecisionLevel::High);
    }

    #[test]
    fn high_greater_than_normal() {
        assert!(PrecisionLevel::High > PrecisionLevel::Normal);
    }

    #[test]
    fn normal_greater_than_none() {
        assert!(PrecisionLevel::Normal > PrecisionLevel::None);
    }

    #[test]
    fn critical_greatest_overall() {
        assert!(PrecisionLevel::Critical > PrecisionLevel::High);
        assert!(PrecisionLevel::Critical > PrecisionLevel::Normal);
        assert!(PrecisionLevel::Critical > PrecisionLevel::None);
    }

    #[test]
    fn none_least_overall() {
        assert!(PrecisionLevel::None < PrecisionLevel::Normal);
        assert!(PrecisionLevel::None < PrecisionLevel::High);
        assert!(PrecisionLevel::None < PrecisionLevel::Critical);
    }

    #[test]
    fn same_levels_are_equal() {
        assert_eq!(PrecisionLevel::Critical, PrecisionLevel::Critical);
        assert_eq!(PrecisionLevel::High, PrecisionLevel::High);
        assert_eq!(PrecisionLevel::Normal, PrecisionLevel::Normal);
        assert_eq!(PrecisionLevel::None, PrecisionLevel::None);
    }

    // ── is_at_least tests ─────────────────────────────────

    #[test]
    fn critical_is_at_least_high() {
        assert!(PrecisionLevel::Critical.is_at_least(PrecisionLevel::High));
    }

    #[test]
    fn critical_is_at_least_critical() {
        assert!(PrecisionLevel::Critical.is_at_least(PrecisionLevel::Critical));
    }

    #[test]
    fn normal_is_not_at_least_high() {
        assert!(!PrecisionLevel::Normal.is_at_least(PrecisionLevel::High));
    }

    #[test]
    fn none_is_not_at_least_normal() {
        assert!(!PrecisionLevel::None.is_at_least(PrecisionLevel::Normal));
    }

    #[test]
    fn high_is_at_least_none() {
        assert!(PrecisionLevel::High.is_at_least(PrecisionLevel::None));
    }
}
