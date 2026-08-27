//! Objective profiles and per-candidate-set normalization (PR2, Phase 3,
//! tasks 3.1 + 3.2; design ADR-2, spec candidate-evaluation "Objective
//! Function J(c)").
//!
//! # J is RELATIVE to the generated candidate set — NOT an absolute score
//!
//! `J(c) = Σ w_i · norm_i(c)` is computed from **per-candidate-set min-max
//! normalization**: `norm(x) = (x − min) / (max − min)` where `min`/`max`
//! are the extrema of THAT component across the candidates under evaluation.
//! Adding a candidate that extends a component's range shifts every norm (and
//! therefore every cost) — `J` has no meaning across different candidate
//! sets. Two runs over different sets are NOT comparable. This is a feature:
//! `J` answers "which candidate best fits this objective *within this
//! alternative set*", not "how good is this plan in absolute terms".
//!
//! Monotonicity (spec "Monotonicity" scenarios): for candidates A, B with
//! `x_A < x_B` on one component and equal on all others, `norm(x_A) ≤
//! norm(x_B)` — and strictly `<` whenever the component's `max > min` over
//! the set (the transform is strictly increasing on `[min, max]`). Tie
//! handling (ADR-2): when `max == min` (every candidate identical on the
//! component) every norm is `0.5` — a neutral contribution that never
//! dominates `J` and avoids division by zero.

/// The objective profiles the evaluator supports (spec "SafetyFirst
/// weights").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectiveProfile {
    /// Safety-first: risk dominates (w_R = 0.5), then duration and
    /// low-manipulability (0.2 each), then path length (0.1).
    SafetyFirst,
}

/// SafetyFirst weights, ordered `[w_R, w_C, w_M, w_L]` — risk, duration,
/// low-manipulability, path length (spec "SafetyFirst weights").
pub const SAFETY_FIRST_WEIGHTS: [f64; 4] = [0.5, 0.2, 0.2, 0.1];

impl ObjectiveProfile {
    /// The component weights for this profile: `[w_risk, w_duration,
    /// w_low_manipulability, w_length]`. All weights are ≥ 0 and Σ w_i = 1
    /// (spec "SafetyFirst weights").
    pub fn weights(&self) -> [f64; 4] {
        match self {
            ObjectiveProfile::SafetyFirst => SAFETY_FIRST_WEIGHTS,
        }
    }
}

/// Epsilon deadband for per-candidate-set normalization (spec
/// candidate-evaluation "Normalization deadband", design ADR "ε placement").
/// Sub-ε differences (e.g., the duration delta 1.38e-5 s between degenerate
/// icebot copies) are treated as tied — all candidates receive 0.5 for that
/// component. ε = 1e-4 is 3+ orders above the observed noise floor (1.38e-5 s)
/// and 3+ orders below genuine deltas (real detours: duration ~3 s, length
/// ~0.7 m). Pinned by `normalize_min_max_deadband_pinned`.
const EPSILON: f64 = 1e-4;

/// Per-candidate-set min-max normalization (design ADR-2).
///
/// `norm(x) = (x − min) / (max − min)` over the given values; when
/// `max − min < EPSILON` (all candidates identical on the component — or
/// sub-ε copies indistinguishable from noise, including a single-candidate
/// set) every value normalizes to `0.5` (neutral). An empty set normalizes
/// to an empty set.
///
/// The result is RELATIVE to the input set: values are only meaningful
/// compared within the set that produced them.
pub fn normalize_min_max(values: &[f64]) -> Vec<f64> {
    let Some(min) = values.iter().copied().reduce(f64::min) else {
        return Vec::new();
    };
    let max = values
        .iter()
        .copied()
        .reduce(f64::max)
        .expect("non-empty set has a max");
    let range = max - min;
    if range < EPSILON {
        // Deadband (ADR-2 + demos-purpose-and-sync): identical or sub-ε
        // values → neutral 0.5 contribution (never a noise-driven gap).
        return vec![0.5; values.len()];
    }
    values.iter().map(|x| (x - min) / range).collect::<Vec<_>>()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── 3.1 — SafetyFirst weights (spec "SafetyFirst weights") ───────────

    #[test]
    fn safety_first_weights_are_0_5_0_2_0_2_0_1() {
        let w = ObjectiveProfile::SafetyFirst.weights();
        assert!(
            (w[0] - 0.5).abs() < 1e-12,
            "w_R (risk) must be 0.5, got {}",
            w[0]
        );
        assert!(
            (w[1] - 0.2).abs() < 1e-12,
            "w_C (duration) must be 0.2, got {}",
            w[1]
        );
        assert!(
            (w[2] - 0.2).abs() < 1e-12,
            "w_M (low-manipulability) must be 0.2, got {}",
            w[2]
        );
        assert!(
            (w[3] - 0.1).abs() < 1e-12,
            "w_L (length) must be 0.1, got {}",
            w[3]
        );
    }

    #[test]
    fn safety_first_weights_are_all_non_negative() {
        let w = ObjectiveProfile::SafetyFirst.weights();
        for (i, wi) in w.iter().enumerate() {
            assert!(*wi >= 0.0, "weight w[{i}] must be ≥ 0, got {wi}");
        }
    }

    #[test]
    fn safety_first_weights_sum_to_one() {
        let w = ObjectiveProfile::SafetyFirst.weights();
        let sum: f64 = w.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-12,
            "Σ w_i must equal 1 (within float tolerance), got {sum}"
        );
    }

    // ── 3.1 — per-candidate-set min-max normalization (ADR-2) ─────────────

    #[test]
    fn min_max_normalization_maps_min_to_zero_max_to_one() {
        let norm = normalize_min_max(&[0.1, 0.7, 0.9]);
        assert!((norm[0] - 0.0).abs() < 1e-12, "min must map to 0");
        assert!((norm[2] - 1.0).abs() < 1e-12, "max must map to 1");
    }

    #[test]
    fn min_max_normalization_scales_intermediate_values_linearly() {
        // norm(3) = (3−1)/(5−1) = 0.5
        let norm = normalize_min_max(&[1.0, 3.0, 5.0]);
        assert!((norm[0] - 0.0).abs() < 1e-12);
        assert!(
            (norm[1] - 0.5).abs() < 1e-12,
            "intermediate value must scale linearly, got {}",
            norm[1]
        );
        assert!((norm[2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn min_max_normalization_tie_returns_neutral_half() {
        // ADR-2 tie handling: max == min (all identical) → 0.5 neutral.
        let norm = normalize_min_max(&[0.7, 0.7, 0.7]);
        for v in &norm {
            assert!(
                (v - 0.5).abs() < 1e-12,
                "tied component must normalize to 0.5, got {v}"
            );
        }
    }

    #[test]
    fn min_max_normalization_is_relative_to_the_given_set() {
        // J is RELATIVE to the generated candidate set: adding a candidate
        // below the old min shifts every norm. norm(0.1) in {0.1, 0.9} is 0;
        // once 0.05 joins the set, norm(0.1) = (0.1−0.05)/(0.9−0.05) > 0.
        let two = normalize_min_max(&[0.1, 0.9]);
        let three = normalize_min_max(&[0.05, 0.1, 0.9]);
        assert!((two[0] - 0.0).abs() < 1e-12);
        let expected = (0.1 - 0.05) / (0.9 - 0.05);
        assert!(
            (three[1] - expected).abs() < 1e-12,
            "norm must be recomputed against the extended set, got {} expected {expected}",
            three[1]
        );
        assert!(
            three[1] > 0.0,
            "the former min must no longer normalize to 0 when a lower candidate joins the set"
        );
    }

    #[test]
    fn min_max_normalization_single_value_set_is_neutral() {
        let norm = normalize_min_max(&[0.42]);
        assert!(
            (norm[0] - 0.5).abs() < 1e-12,
            "a one-candidate set has max == min → 0.5, got {}",
            norm[0]
        );
    }

    #[test]
    fn min_max_normalization_of_empty_set_is_empty() {
        let norm = normalize_min_max(&[]);
        assert!(
            norm.is_empty(),
            "no candidates → no norms (the evaluator guards emptiness before normalizing)"
        );
    }

    // ── demos-purpose-and-sync — ε deadband (spec candidate-evaluation
    //    "Epsilon pinned by test", design ADR "ε placement") ────────────────

    #[test]
    fn normalize_min_max_deadband_pinned() {
        // Observed degeneracy: the icebot AlternateElbow re-solve is a sub-ε
        // copy of Direct (duration delta 1.38e-5 s, manipulability 2.33e-9,
        // length 1.177e-5 m — design "Data Flow (b)"). The deadband MUST tie
        // such sub-ε differences at 0.5 (neutral) so floating-point noise can
        // never produce an O(1) selection gap.
        let sub_epsilon = normalize_min_max(&[18.893269, 18.893255]);
        for v in &sub_epsilon {
            assert!(
                (v - 0.5).abs() < 1e-12,
                "sub-ε duration copies must tie at 0.5 (deadband), got {v}"
            );
        }
        // Genuine deltas stay above the deadband (duration 1e-2 s ≫ ε=1e-4):
        // a real improvement MUST still separate the candidates.
        let genuine = normalize_min_max(&[18.0, 18.01]);
        assert!(
            (genuine[0] - 0.0).abs() < 1e-12,
            "min must map to 0 for a genuine delta, got {}",
            genuine[0]
        );
        assert!(
            (genuine[1] - 1.0).abs() < 1e-12,
            "max must map to 1 for a genuine delta, got {}",
            genuine[1]
        );
    }
}
