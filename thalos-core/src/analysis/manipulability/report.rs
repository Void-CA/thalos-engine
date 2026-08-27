use crate::kinematics::jacobian::{ManipulabilityReport, SingularityReport};
use thalos_math::Vector3;

use super::metrics::ManipulabilityMetrics;

/// One workspace sample with its derived manipulability metrics.
#[derive(Debug, Clone)]
pub struct ManipulabilitySample {
    pub q: Vec<f64>,
    pub position: Vector3,
    pub singularity: SingularityReport,
    pub manipulability: ManipulabilityReport,
    /// Percentile-based score of THIS configuration relative to the robot's
    /// own `normalized_yoshikawa` distribution (design
    /// "relative_manipulability"): `(w − P05) / (P95 − P05)`, clamped to
    /// [0, 1]. Computed by [`ManipulabilityAnalysis::from_samples`] over the
    /// FULL sample set — `1.0` for degenerate distributions (P95 == P05,
    /// every sample sits at the reference top).
    pub relative_manipulability: f64,
}

/// Aggregated result of a workspace manipulability analysis.
#[derive(Debug, Clone)]
pub struct ManipulabilityAnalysis {
    pub samples: Vec<ManipulabilitySample>,
    pub metrics: ManipulabilityMetrics,
}

fn aggregate(
    samples: &[ManipulabilitySample],
    reference_dimension: f64,
    p05: f64,
    p50: f64,
    p95: f64,
) -> ManipulabilityMetrics {
    let total = samples.len();
    let mut sum_y = 0.0_f64;
    let mut min_y = f64::MAX;
    let mut max_y = 0.0_f64;
    let mut sum_i = 0.0_f64;
    let mut min_i = f64::MAX;
    let mut max_i = 0.0_f64;
    let mut sum_rel = 0.0_f64;

    for s in samples {
        let y = s.manipulability.yoshikawa;
        let i = s.manipulability.isotropy;
        sum_y += y;
        if y < min_y {
            min_y = y;
        }
        if y > max_y {
            max_y = y;
        }
        sum_i += i;
        if i < min_i {
            min_i = i;
        }
        if i > max_i {
            max_i = i;
        }
        if s.relative_manipulability.is_finite() {
            sum_rel += s.relative_manipulability;
        }
    }

    ManipulabilityMetrics {
        total_samples: total,
        avg_yoshikawa: if total > 0 { sum_y / total as f64 } else { 0.0 },
        min_yoshikawa: if min_y == f64::MAX { 0.0 } else { min_y },
        max_yoshikawa: max_y,
        avg_isotropy: if total > 0 { sum_i / total as f64 } else { 0.0 },
        min_isotropy: if min_i == f64::MAX { 0.0 } else { min_i },
        max_isotropy: max_i,
        reference_dimension,
        p05,
        p50,
        p95,
        avg_relative: if total > 0 {
            sum_rel / total as f64
        } else {
            0.0
        },
    }
}

/// Linear-interpolation percentile (numpy-style Type 7) of the samples'
/// `normalized_yoshikawa` — `q` ∈ [0, 1]. Non-finite values are excluded
/// (the analyzer guarantees finite normalized measures); an empty or
/// all-degenerate set floors at 0.0 so `from_samples` still yields a
/// well-defined score.
fn percentile_of(samples: &[ManipulabilitySample], q: f64) -> f64 {
    let mut values: Vec<f64> = samples
        .iter()
        .map(|s| s.manipulability.normalized_yoshikawa)
        .filter(|v| v.is_finite())
        .collect();
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|a, b| a.partial_cmp(b).expect("finite values"));
    if values.len() == 1 {
        return values[0];
    }
    let rank = q * (values.len() - 1) as f64;
    let lo = rank.floor() as usize;
    let hi = rank.ceil() as usize;
    if lo == hi {
        values[lo]
    } else {
        let frac = rank - lo as f64;
        values[lo] + (values[hi] - values[lo]) * frac
    }
}

impl ManipulabilityAnalysis {
    pub fn from_samples(mut samples: Vec<ManipulabilitySample>, reference_dimension: f64) -> Self {
        // Relative score pass (design "relative_manipulability"): stage the
        // FULL sample set first — P05 / P50 / P95 of normalized_yoshikawa —
        // then rank every sample against its own robot's distribution.
        let p05 = percentile_of(&samples, 0.05);
        let p50 = percentile_of(&samples, 0.50);
        let p95 = percentile_of(&samples, 0.95);
        let spread = p95 - p05;

        for s in &mut samples {
            let w = s.manipulability.normalized_yoshikawa;
            s.relative_manipulability = if !w.is_finite() || !spread.is_finite() || spread <= 0.0 {
                // Degenerate distribution (P95 == P05 — e.g. a single sample
                // or a uniform robot): every configuration equals the
                // reference top; no discrimination exists, score 1.0.
                1.0
            } else {
                ((w - p05) / spread).clamp(0.0, 1.0)
            };
        }

        let metrics = aggregate(&samples, reference_dimension, p05, p50, p95);
        Self { samples, metrics }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::jacobian::SingularityReport;

    fn sample(yoshikawa: f64, isotropy: f64) -> ManipulabilitySample {
        ManipulabilitySample {
            q: vec![],
            position: Vector3::new(0.0, 0.0, 0.0),
            singularity: SingularityReport {
                det_jtj: 0.0,
                condition_number: 1.0,
                rank: 1,
                singular_values: vec![],
            },
            manipulability: ManipulabilityReport {
                yoshikawa,
                isotropy,
                normalized_yoshikawa: 0.0,
                ..Default::default()
            },
            relative_manipulability: 0.0,
        }
    }

    fn sample_normalized(normalized: f64, yoshikawa: f64, isotropy: f64) -> ManipulabilitySample {
        ManipulabilitySample {
            q: vec![],
            position: Vector3::new(0.0, 0.0, 0.0),
            singularity: SingularityReport {
                det_jtj: 0.0,
                condition_number: 1.0,
                rank: 1,
                singular_values: vec![],
            },
            manipulability: ManipulabilityReport {
                yoshikawa,
                isotropy,
                normalized_yoshikawa: normalized,
                ..Default::default()
            },
            relative_manipulability: 0.0,
        }
    }

    #[test]
    fn aggregate_single() {
        let a = ManipulabilityAnalysis::from_samples(vec![sample(10.0, 0.5)], 2.3);
        assert_eq!(a.metrics.total_samples, 1);
        assert!((a.metrics.avg_yoshikawa - 10.0).abs() < 1e-12);
        assert!((a.metrics.avg_isotropy - 0.5).abs() < 1e-12);
    }

    #[test]
    fn aggregate_multiple() {
        let samples = vec![sample(10.0, 0.9), sample(2.0, 0.1), sample(6.0, 0.5)];
        let a = ManipulabilityAnalysis::from_samples(samples, 1.8);
        assert_eq!(a.metrics.total_samples, 3);
        assert!((a.metrics.avg_yoshikawa - 6.0).abs() < 1e-12);
        assert!((a.metrics.min_yoshikawa - 2.0).abs() < 1e-12);
        assert!((a.metrics.max_yoshikawa - 10.0).abs() < 1e-12);
        assert!((a.metrics.avg_isotropy - 0.5).abs() < 1e-12);
        assert!((a.metrics.min_isotropy - 0.1).abs() < 1e-12);
        assert!((a.metrics.max_isotropy - 0.9).abs() < 1e-12);
    }

    #[test]
    fn metrics_expose_reference_dimension() {
        // Task 2.2 (spec analysis-report-contract "Additive Reference
        // Dimension on Metrics"): the aggregate metrics carry the chain-side
        // L_ref so consumers (workspace DTO, dashboard) can expose it.
        let samples = vec![sample(4.0, 0.5), sample(9.0, 0.5)];
        let a = ManipulabilityAnalysis::from_samples(samples, 2.3);
        assert!(
            (a.metrics.reference_dimension - 2.3).abs() < 1e-12,
            "metrics must expose reference_dimension = 2.3"
        );
    }

    #[test]
    fn relative_manipulability_is_percentile_staged_and_clamped() {
        // Design "relative_manipulability": normalized values
        // [0.0, 0.9, 0.1, 0.5, 1.0] → sorted [0, 0.1, 0.5, 0.9, 1.0].
        //   P05 = 0.02, P50 = 0.5, P95 = 0.98 (linear interpolation, n=5)
        //   spread = 0.96; score = (w − P05) / spread, clamped to [0, 1].
        let samples = vec![
            sample_normalized(0.0, 1.0, 0.5),
            sample_normalized(0.9, 1.0, 0.5),
            sample_normalized(0.1, 1.0, 0.5),
            sample_normalized(0.5, 1.0, 0.5),
            sample_normalized(1.0, 1.0, 0.5),
        ];
        let a = ManipulabilityAnalysis::from_samples(samples, 1.0);

        assert!((a.metrics.p05 - 0.02).abs() < 1e-12, "P05 = 0.02");
        assert!((a.metrics.p50 - 0.5).abs() < 1e-12, "P50 = 0.5");
        assert!((a.metrics.p95 - 0.98).abs() < 1e-12, "P95 = 0.98");

        // Every score lives in [0, 1] — clamped on both ends.
        for s in &a.samples {
            assert!(
                (0.0..=1.0).contains(&s.relative_manipulability),
                "score {} must be clamped to [0, 1]",
                s.relative_manipulability
            );
        }
        assert!(
            (a.samples[0].relative_manipulability - 0.0).abs() < 1e-12,
            "w=0.0 below P05 → clamped to 0.0"
        );
        assert!(
            (a.samples[4].relative_manipulability - 1.0).abs() < 1e-12,
            "w=1.0 above P95 → clamped to 1.0"
        );
        assert!(
            (a.samples[3].relative_manipulability - 0.5).abs() < 1e-12,
            "w=0.5 exactly at median → 0.5"
        );
        // avg_relative = (0 + 0.9167 + 0.0833 + 0.5 + 1) / 5 = 0.5
        assert!(
            (a.metrics.avg_relative - 0.5).abs() < 1e-9,
            "avg_relative must be the mean of the per-sample scores"
        );
    }

    #[test]
    fn relative_manipulability_degenerate_distribution_scores_one() {
        // Zero spread (single sample, or every normalized value equal):
        // P95 == P05 → every configuration sits at the reference top and no
        // score is fabricated outside [0, 1].
        let single =
            ManipulabilityAnalysis::from_samples(vec![sample_normalized(0.7, 1.0, 0.5)], 1.0);
        assert_eq!(single.samples[0].relative_manipulability, 1.0);
        assert_eq!(single.metrics.avg_relative, 1.0);
        assert_eq!(single.metrics.p05, 0.7);
        assert_eq!(single.metrics.p50, 0.7);
        assert_eq!(single.metrics.p95, 0.7);

        let flat = ManipulabilityAnalysis::from_samples(
            vec![
                sample_normalized(0.2, 1.0, 0.5),
                sample_normalized(0.2, 1.0, 0.5),
            ],
            1.0,
        );
        assert_eq!(flat.samples[0].relative_manipulability, 1.0);
        assert_eq!(flat.metrics.avg_relative, 1.0);
    }

    #[test]
    fn empty_analysis_yields_zero_metrics_without_panicking() {
        let a = ManipulabilityAnalysis::from_samples(vec![], 1.0);
        assert_eq!(a.metrics.total_samples, 0);
        assert_eq!(a.metrics.avg_relative, 0.0);
        assert_eq!(a.metrics.p05, 0.0);
        assert_eq!(a.metrics.p95, 0.0);
    }
}
