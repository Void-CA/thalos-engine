//! Per-sample singularity data and the aggregated workspace analysis.

use crate::kinematics::jacobian::SingularityReport;
use thalos_math::Vector3;

use super::config::SingularityConfig;
use super::metrics::SingularityMetrics;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SingularityState {
    Normal,
    NearSingular,
    Singular,
}

#[derive(Debug, Clone)]
pub struct SingularitySample {
    pub q: Vec<f64>,
    pub position: Vector3,
    pub analysis: SingularityReport,
    pub state: SingularityState,
}
#[derive(Debug, Clone)]
pub struct SingularityAnalysis {
    pub samples: Vec<SingularitySample>,
    pub metrics: SingularityMetrics,
}

// ─── Internal helpers ───────────────────────────────────────────────
fn classify(report: &SingularityReport, config: &SingularityConfig) -> SingularityState {
    // Singular: rank-deficient or infinite condition number
    if report.condition_number.is_infinite() || report.rank < 2 {
        return SingularityState::Singular;
    }

    // Near-singular: elevated condition number
    if report.condition_number > config.near_singular_condition_threshold {
        return SingularityState::NearSingular;
    }

    SingularityState::Normal
}
fn aggregate(samples: &[SingularitySample]) -> SingularityMetrics {
    let total = samples.len();
    let mut singular = 0;
    let mut near = 0;
    let mut normal = 0;
    let mut sum_cond = 0.0;
    let mut min_cond = f64::MAX;
    let mut max_cond = 0.0_f64;
    let mut sum_sigma_min = 0.0;

    for s in samples {
        match s.state {
            SingularityState::Singular => singular += 1,
            SingularityState::NearSingular => near += 1,
            SingularityState::Normal => normal += 1,
        }

        let cond = s.analysis.condition_number;
        let sigma_min = s.analysis.singular_values.last().copied().unwrap_or(0.0);

        // Only accumulate finite condition numbers for average
        if cond.is_finite() {
            sum_cond += cond;
            if cond < min_cond {
                min_cond = cond;
            }
            if cond > max_cond {
                max_cond = cond;
            }
        } else {
            // Singular → max condition number is infinity
            max_cond = f64::INFINITY;
        }

        sum_sigma_min += sigma_min;
    }

    let finite_count = samples
        .iter()
        .filter(|s| s.analysis.condition_number.is_finite())
        .count();

    let avg_cond = if finite_count > 0 {
        sum_cond / finite_count as f64
    } else {
        f64::INFINITY
    };

    let avg_sigma_min = if total > 0 {
        sum_sigma_min / total as f64
    } else {
        0.0
    };

    SingularityMetrics {
        total_samples: total,
        singular_count: singular,
        near_singular_count: near,
        normal_count: normal,
        avg_condition_number: avg_cond,
        min_condition_number: if min_cond == f64::MAX { 0.0 } else { min_cond },
        max_condition_number: max_cond,
        avg_sigma_min,
    }
}

// ─── Constructor ────────────────────────────────────────────────────

impl SingularityAnalysis {
    pub fn from_samples(samples: Vec<SingularitySample>) -> Self {
        let metrics = aggregate(&samples);
        Self { samples, metrics }
    }

    pub fn classify_report(
        report: &SingularityReport,
        config: &SingularityConfig,
    ) -> SingularityState {
        classify(report, config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::jacobian::SingularityReport;
    use thalos_math::Vector3;

    fn singular_report() -> SingularityReport {
        SingularityReport {
            det_jtj: 0.0,
            condition_number: f64::INFINITY,
            rank: 1,
            singular_values: vec![2.0, 0.0],
        }
    }

    fn healthy_report() -> SingularityReport {
        SingularityReport {
            det_jtj: 16.0,
            condition_number: 2.0,
            rank: 2,
            singular_values: vec![4.0, 2.0],
        }
    }

    fn near_singular_report() -> SingularityReport {
        SingularityReport {
            det_jtj: 0.01,
            condition_number: 150.0,
            rank: 2,
            singular_values: vec![5.0, 0.033],
        }
    }

    #[test]
    fn classify_singular() {
        let config = SingularityConfig::default();
        let state = SingularityAnalysis::classify_report(&singular_report(), &config);
        assert_eq!(state, SingularityState::Singular);
    }

    #[test]
    fn classify_normal() {
        let config = SingularityConfig::default();
        let state = SingularityAnalysis::classify_report(&healthy_report(), &config);
        assert_eq!(state, SingularityState::Normal);
    }

    #[test]
    fn classify_near_singular() {
        let config = SingularityConfig::default();
        let state = SingularityAnalysis::classify_report(&near_singular_report(), &config);
        assert_eq!(state, SingularityState::NearSingular);
    }

    #[test]
    fn aggregate_metrics_counts() {
        let samples = vec![
            SingularitySample {
                q: vec![0.0, 0.0],
                position: Vector3::new(2.0, 0.0, 0.0),
                analysis: singular_report(),
                state: SingularityState::Singular,
            },
            SingularitySample {
                q: vec![1.0, 0.5],
                position: Vector3::new(1.5, 0.8, 0.0),
                analysis: healthy_report(),
                state: SingularityState::Normal,
            },
            SingularitySample {
                q: vec![0.5, 1.0],
                position: Vector3::new(0.8, 1.2, 0.0),
                analysis: near_singular_report(),
                state: SingularityState::NearSingular,
            },
        ];

        let analysis = SingularityAnalysis::from_samples(samples);

        assert_eq!(analysis.metrics.total_samples, 3);
        assert_eq!(analysis.metrics.singular_count, 1);
        assert_eq!(analysis.metrics.normal_count, 1);
        assert_eq!(analysis.metrics.near_singular_count, 1);
    }

    #[test]
    fn aggregate_metrics_condition_number() {
        let samples = vec![
            SingularitySample {
                q: vec![1.0, 0.5],
                position: Vector3::new(1.5, 0.8, 0.0),
                analysis: healthy_report(),
                state: SingularityState::Normal,
            },
            SingularitySample {
                q: vec![0.5, 1.0],
                position: Vector3::new(0.8, 1.2, 0.0),
                analysis: near_singular_report(),
                state: SingularityState::NearSingular,
            },
        ];

        let analysis = SingularityAnalysis::from_samples(samples);

        // avg_cond = (2.0 + 150.0) / 2 = 76.0
        assert!((analysis.metrics.avg_condition_number - 76.0).abs() < 1e-10);
        assert!((analysis.metrics.min_condition_number - 2.0).abs() < 1e-10);
        assert!((analysis.metrics.max_condition_number - 150.0).abs() < 1e-10);
    }
}
