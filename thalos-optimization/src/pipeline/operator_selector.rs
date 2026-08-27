use crate::domain::{OperatorAssessment, OperatorScore, TrajectoryOperator, score};
use thalos_core::{analysis::region::ProblemRegion, evaluation::PlanMetrics};

/// Selects and ranks trajectory operators by their composite score
/// for a given problem region and plan metrics.
///
/// The selector is stateless — it computes assessments on demand
/// using the `TrajectoryOperator` trait methods and the scoring
/// formula from `domain::score`.
#[derive(Debug, Clone)]
pub struct OperatorSelector;

impl OperatorSelector {
    /// Assess a single operator for a given region, computing its
    /// composite score from applicability, estimated improvement,
    /// and estimated cost.
    pub fn assess(
        op: &dyn TrajectoryOperator,
        region: &ProblemRegion,
        metrics: &PlanMetrics,
    ) -> OperatorAssessment {
        let applicability = op.applicability(region);
        let improvement = op.estimate_improvement(region, metrics);
        let cost = op.estimate_cost();
        let composite = score::compute_score(applicability, improvement, cost);

        OperatorAssessment {
            operator_id: op.id(),
            family: op.family(),
            score: OperatorScore {
                applicability,
                estimated_improvement: improvement,
                estimated_cost: cost,
                composite,
            },
            rationale: vec![],
        }
    }

    /// Rank a list of operators by composite score for a given region,
    /// returning them in descending order (highest score first).
    pub fn rank<'a>(
        operators: &[&'a dyn TrajectoryOperator],
        region: &ProblemRegion,
        metrics: &PlanMetrics,
    ) -> Vec<(&'a dyn TrajectoryOperator, OperatorAssessment)> {
        let mut results: Vec<_> = operators
            .iter()
            .map(|op| (*op, Self::assess(*op, region, metrics)))
            .collect();
        results.sort_by(|a, b| {
            b.1.score
                .composite
                .partial_cmp(&a.1.score.composite)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        results
    }
}
