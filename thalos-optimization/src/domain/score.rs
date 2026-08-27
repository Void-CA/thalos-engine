use super::assessment::OperatorScore;

/// Compute the composite score for an operator.
///
/// Formula: `applicability * estimated_improvement / max(cost, 0.01)`
///
/// The cost is clamped to a minimum of 0.01 to avoid division by zero.
pub fn compute_score(applicability: f32, improvement: f32, cost: f32) -> f32 {
    let safe_cost = cost.max(0.01);
    applicability * improvement / safe_cost
}

/// Rank a slice of `OperatorScore`-bearing items by their `composite` score
/// in descending order (highest score first).
pub fn rank_by_score(scores: &mut [OperatorScore]) {
    scores.sort_by(|a, b| {
        b.composite
            .partial_cmp(&a.composite)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compute_score_basic() {
        let score = compute_score(0.8, 0.5, 1.0);
        let expected = 0.8 * 0.5 / 1.0;
        assert!((score - expected).abs() < f32::EPSILON);
    }

    #[test]
    fn compute_score_clamps_zero_cost() {
        let score = compute_score(0.5, 0.5, 0.0);
        let expected = 0.5 * 0.5 / 0.01;
        assert!((score - expected).abs() < f32::EPSILON);
    }

    #[test]
    fn compute_score_zero_applicability() {
        let score = compute_score(0.0, 0.5, 1.0);
        assert_eq!(score, 0.0);
    }

    #[test]
    fn rank_by_score_descending() {
        let mut scores = vec![
            OperatorScore {
                applicability: 0.5,
                estimated_improvement: 0.5,
                estimated_cost: 1.0,
                composite: 0.25,
            },
            OperatorScore {
                applicability: 0.9,
                estimated_improvement: 0.8,
                estimated_cost: 1.0,
                composite: 0.72,
            },
            OperatorScore {
                applicability: 0.3,
                estimated_improvement: 0.3,
                estimated_cost: 2.0,
                composite: 0.045,
            },
        ];

        rank_by_score(&mut scores);
        assert!(scores[0].composite >= scores[1].composite);
        assert!(scores[1].composite >= scores[2].composite);
    }
}
