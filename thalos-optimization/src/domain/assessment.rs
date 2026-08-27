use super::operator::OperatorFamily;

/// Composite score for an operator applied to a specific region.
#[derive(Debug, Clone)]
pub struct OperatorScore {
    /// How applicable the operator is to the region [0.0, 1.0].
    pub applicability: f32,
    /// Expected improvement [0.0, 1.0].
    pub estimated_improvement: f32,
    /// Estimated computational cost (arbitrary units).
    pub estimated_cost: f32,
    /// Composite score = applicability * improvement / cost.
    pub composite: f32,
}

/// Assessment of an operator for a specific region, including score and rationale.
#[derive(Debug, Clone)]
pub struct OperatorAssessment {
    /// Stable identifier of the operator.
    pub operator_id: &'static str,
    /// Family of the operator.
    pub family: OperatorFamily,
    /// Composite score and sub-scores.
    pub score: OperatorScore,
    /// List of reasons supporting this assessment.
    pub rationale: Vec<Reason>,
}

/// A single reason factor contributing to an operator assessment.
#[derive(Debug, Clone)]
pub struct Reason {
    /// Human-readable factor name (e.g. "region_kind_match", "cost_too_high").
    pub factor: String,
    /// Impact of this factor on the overall assessment (-1.0 to 1.0).
    pub impact: f32,
}
