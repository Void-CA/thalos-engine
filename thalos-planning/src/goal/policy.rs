use super::types::{
    AssessmentWarning, GoalMetadata, MetricAction, MetricKind, PlanningAssessment,
    PlanningDecision, ThresholdDirection,
};

#[derive(Debug, Clone)]
pub struct PlanningPolicy {
    pub singularity: MetricAction,
    pub manipulability: MetricAction,
}

impl Default for PlanningPolicy {
    fn default() -> Self {
        Self {
            // condition number > 1000 near-singular in practice
            singularity: MetricAction::Warn(1000.0),
            manipulability: MetricAction::Ignore,
        }
    }
}

impl PlanningPolicy {
    pub fn evaluate(&self, metadata: &GoalMetadata) -> PlanningAssessment {
        let mut warnings = Vec::new();
        let mut rejected = false;

        if let Some(ref singularity) = metadata.singularity {
            rejected |= Self::check_metric(
                &self.singularity,
                MetricKind::ConditionNumber,
                ThresholdDirection::HigherIsWorse,
                singularity.condition_number,
                &mut warnings,
            );
        }

        if let Some(ref manip) = metadata.manipulability {
            rejected |= Self::check_metric(
                &self.manipulability,
                MetricKind::YoshikawaManipulability,
                ThresholdDirection::LowerIsWorse,
                manip.yoshikawa,
                &mut warnings,
            );
        }

        let decision = if rejected {
            PlanningDecision::Rejected
        } else if warnings.is_empty() {
            PlanningDecision::Accepted
        } else {
            PlanningDecision::AcceptedWithWarnings
        };

        PlanningAssessment { decision, warnings }
    }

    fn check_metric(
        action: &MetricAction,
        kind: MetricKind,
        direction: ThresholdDirection,
        value: f64,
        warnings: &mut Vec<AssessmentWarning>,
    ) -> bool {
        let (threshold, is_reject) = match action {
            MetricAction::Ignore => return false,
            MetricAction::Warn(t) => (*t, false),
            MetricAction::Reject(t) => (*t, true),
        };

        let triggered = match direction {
            ThresholdDirection::HigherIsWorse => value > threshold,
            ThresholdDirection::LowerIsWorse => value < threshold,
        };

        if triggered {
            warnings.push(AssessmentWarning {
                metric: kind,
                value,
                threshold,
            });
        }

        triggered && is_reject
    }
}
