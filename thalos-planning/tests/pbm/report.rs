//! Pipeline benchmark report — wraps `OptimizationReport` for test assertions.
//!
//! Provides benchmark-specific types for recording operator activity
//! per region and computing summary statistics.

use std::collections::BTreeSet;

use thalos_optimization::OptimizationReport;

/// Outcome of an operator application in the pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatorStatus {
    /// Operator was accepted (region improved).
    Applied,
    /// Operator was attempted but rejected (below improvement threshold).
    Rejected,
    /// Operator failed with an error.
    Failed,
}

/// Record of a single operator application during pipeline execution.
#[derive(Debug, Clone)]
pub struct OperatorEntry {
    /// Operator identifier (e.g. "joint_centering").
    pub id: String,
    /// Outcome of the operator application.
    pub status: OperatorStatus,
}

/// Summary of a pipeline optimization run for benchmark assertions.
///
/// Wraps the production `OptimizationReport` into a simpler structure
/// that benchmark scenarios can assert against.
#[derive(Debug, Clone)]
pub struct PipelineReport {
    /// Number of distinct problem regions detected (from the analysis phase).
    /// Set externally since `OptimizationReport` only tracks processed regions.
    pub regions_detected: usize,
    /// Ordered list of operator entries, one per optimization step.
    pub operators: Vec<OperatorEntry>,
}

impl PipelineReport {
    /// Build a `PipelineReport` from a production `OptimizationReport`.
    ///
    /// Maps each optimization step to an `OperatorEntry`:
    /// - A step with `operator_id != "none"` and `accepted == true` → `Applied`
    /// - A step with `operator_id != "none"` and `accepted == false` → `Rejected`
    /// - A step with `operator_id == "none"` → `Failed` (all operators failed for that region)
    pub fn from_optimization_report(report: &OptimizationReport, regions_detected: usize) -> Self {
        let operators: Vec<OperatorEntry> = report
            .steps
            .iter()
            .map(|step| {
                let status = if step.accepted {
                    OperatorStatus::Applied
                } else if step.operator_id == "none" {
                    OperatorStatus::Failed
                } else {
                    OperatorStatus::Rejected
                };
                OperatorEntry {
                    id: step.operator_id.to_string(),
                    status,
                }
            })
            .collect();

        Self {
            regions_detected,
            operators,
        }
    }

    /// Number of operators that were successfully applied.
    pub fn applied_count(&self) -> usize {
        self.operators
            .iter()
            .filter(|e| e.status == OperatorStatus::Applied)
            .count()
    }

    /// Number of operators that failed.
    pub fn failed_count(&self) -> usize {
        self.operators
            .iter()
            .filter(|e| e.status == OperatorStatus::Failed)
            .count()
    }

    /// Number of distinct operator IDs that were applied at least once.
    pub fn unique_operators_applied(&self) -> usize {
        self.operators
            .iter()
            .filter(|e| e.status == OperatorStatus::Applied)
            .map(|e| e.id.as_str())
            .collect::<BTreeSet<&str>>()
            .len()
    }

    /// Check that at least `min_operators` were applied across all regions.
    pub fn assert_min_operators_applied(&self, min_operators: usize) {
        assert!(
            self.applied_count() >= min_operators,
            "Expected at least {} applied operators, got {}",
            min_operators,
            self.applied_count()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::analysis::region::RegionId;
    use thalos_optimization::domain::OptimizationStep;

    fn make_step(region_id: usize, operator_id: &'static str, accepted: bool) -> OptimizationStep {
        OptimizationStep {
            region_id: RegionId(region_id),
            operator_id,
            improvement: 0.0,
            accepted,
            iteration: 0,
            rejection_reason: None,
        }
    }

    #[test]
    fn empty_report_has_no_operators() {
        let report = OptimizationReport {
            steps: vec![],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 0);
        assert!(pr.operators.is_empty());
        assert_eq!(pr.applied_count(), 0);
        assert_eq!(pr.failed_count(), 0);
    }

    #[test]
    fn accepted_steps_map_to_applied() {
        let report = OptimizationReport {
            steps: vec![
                make_step(0, "joint_centering", true),
                make_step(1, "retime", true),
            ],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 2);
        assert_eq!(pr.operators.len(), 2);
        assert_eq!(pr.applied_count(), 2);
        assert_eq!(pr.failed_count(), 0);
        assert_eq!(pr.operators[0].status, OperatorStatus::Applied);
        assert_eq!(pr.operators[1].status, OperatorStatus::Applied);
    }

    #[test]
    fn rejected_steps_map_to_rejected() {
        let report = OptimizationReport {
            steps: vec![make_step(0, "joint_centering", false)],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 1);
        assert_eq!(pr.operators.len(), 1);
        assert_eq!(pr.operators[0].status, OperatorStatus::Rejected);
        assert_eq!(pr.applied_count(), 0);
    }

    #[test]
    fn none_operator_id_maps_to_failed() {
        let report = OptimizationReport {
            steps: vec![make_step(0, "none", false)],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 1);
        assert_eq!(pr.operators.len(), 1);
        assert_eq!(pr.operators[0].status, OperatorStatus::Failed);
        assert_eq!(pr.failed_count(), 1);
    }

    #[test]
    fn mixed_steps_map_correctly() {
        let report = OptimizationReport {
            steps: vec![
                make_step(0, "joint_centering", true),
                make_step(1, "retime", false),
                make_step(2, "none", false),
            ],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 3);
        assert_eq!(pr.operators.len(), 3);
        assert_eq!(pr.operators[0].status, OperatorStatus::Applied);
        assert_eq!(pr.operators[1].status, OperatorStatus::Rejected);
        assert_eq!(pr.operators[2].status, OperatorStatus::Failed);
        assert_eq!(pr.applied_count(), 1);
        assert_eq!(pr.failed_count(), 1);
    }

    #[test]
    fn unique_operators_applied_counts_distinct_ids() {
        let report = OptimizationReport {
            steps: vec![
                make_step(0, "joint_centering", true),
                make_step(1, "joint_centering", true),
                make_step(2, "retime", true),
            ],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 3);
        assert_eq!(pr.unique_operators_applied(), 2);
    }

    #[test]
    fn assert_min_operators_applied_passes() {
        let report = OptimizationReport {
            steps: vec![
                make_step(0, "joint_centering", true),
                make_step(1, "retime", true),
            ],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 2);
        pr.assert_min_operators_applied(2); // should not panic
    }

    #[test]
    #[should_panic(expected = "Expected at least 3 applied operators")]
    fn assert_min_operators_applied_fails() {
        let report = OptimizationReport {
            steps: vec![make_step(0, "joint_centering", true)],
            final_trajectory: None,
            total_improvement: 0.0,
        };
        let pr = PipelineReport::from_optimization_report(&report, 1);
        pr.assert_min_operators_applied(3); // should panic
    }
}
