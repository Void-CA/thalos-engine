use serde::{Deserialize, Serialize};

/// Unique identifier for a problem region within an analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RegionId(pub usize);

/// Classification of the nature of a problem region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RegionKind {
    Singularity,
    LowManipulability,
    Collision,
    Tracking,
    Velocity,
    Constraint,
}

impl RegionKind {
    pub fn name(&self) -> &'static str {
        match self {
            RegionKind::Singularity => "singularity",
            RegionKind::LowManipulability => "low_manipulability",
            RegionKind::Collision => "collision",
            RegionKind::Tracking => "tracking",
            RegionKind::Velocity => "velocity",
            RegionKind::Constraint => "constraint",
        }
    }
}

/// Aggregate severity of a region, derived from the maximum severity of its findings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum RegionSeverity {
    Info,
    Warning,
    Critical,
}

/// Knowledge evidence for a problem region.
#[derive(Debug, Clone)]
pub struct RegionEvidence {
    pub source: String,
    pub reason: String,
    pub weight: f64,
}

/// Aggregate metrics for a problem region.
#[derive(Debug, Clone)]
pub struct RegionMetrics {
    /// Number of waypoints in the region.
    pub waypoint_count: usize,
    /// Average value of the affected metric.
    pub average_value: Option<f64>,
    /// Minimum observed value.
    pub min_value: Option<f64>,
    /// Maximum observed value.
    pub max_value: Option<f64>,
    /// Number of error-level findings in this region.
    pub error_count: usize,
    /// Number of warning-level findings.
    pub warning_count: usize,
}

impl RegionMetrics {
    /// Merge two adjacent region metrics of the same kind.
    pub fn merge(&self, other: &Self) -> Self {
        let total = self.waypoint_count + other.waypoint_count;
        let weighted_avg = |a: f64, b: f64, a_cnt: usize, b_cnt: usize| -> f64 {
            (a * a_cnt as f64 + b * b_cnt as f64) / total as f64
        };
        Self {
            waypoint_count: total,
            average_value: match (self.average_value, other.average_value) {
                (Some(a), Some(b)) => Some(weighted_avg(
                    a,
                    b,
                    self.waypoint_count,
                    other.waypoint_count,
                )),
                (Some(a), None) => Some(a),
                (None, Some(b)) => Some(b),
                (None, None) => None,
            },
            min_value: match (self.min_value, other.min_value) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (Some(a), None) => Some(a),
                (None, Some(b)) => Some(b),
                (None, None) => None,
            },
            max_value: match (self.max_value, other.max_value) {
                (Some(a), Some(b)) => Some(a.max(b)),
                (Some(a), None) => Some(a),
                (None, Some(b)) => Some(b),
                (None, None) => None,
            },
            error_count: self.error_count + other.error_count,
            warning_count: self.warning_count + other.warning_count,
        }
    }
}

/// Entry and exit boundaries of a problem region.
#[derive(Debug, Clone)]
pub struct RegionBoundary {
    /// Pose at the waypoint before the region starts (None if start of trajectory).
    pub entry_pose: Option<thalos_math::Transform3D>,
    /// Pose at the waypoint after the region ends (None if end of trajectory).
    pub exit_pose: Option<thalos_math::Transform3D>,
}

/// Human-readable explanation of a problem region.
#[derive(Debug, Clone)]
pub struct RegionExplanation {
    /// Root cause of the problem in natural language.
    pub cause: String,
    /// Consequences and impact.
    pub consequence: String,
    /// Suggested repair strategies (strings, not domain types).
    pub recommended_strategies: Vec<String>,
    /// Confidence in detection (0.0..1.0).
    pub confidence: f64,
}

/// A semantic problem region — contiguous range of waypoints with the same root cause.
///
/// # Invariants
/// - `waypoint_range` contains at least one waypoint
/// - Waypoints are contiguous (no gaps)
/// - All findings share the same `kind`
/// - `severity` reflects the maximum severity across all findings in the region
#[derive(Debug, Clone)]
pub struct ProblemRegion {
    pub id: RegionId,
    pub kind: RegionKind,
    pub severity: RegionSeverity,
    pub waypoint_range: std::ops::Range<usize>,
    pub metrics: Option<RegionMetrics>,
    pub boundary: Option<RegionBoundary>,
    pub explanation: Option<RegionExplanation>,
    /// Detection confidence (0.0..1.0). Initially 1.0.
    pub confidence: f64,
    /// Structured knowledge evidence supporting the region.
    pub evidence: Vec<RegionEvidence>,
}

impl ProblemRegion {
    /// Create a new region, validating basic invariants.
    ///
    /// # Panics
    /// In debug builds, if the range is empty or not contiguous.
    pub fn new(
        id: RegionId,
        kind: RegionKind,
        severity: RegionSeverity,
        waypoint_range: std::ops::Range<usize>,
    ) -> Self {
        debug_assert!(
            !waypoint_range.is_empty(),
            "ProblemRegion must contain at least one waypoint"
        );
        debug_assert!(
            waypoint_range.start <= waypoint_range.end,
            "ProblemRegion waypoint_range must be ordered"
        );
        Self {
            id,
            kind,
            severity,
            waypoint_range,
            metrics: None,
            boundary: None,
            explanation: None,
            confidence: 1.0,
            evidence: vec![],
        }
    }

    pub fn waypoint_count(&self) -> usize {
        self.waypoint_range.len()
    }
}

/// A problem region enriched with operation-level semantic context.
///
/// Bridges the gap between low-level waypoint ranges and high-level
/// operation intent for frontend consumption.
#[derive(Debug, Clone)]
pub struct SemanticProblem {
    /// The operation that produced this problem, if provenance is available.
    pub operation_id: Option<crate::operation::OperationId>,
    /// The role of the operation within its parent operation, if provenance is available.
    pub role: Option<crate::operation::MotionRole>,
    /// The kind of problem (inherited from the source region).
    pub kind: RegionKind,
    /// The severity of the problem (inherited from the source region).
    pub severity: RegionSeverity,
    /// The waypoint range of the problem (inherited from the source region).
    pub waypoint_range: std::ops::Range<usize>,
}

/// Project a `ProblemRegion` into a `SemanticProblem` by attaching
/// operation-level context from provenance.
///
/// Finds the first `MotionProvenance` entry whose waypoint range overlaps
/// the region's waypoint range, and extracts `operation_id` and `role`.
/// Returns a `SemanticProblem` with `operation_id: None` and `role: None`
/// when no matching provenance is found.
pub fn project_semantic_problem(
    region: &ProblemRegion,
    provenance: &[crate::operation::MotionProvenance],
) -> SemanticProblem {
    let matching = provenance.iter().find(|p| {
        p.waypoint_range.start < region.waypoint_range.end
            && p.waypoint_range.end > region.waypoint_range.start
    });
    SemanticProblem {
        operation_id: matching.map(|p| p.operation_id.clone()),
        role: matching.map(|p| p.role),
        kind: region.kind,
        severity: region.severity,
        waypoint_range: region.waypoint_range.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn region_id_new() {
        let id = RegionId(42);
        assert_eq!(id.0, 42);
    }

    #[test]
    fn region_kind_name() {
        assert_eq!(RegionKind::Singularity.name(), "singularity");
        assert_eq!(RegionKind::Collision.name(), "collision");
        assert_eq!(RegionKind::LowManipulability.name(), "low_manipulability");
        assert_eq!(RegionKind::Tracking.name(), "tracking");
        assert_eq!(RegionKind::Velocity.name(), "velocity");
        assert_eq!(RegionKind::Constraint.name(), "constraint");
    }

    #[test]
    fn region_severity_ordering() {
        assert!(RegionSeverity::Info < RegionSeverity::Warning);
        assert!(RegionSeverity::Warning < RegionSeverity::Critical);
    }

    #[test]
    fn problem_region_new_validates() {
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Critical,
            5..10,
        );
        assert_eq!(region.waypoint_count(), 5);
        assert!(region.evidence.is_empty());
        assert!((region.confidence - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn region_evidence_is_clone() {
        let e = RegionEvidence {
            source: "test".into(),
            reason: "reason".into(),
            weight: 0.5,
        };
        let e2 = e.clone();
        assert_eq!(e.source, e2.source);
    }

    #[test]
    fn region_metrics_merge() {
        let a = RegionMetrics {
            waypoint_count: 3,
            average_value: Some(0.5),
            min_value: Some(0.1),
            max_value: Some(0.9),
            error_count: 1,
            warning_count: 0,
        };
        let b = RegionMetrics {
            waypoint_count: 2,
            average_value: Some(0.8),
            min_value: Some(0.6),
            max_value: Some(1.0),
            error_count: 0,
            warning_count: 1,
        };
        let merged = a.merge(&b);
        assert_eq!(merged.waypoint_count, 5);
        assert!((merged.average_value.unwrap() - 0.62).abs() < 1e-10);
        assert!((merged.min_value.unwrap() - 0.1).abs() < 1e-10);
        assert!((merged.max_value.unwrap() - 1.0).abs() < 1e-10);
        assert_eq!(merged.error_count, 1);
        assert_eq!(merged.warning_count, 1);
    }

    #[test]
    fn region_explanation_fields() {
        let exp = RegionExplanation {
            cause: "singularity".into(),
            consequence: "high joint velocity".into(),
            recommended_strategies: vec!["lift_tcp".into()],
            confidence: 0.9,
        };
        assert_eq!(exp.cause, "singularity");
        assert_eq!(exp.recommended_strategies.len(), 1);
    }

    #[test]
    fn region_boundary_optional_poses() {
        let boundary = RegionBoundary {
            entry_pose: None,
            exit_pose: None,
        };
        assert!(boundary.entry_pose.is_none());
        assert!(boundary.exit_pose.is_none());
    }

    // ── SemanticProblem ───────────────────────────────────

    use crate::operation::{MotionProvenance, MotionRole, OperationId};

    #[test]
    fn semantic_problem_construction() {
        let problem = SemanticProblem {
            operation_id: Some(OperationId("1".to_string())),
            role: Some(MotionRole::Execution),
            kind: RegionKind::Singularity,
            severity: RegionSeverity::Critical,
            waypoint_range: 5..10,
        };

        assert_eq!(problem.operation_id, Some(OperationId("1".to_string())));
        assert_eq!(problem.role, Some(MotionRole::Execution));
        assert_eq!(problem.kind, RegionKind::Singularity);
        assert_eq!(problem.severity, RegionSeverity::Critical);
        assert_eq!(problem.waypoint_range, 5..10);
    }

    #[test]
    fn semantic_problem_without_operation_context() {
        let problem = SemanticProblem {
            operation_id: None,
            role: None,
            kind: RegionKind::Velocity,
            severity: RegionSeverity::Warning,
            waypoint_range: 0..20,
        };

        assert!(problem.operation_id.is_none());
        assert!(problem.role.is_none());
        assert_eq!(problem.kind, RegionKind::Velocity);
        assert_eq!(problem.severity, RegionSeverity::Warning);
        assert_eq!(problem.waypoint_range, 0..20);
    }

    // ── project_semantic_problem ──────────────────────────

    #[test]
    fn project_with_matching_provenance() {
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Critical,
            5..10,
        );
        let provenance = vec![
            MotionProvenance {
                waypoint_range: 0..5,
                operation_id: OperationId("1".to_string()),
                role: MotionRole::Approach,
            },
            MotionProvenance {
                waypoint_range: 5..10,
                operation_id: OperationId("2".to_string()),
                role: MotionRole::Execution,
            },
        ];

        let result = project_semantic_problem(&region, &provenance);

        assert_eq!(result.operation_id, Some(OperationId("2".to_string())));
        assert_eq!(result.role, Some(MotionRole::Execution));
        assert_eq!(result.kind, RegionKind::Singularity);
        assert_eq!(result.severity, RegionSeverity::Critical);
        assert_eq!(result.waypoint_range, 5..10);
    }

    #[test]
    fn project_without_provenance() {
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            0..20,
        );
        let provenance: Vec<MotionProvenance> = vec![];

        let result = project_semantic_problem(&region, &provenance);

        assert!(result.operation_id.is_none());
        assert!(result.role.is_none());
        assert_eq!(result.kind, RegionKind::Velocity);
        assert_eq!(result.severity, RegionSeverity::Warning);
        assert_eq!(result.waypoint_range, 0..20);
    }

    #[test]
    fn project_with_provenance_before_region() {
        // Provenance range ends before region starts → no match
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Collision,
            RegionSeverity::Critical,
            10..15,
        );
        let provenance = vec![MotionProvenance {
            waypoint_range: 0..5,
            operation_id: OperationId("1".to_string()),
            role: MotionRole::Approach,
        }];

        let result = project_semantic_problem(&region, &provenance);

        assert!(result.operation_id.is_none());
        assert!(result.role.is_none());
    }

    #[test]
    fn project_with_provenance_after_region() {
        // Provenance range starts after region ends → no match
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Constraint,
            RegionSeverity::Info,
            0..5,
        );
        let provenance = vec![MotionProvenance {
            waypoint_range: 10..15,
            operation_id: OperationId("1".to_string()),
            role: MotionRole::Transit,
        }];

        let result = project_semantic_problem(&region, &provenance);

        assert!(result.operation_id.is_none());
        assert!(result.role.is_none());
    }

    #[test]
    fn project_with_overlapping_provenance() {
        // Region 3..8, provenance 0..6 → overlap (start < 8 && end > 3)
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::LowManipulability,
            RegionSeverity::Warning,
            3..8,
        );
        let provenance = vec![MotionProvenance {
            waypoint_range: 0..6,
            operation_id: OperationId("42".to_string()),
            role: MotionRole::Execution,
        }];

        let result = project_semantic_problem(&region, &provenance);

        assert_eq!(result.operation_id, Some(OperationId("42".to_string())));
        assert_eq!(result.role, Some(MotionRole::Execution));
    }

    // ── SemanticProblem is cloneable ──────────────────────

    #[test]
    fn semantic_problem_is_cloneable() {
        let problem = SemanticProblem {
            operation_id: Some(OperationId("3".to_string())),
            role: Some(MotionRole::Interaction),
            kind: RegionKind::Tracking,
            severity: RegionSeverity::Warning,
            waypoint_range: 1..4,
        };

        let cloned = problem.clone();
        assert_eq!(cloned.operation_id, problem.operation_id);
        assert_eq!(cloned.role, problem.role);
        assert_eq!(cloned.kind, problem.kind);
        assert_eq!(cloned.severity, problem.severity);
        assert_eq!(cloned.waypoint_range, problem.waypoint_range);
    }
}
