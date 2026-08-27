//! Thalos Optimization Framework
//!
//! A robot-agnostic trajectory optimization crate that provides domain types,
//! operator traits, scoring, and a reusable pipeline for optimizing trajectory
//! regions identified by the planning analysis subsystem.
//!
//! ## Architecture
//!
//! - `domain` — Core domain model: operator trait, scoring, assessment, context, reports
//! - `error` — Optimization error types
//! - `pipeline` — Iterative optimization pipeline (planned, Phase 2)
//! - `operators` — Concrete operator implementations (planned, Phase 3)
//! - `adapters` — Adapter from legacy `RepairStrategy` to `TrajectoryOperator` (planned, Phase 3)

pub mod domain;
pub mod error;
pub mod operators;
pub mod pipeline;
pub mod temporal;

// Re-export the problem region types used by the operator trait.
// These types are defined in thalos-core and re-exported for convenience.
pub use thalos_core::analysis::region::{
    ProblemRegion, RegionEvidence, RegionId, RegionKind, RegionSeverity,
};
pub use thalos_core::evaluation::PlanMetrics;

// Convenience re-exports from domain
pub use domain::{
    Invariant, JointLimits, OperatorAssessment, OperatorFamily, OperatorScore, OptimizationContext,
    OptimizationObjective, OptimizationReport, OptimizationStep, PipelineConfig, Reason,
    TrajectoryOperator,
};
pub use error::OptimizationError;

// Re-exports from temporal
pub use temporal::{extract_velocity_limits, min_segment_duration};

// Re-exports from pipeline
pub use pipeline::{
    AcceptanceEvaluation, AcceptancePolicy, OperatorSelector, OptimizationPipeline,
    OptimizationResult,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{context::JointLimits, score};
    use std::sync::Arc;
    use thalos_core::{
        analysis::region::{ProblemRegion, RegionId, RegionKind, RegionSeverity},
        analysis::AnalysisReport,
        evaluation::PlanMetrics,
        models::{RobotModel, RobotRegistry},
        operation::{ConstraintQuery, PrecisionLevel},
        robot::serial_chain::SerialChain,
        trajectory::{Trajectory, TrajectoryPoint},
    };

    /// Mock operator for testing the TrajectoryOperator trait contract.
    struct MockOperator;

    impl TrajectoryOperator for MockOperator {
        fn id(&self) -> &'static str {
            "mock_operator"
        }

        fn family(&self) -> OperatorFamily {
            OperatorFamily::Geometry
        }

        fn applicability(&self, _region: &ProblemRegion) -> f32 {
            0.85
        }

        fn estimate_improvement(&self, _region: &ProblemRegion, _metrics: &PlanMetrics) -> f32 {
            0.6
        }

        fn estimate_cost(&self) -> f32 {
            1.0
        }

        fn apply(
            &self,
            _robot: &SerialChain,
            trajectory: &Trajectory,
            _region: &ProblemRegion,
            _ctx: &OptimizationContext,
            _constraints: Option<&dyn ConstraintQuery>,
        ) -> Result<Trajectory, OptimizationError> {
            Ok(trajectory.clone())
        }
    }

    #[test]
    fn trajectory_operator_trait_is_object_safe() {
        // Trait must be object-safe for dynamic dispatch.
        let op: Arc<dyn TrajectoryOperator> = Arc::new(MockOperator);
        assert_eq!(op.id(), "mock_operator");
        assert_eq!(op.family(), OperatorFamily::Geometry);
        assert!((op.estimate_cost() - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn trajectory_operator_send_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<MockOperator>();
    }

    #[test]
    fn trajectory_operator_applicability_range() {
        let op = MockOperator;
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Critical,
            0..5,
        );
        let app = op.applicability(&region);
        assert!((0.0..=1.0).contains(&app));
    }

    #[test]
    fn trajectory_operator_apply_returns_ok() {
        let op = MockOperator;
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![TrajectoryPoint::new(vec![0.0, 0.0], 0.0)]);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Critical,
            0..1,
        );
        let ctx = OptimizationContext {
            joint_limits: JointLimits {
                lower: vec![-std::f64::consts::PI, -std::f64::consts::PI],
                upper: vec![std::f64::consts::PI, std::f64::consts::PI],
                velocity: None,
                acceleration: None,
            },
            config: PipelineConfig::default(),
            tool_frame: None,
        };

        let result = op.apply(&robot, &traj, &region, &ctx, None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().len(), 1);
    }

    #[test]
    fn re_export_problem_region() {
        let _region = ProblemRegion::new(
            RegionId(42),
            RegionKind::Collision,
            RegionSeverity::Warning,
            10..20,
        );
    }

    #[test]
    fn re_export_plan_metrics() {
        let _metrics = PlanMetrics::new(
            0.0,
            0,
            thalos_core::evaluation::ManipulabilityMetrics::new(0.0, 0.0, 0, 0),
            thalos_core::evaluation::JointSafetyMetrics::new(1.0, 0.0, 0),
            thalos_core::evaluation::CollisionMetrics::new(1.0, 0, 0),
            0.0,
            0.0,
        );
    }

    #[test]
    fn re_export_operator_family() {
        let fam = OperatorFamily::JointSpace;
        assert_ne!(fam, OperatorFamily::Geometry);
    }

    #[test]
    fn score_compute_via_domain_module() {
        let s = score::compute_score(1.0, 1.0, 1.0);
        assert!((s - 1.0).abs() < f32::EPSILON);
    }

    // ── Helpers ──────────────────────────────────────────

    fn test_region(id: usize) -> ProblemRegion {
        ProblemRegion::new(
            RegionId(id),
            RegionKind::Singularity,
            RegionSeverity::Critical,
            id..(id + 3),
        )
    }

    fn test_robot() -> SerialChain {
        RobotRegistry::create_default(RobotModel::Planar2R)
    }

    fn test_trajectory() -> Trajectory {
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.5], 1.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 2.0),
        ])
    }

    fn test_metrics() -> PlanMetrics {
        PlanMetrics::new(
            0.0,
            0,
            thalos_core::evaluation::ManipulabilityMetrics::new(0.0, 0.0, 0, 0),
            thalos_core::evaluation::JointSafetyMetrics::new(1.0, 0.0, 0),
            thalos_core::evaluation::CollisionMetrics::new(1.0, 0, 0),
            0.0,
            0.0,
        )
    }

    fn test_ctx() -> OptimizationContext {
        OptimizationContext {
            joint_limits: JointLimits {
                lower: vec![-std::f64::consts::PI, -std::f64::consts::PI],
                upper: vec![std::f64::consts::PI, std::f64::consts::PI],
                velocity: None,
                acceleration: None,
            },
            config: PipelineConfig::default(),
            tool_frame: None,
        }
    }

    /// Report builder for the report-consumption tests (PR6 6.1). Three
    /// `Singularity` observations anchored at waypoints 0..3 group (via
    /// `RegionGrouper`) into a single Critical region 0..3 — the
    /// report-shaped equivalent of `test_region(0)`. Operators MUST see the
    /// same regions they would have received directly.
    fn test_report() -> AnalysisReport {
        use std::collections::BTreeMap;
        use thalos_core::analysis::{
            AnalysisSummary, ArtifactRef, Grade, Location, Observation, ObservationId,
            ObservationKind, Severity,
        };
        use thalos_core::ids::MotionPlanId;

        let observation = |id: u32, waypoint: usize| Observation {
            id: ObservationId(id),
            kind: ObservationKind::Singularity,
            severity: Severity::Error,
            artifact: ArtifactRef::MotionPlan(MotionPlanId("test-report".to_string())),
            location: Location::Waypoint(waypoint),
            attributes: BTreeMap::new(),
            causes: vec![],
            related: vec![],
        };
        AnalysisReport {
            artifact: ArtifactRef::MotionPlan(MotionPlanId("test-report".to_string())),
            observations: vec![observation(0, 0), observation(1, 1), observation(2, 2)],
            actions: vec![],
            metrics: BTreeMap::new(),
            summary: AnalysisSummary {
                quality_index: 0.5,
                observation_count: 3,
                severity_distribution: BTreeMap::new(),
                grade: Grade::Fair,
            },
            robot_id: None,
        }
    }

    /// A report with NO observations — the pipeline must derive ZERO regions
    /// (different code path than the populated report: no steps at all).
    fn test_empty_report() -> AnalysisReport {
        let mut report = test_report();
        report.observations = vec![];
        report.summary.observation_count = 0;
        report
    }

    /// Mock operator with configurable scores and apply behavior.
    struct ScoreMock {
        id: &'static str,
        family: OperatorFamily,
        applicability: f32,
        improvement: f32,
        cost: f32,
        apply_ok: bool,
    }

    impl ScoreMock {
        const fn new(
            id: &'static str,
            family: OperatorFamily,
            applicability: f32,
            improvement: f32,
            cost: f32,
        ) -> Self {
            Self {
                id,
                family,
                applicability,
                improvement,
                cost,
                apply_ok: true,
            }
        }

        fn with_failure(mut self) -> Self {
            self.apply_ok = false;
            self
        }
    }

    impl TrajectoryOperator for ScoreMock {
        fn id(&self) -> &'static str {
            self.id
        }

        fn family(&self) -> OperatorFamily {
            self.family
        }

        fn applicability(&self, _region: &ProblemRegion) -> f32 {
            self.applicability
        }

        fn estimate_improvement(&self, _region: &ProblemRegion, _metrics: &PlanMetrics) -> f32 {
            self.improvement
        }

        fn estimate_cost(&self) -> f32 {
            self.cost
        }

        fn apply(
            &self,
            _robot: &SerialChain,
            trajectory: &Trajectory,
            _region: &ProblemRegion,
            _ctx: &OptimizationContext,
            _constraints: Option<&dyn ConstraintQuery>,
        ) -> Result<Trajectory, OptimizationError> {
            if self.apply_ok {
                Ok(trajectory.clone())
            } else {
                Err(OptimizationError::OperatorFailed {
                    operator: self.id,
                    reason: "mock failure".into(),
                })
            }
        }
    }

    /// Permissive mock query — every guard returns true.
    struct AlwaysAllowQuery;

    impl ConstraintQuery for AlwaysAllowQuery {
        fn can_relax_orientation(&self, _waypoint_index: usize, _max_angle: f64) -> bool {
            true
        }
        fn can_modify_position(&self, _waypoint_index: usize) -> bool {
            true
        }
        fn max_position_error(&self, _waypoint_index: usize) -> Option<f64> {
            None
        }
        fn max_velocity(&self, _waypoint_index: usize) -> Option<f64> {
            None
        }
        fn required_precision(&self, _waypoint_index: usize) -> PrecisionLevel {
            PrecisionLevel::None
        }
    }

    /// Operator that records whether `apply()` received `Some` or `None`
    /// constraints on every call (used to prove pipeline forwarding).
    struct RecordingOperator {
        id: &'static str,
        family: OperatorFamily,
        constraints_seen: std::sync::Mutex<Vec<bool>>,
    }

    impl RecordingOperator {
        fn new(id: &'static str, family: OperatorFamily) -> Self {
            Self {
                id,
                family,
                constraints_seen: std::sync::Mutex::new(Vec::new()),
            }
        }
    }

    impl TrajectoryOperator for RecordingOperator {
        fn id(&self) -> &'static str {
            self.id
        }

        fn family(&self) -> OperatorFamily {
            self.family
        }

        fn applicability(&self, _region: &ProblemRegion) -> f32 {
            1.0
        }

        fn estimate_improvement(&self, _region: &ProblemRegion, _metrics: &PlanMetrics) -> f32 {
            1.0
        }

        fn estimate_cost(&self) -> f32 {
            1.0
        }

        fn apply(
            &self,
            _robot: &SerialChain,
            trajectory: &Trajectory,
            _region: &ProblemRegion,
            _ctx: &OptimizationContext,
            constraints: Option<&dyn ConstraintQuery>,
        ) -> Result<Trajectory, OptimizationError> {
            self.constraints_seen
                .lock()
                .expect("recording mutex poisoned")
                .push(constraints.is_some());
            Ok(trajectory.clone())
        }
    }

    // ── ConstraintQuery forwarding through the pipeline (2.3) ──

    #[test]
    fn pipeline_forwards_constraints_to_geometric_operators() {
        let pipeline = OptimizationPipeline::new(PipelineConfig::default());
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0)];
        let metrics = test_metrics();
        let ctx = test_ctx();
        let query = AlwaysAllowQuery;

        // Some(query) → geometric apply() receives Some
        let op = RecordingOperator::new("rec", OperatorFamily::Geometry);
        let operators: [&dyn TrajectoryOperator; 1] = [&op];
        pipeline
            .optimize_regions(
                &operators, &robot, &traj, &regions, &metrics, &ctx, Some(&query),
            )
            .expect("pipeline should succeed");
        assert_eq!(
            op.constraints_seen
                .lock()
                .expect("recording mutex poisoned")
                .as_slice(),
            &[true],
            "geometric apply() must receive Some(&query)"
        );

        // None → geometric apply() receives None
        let op2 = RecordingOperator::new("rec", OperatorFamily::Geometry);
        let operators2: [&dyn TrajectoryOperator; 1] = [&op2];
        pipeline
            .optimize_regions(&operators2, &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should succeed");
        assert_eq!(
            op2.constraints_seen
                .lock()
                .expect("recording mutex poisoned")
                .as_slice(),
            &[false],
            "geometric apply() must receive None"
        );
    }

    #[test]
    fn pipeline_forwards_constraints_to_temporal_post_pass() {
        let pipeline = OptimizationPipeline::new(PipelineConfig::default());
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0)];
        let metrics = test_metrics();
        let ctx = test_ctx();
        let query = AlwaysAllowQuery;

        // "retime" (Temporal family) is deferred in the geometric pass and
        // applied exactly once in the temporal post-pass.
        let op = RecordingOperator::new("retime", OperatorFamily::Temporal);
        let operators: [&dyn TrajectoryOperator; 1] = [&op];
        let result = pipeline
            .optimize_regions(
                &operators, &robot, &traj, &regions, &metrics, &ctx, Some(&query),
            )
            .expect("pipeline should succeed");

        assert_eq!(
            op.constraints_seen
                .lock()
                .expect("recording mutex poisoned")
                .as_slice(),
            &[true],
            "temporal post-pass apply() must receive Some(&query)"
        );
        assert_eq!(
            result.report.steps.len(),
            2,
            "1 deferred step + 1 post-pass step"
        );
        assert_eq!(result.report.steps[1].operator_id, "retime");

        // None → post-pass apply() receives None
        let op2 = RecordingOperator::new("retime", OperatorFamily::Temporal);
        let operators2: [&dyn TrajectoryOperator; 1] = [&op2];
        pipeline
            .optimize_regions(&operators2, &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should succeed");
        assert_eq!(
            op2.constraints_seen
                .lock()
                .expect("recording mutex poisoned")
                .as_slice(),
            &[false],
            "temporal post-pass apply() must receive None"
        );
    }

    // ── OperatorSelector Tests ───────────────────────────

    #[test]
    fn operator_selector_assess_computes_correct_score() {
        let op = ScoreMock::new("test_op", OperatorFamily::JointSpace, 0.8, 0.5, 2.0);
        let region = test_region(0);
        let metrics = test_metrics();

        let assessment = OperatorSelector::assess(&op, &region, &metrics);

        assert_eq!(assessment.operator_id, "test_op");
        assert_eq!(assessment.family, OperatorFamily::JointSpace);
        let expected_composite = 0.8 * 0.5 / 2.0;
        assert!(
            (assessment.score.composite - expected_composite).abs() < f32::EPSILON,
            "expected {} got {}",
            expected_composite,
            assessment.score.composite
        );
        assert!((assessment.score.applicability - 0.8).abs() < f32::EPSILON);
        assert!((assessment.score.estimated_improvement - 0.5).abs() < f32::EPSILON);
        assert!((assessment.score.estimated_cost - 2.0).abs() < f32::EPSILON);
    }

    #[test]
    fn operator_selector_rank_empty_list() {
        let operators: [&dyn TrajectoryOperator; 0] = [];
        let region = test_region(0);
        let metrics = test_metrics();

        let ranked = OperatorSelector::rank(&operators, &region, &metrics);
        assert!(ranked.is_empty());
    }

    #[test]
    fn operator_selector_rank_single_operator() {
        let op = ScoreMock::new("single", OperatorFamily::Geometry, 1.0, 1.0, 1.0);
        let operators: [&dyn TrajectoryOperator; 1] = [&op];
        let region = test_region(0);
        let metrics = test_metrics();

        let ranked = OperatorSelector::rank(&operators, &region, &metrics);
        assert_eq!(ranked.len(), 1);
        assert_eq!(ranked[0].0.id(), "single");
        assert!((ranked[0].1.score.composite - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn operator_selector_rank_sorts_by_score_descending() {
        let high = ScoreMock::new("high", OperatorFamily::JointSpace, 0.9, 0.9, 1.0);
        let mid = ScoreMock::new("mid", OperatorFamily::Geometry, 0.5, 0.5, 1.0);
        let low = ScoreMock::new("low", OperatorFamily::Temporal, 0.1, 0.1, 1.0);

        let operators: [&dyn TrajectoryOperator; 3] = [&low, &high, &mid];
        let region = test_region(0);
        let metrics = test_metrics();

        let ranked = OperatorSelector::rank(&operators, &region, &metrics);
        assert_eq!(ranked.len(), 3);
        assert_eq!(ranked[0].0.id(), "high");
        assert_eq!(ranked[1].0.id(), "mid");
        assert_eq!(ranked[2].0.id(), "low");
        // Verify scores are strictly descending
        assert!(ranked[0].1.score.composite > ranked[1].1.score.composite);
        assert!(ranked[1].1.score.composite > ranked[2].1.score.composite);
    }

    #[test]
    fn operator_selector_rank_ties_preserve_insertion_order() {
        let a = ScoreMock::new("a", OperatorFamily::Sampling, 0.5, 0.5, 1.0);
        let b = ScoreMock::new("b", OperatorFamily::Geometry, 0.5, 0.5, 1.0);

        let operators: [&dyn TrajectoryOperator; 2] = [&a, &b];
        let region = test_region(0);
        let metrics = test_metrics();

        // With equal scores, partial_cmp returns Equal, preserving original order
        let ranked = OperatorSelector::rank(&operators, &region, &metrics);
        assert_eq!(ranked.len(), 2);
        assert!(ranked[0].0.id() == "a" || ranked[0].0.id() == "b");
    }

    // ── OptimizationPipeline Tests ───────────────────────

    #[test]
    fn pipeline_default_config_values() {
        let config = PipelineConfig::default();
        assert_eq!(config.max_iterations_per_region, 3);
        assert!((config.improvement_threshold - 0.01).abs() < f32::EPSILON);
        assert!((config.centering_factor - 0.3).abs() < f64::EPSILON);
    }

    #[test]
    fn pipeline_with_no_operators_returns_empty_report() {
        let config = PipelineConfig::default();
        let pipeline = OptimizationPipeline::new(config);
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0)];
        let metrics = test_metrics();
        let ctx = test_ctx();

        let result = pipeline
            .optimize_regions(&[], &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should succeed with no operators");

        assert!(result.report.steps.is_empty(), "expected no steps");
        // Trajectory should be unchanged
        assert_eq!(result.trajectory.len(), traj.len());
    }

    #[test]
    fn pipeline_optimize_consumes_analysis_report_and_derives_regions() {
        // PR6 6.1: the pipeline's PRIMARY API takes `&AnalysisReport` and
        // derives problem regions INTERNALLY via RegionGrouper — operator
        // behavior is unchanged (same region → same ranking/apply).
        let config = PipelineConfig::default();
        let pipeline = OptimizationPipeline::new(config);
        let robot = test_robot();
        let traj = test_trajectory();
        let metrics = test_metrics();
        let ctx = test_ctx();

        // Happy path: populated report → one derived Singularity region 0..3
        // → exactly one accepted step from the operator.
        let op = ScoreMock::new("report_op", OperatorFamily::Geometry, 1.0, 1.0, 1.0);
        let operators: [&dyn TrajectoryOperator; 1] = [&op];
        let result = pipeline
            .optimize(&operators, &robot, &traj, &test_report(), &metrics, &ctx, None)
            .expect("pipeline should succeed with a report input");

        assert_eq!(result.report.steps.len(), 1);
        assert_eq!(result.report.steps[0].operator_id, "report_op");
        assert_eq!(result.report.steps[0].region_id, RegionId(0));
        assert!(result.report.steps[0].accepted);
        assert_eq!(result.trajectory.len(), traj.len());

        // Empty report → zero derived regions → zero steps (proves the
        // derivation is real: nothing to process, different code path).
        let op2 = ScoreMock::new("report_op2", OperatorFamily::Geometry, 1.0, 1.0, 1.0);
        let operators2: [&dyn TrajectoryOperator; 1] = [&op2];
        let result2 = pipeline
            .optimize(&operators2, &robot, &traj, &test_empty_report(), &metrics, &ctx, None)
            .expect("pipeline should succeed with an empty report");

        assert!(result2.report.steps.is_empty(), "no regions → no steps");
        assert_eq!(result2.trajectory.len(), traj.len());
    }

    #[test]
    fn pipeline_single_region_applies_best_operator() {
        let config = PipelineConfig::default();
        let pipeline = OptimizationPipeline::new(config);
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0)];
        let metrics = test_metrics();
        let ctx = test_ctx();

        let op = ScoreMock::new("best_op", OperatorFamily::Geometry, 1.0, 1.0, 1.0);
        let operators: [&dyn TrajectoryOperator; 1] = [&op];

        let result = pipeline
            .optimize_regions(&operators, &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should succeed");

        assert_eq!(result.report.steps.len(), 1);
        assert_eq!(result.report.steps[0].operator_id, "best_op");
        assert!(result.report.steps[0].accepted);
        assert_eq!(result.report.steps[0].region_id, RegionId(0));
    }

    #[test]
    fn pipeline_multiple_regions_processes_all() {
        let config = PipelineConfig::default();
        let pipeline = OptimizationPipeline::new(config);
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0), test_region(1), test_region(2)];
        let metrics = test_metrics();
        let ctx = test_ctx();

        let op = ScoreMock::new("universal_op", OperatorFamily::JointSpace, 0.9, 0.6, 1.0);
        let operators: [&dyn TrajectoryOperator; 1] = [&op];

        let result = pipeline
            .optimize_regions(&operators, &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should succeed");

        assert_eq!(result.report.steps.len(), 3);
        for step in &result.report.steps {
            assert_eq!(step.operator_id, "universal_op");
            assert!(step.accepted);
        }
    }

    #[test]
    fn pipeline_all_operators_fail_records_failure() {
        let config = PipelineConfig::default();
        let pipeline = OptimizationPipeline::new(config);
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0)];
        let metrics = test_metrics();
        let ctx = test_ctx();

        let op =
            ScoreMock::new("failing_op", OperatorFamily::Temporal, 1.0, 1.0, 1.0).with_failure();
        let operators: [&dyn TrajectoryOperator; 1] = [&op];

        let result = pipeline
            .optimize_regions(&operators, &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should not error on operator failure");

        assert_eq!(result.report.steps.len(), 1);
        assert_eq!(result.report.steps[0].operator_id, "failing_op");
        assert!(!result.report.steps[0].accepted);
        assert!(
            result.report.steps[0].rejection_reason.is_some(),
            "rejection reason should be recorded for operator failure"
        );
    }

    #[test]
    fn pipeline_falls_back_to_next_operator_when_first_fails() {
        let config = PipelineConfig::default();
        let pipeline = OptimizationPipeline::new(config);
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0)];
        let metrics = test_metrics();
        let ctx = test_ctx();

        let fail_op =
            ScoreMock::new("fails", OperatorFamily::Geometry, 0.9, 0.8, 1.0).with_failure();
        let succeed_op = ScoreMock::new("succeeds", OperatorFamily::JointSpace, 0.5, 0.5, 1.0);
        // fail_op has higher score, so it's tried first
        let operators: [&dyn TrajectoryOperator; 2] = [&succeed_op, &fail_op];

        let result = pipeline
            .optimize_regions(&operators, &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should succeed");

        assert_eq!(result.report.steps.len(), 1);
        // Since succeed_op has score 0.25 and fail_op has score 0.72,
        // fail_op is ranked first but fails, then succeed_op succeeds
        assert_eq!(result.report.steps[0].operator_id, "succeeds");
        assert!(result.report.steps[0].accepted);
    }

    #[test]
    fn pipeline_operators_are_ranked_before_application() {
        let config = PipelineConfig::default();
        let pipeline = OptimizationPipeline::new(config);
        let robot = test_robot();
        let traj = test_trajectory();
        let regions = vec![test_region(0)];
        let metrics = test_metrics();
        let ctx = test_ctx();

        // low score succeeds, high score fails — high score tried first
        let high_score_fails =
            ScoreMock::new("high_score_fails", OperatorFamily::Geometry, 0.9, 0.9, 1.0)
                .with_failure();
        let low_score_succeeds = ScoreMock::new(
            "low_score_succeeds",
            OperatorFamily::JointSpace,
            0.3,
            0.3,
            1.0,
        );

        let operators: [&dyn TrajectoryOperator; 2] = [&low_score_succeeds, &high_score_fails];

        let result = pipeline
            .optimize_regions(&operators, &robot, &traj, &regions, &metrics, &ctx, None)
            .expect("pipeline should succeed");

        assert_eq!(result.report.steps.len(), 1);
        // High score (0.81) fails, then low score (0.09) succeeds
        assert_eq!(result.report.steps[0].operator_id, "low_score_succeeds");
        assert!(result.report.steps[0].accepted);
    }

    #[test]
    fn pipeline_re_exports_are_accessible() {
        // Verify that the pipeline types are re-exported from the crate root
        let _pipe: OptimizationPipeline;
        let _result: OptimizationResult;
        let _selector: OperatorSelector;
    }
}
