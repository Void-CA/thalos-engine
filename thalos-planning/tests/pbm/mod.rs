//! Pipeline Benchmark — infrastructure for end-to-end OptimizationPipeline tests.
//!
//! Provides the `BenchmarkScenario` trait, `run_scenario()` helper, and
//! re-exports from `metrics` and `report` submodules.
//!
//! # Architecture
//!
//! Each scenario implements `BenchmarkScenario` to define:
//! - Which robot model to use
//! - A problematic trajectory
//! - Expected metric improvements
//!
//! `run_scenario()` orchestrates the full analyze → detect → optimize →
//! re-evaluate → assert flow for a given scenario.

pub mod metrics;
pub mod report;
pub mod scenarios;

pub use metrics::{
    ExpectedImprovement, ImprovementDirection, MetricDelta, MetricKind, assert_improvements,
    compare_metrics,
};
pub use report::{OperatorEntry, OperatorStatus, PipelineReport};

use thalos_core::{
    analysis::RegionGrouper,
    analysis::constraints::{Constraint, DefaultConstraintEvaluator},
    analysis::observation::ArtifactRef,
    ids::MotionPlanId,
    models::{RobotModel, RobotRegistry},
    trajectory::Trajectory,
};
use thalos_optimization::{
    TrajectoryOperator,
    domain::{JointLimits, OptimizationContext, PipelineConfig},
    operators::{
        AdaptiveSampling, JointCenteringOperator, NullSpaceOptimization, OrientationRelaxation,
        Retime,
    },
    pipeline::OptimizationPipeline,
};
use thalos_planning::{analysis::TrajectoryAnalyzer, evaluation::PlanEvaluator};

/// A benchmark scenario defines the inputs and expected outcomes for a
/// single pipeline optimization run.
///
/// Implementations provide:
/// - A robot model (used to build a `SerialChain` via `RobotRegistry`)
/// - A trajectory with known problems
/// - Expected improvements that the pipeline should achieve
pub trait BenchmarkScenario {
    /// Human-readable scenario name (e.g. "joint_limit", "near_singularity").
    fn name(&self) -> &'static str;

    /// The robot model to use for this scenario.
    ///
    /// The `run_scenario` helper calls `RobotRegistry::create_default()`
    /// with this model to obtain the `SerialChain`.
    fn robot_model(&self) -> RobotModel;

    /// The input trajectory with known problems for the pipeline to solve.
    fn trajectory(&self) -> Trajectory;

    /// Expected metric improvements after pipeline optimization.
    ///
    /// Must contain at least one entry — scenarios with zero expected
    /// improvements are considered invalid.
    fn expected_improvements(&self) -> Vec<ExpectedImprovement>;

    /// Optional constraints to enforce during trajectory analysis.
    ///
    /// When non-empty, the analyzer is set up with the given constraints
    /// and `DefaultConstraintEvaluator`. Violations produce
    /// `ConstraintViolation` findings, which create `Constraint`-kind
    /// problem regions. This is the only way to trigger operators like
    /// `JointCenteringOperator` on Planar2R (where `AdaptiveSampling`
    /// otherwise wins the ranking for all other region kinds).
    ///
    /// Default: empty (no constraint evaluation).
    fn constraints(&self) -> Vec<Constraint> {
        vec![]
    }
}

/// Run a benchmark scenario through the full pipeline.
///
/// Orchestrates the complete flow:
/// 1. Build robot from scenario's robot model
/// 2. Analyze the input trajectory → compute before-metrics
/// 3. Detect problem regions
/// 4. Run the optimization pipeline with all 5 operators
/// 5. Re-analyze the output trajectory → compute after-metrics
/// 6. Assert that expected improvements materialized
///
/// # Panics
///
/// Panics if analysis fails, the pipeline returns an error, or expected
/// improvements do not materialize.
///
/// # Returns
///
/// A `PipelineReport` summarizing operator activity and region count.
pub fn run_scenario(scenario: &dyn BenchmarkScenario) -> PipelineReport {
    // ── 1. Build robot ────────────────────────────────────
    let chain = RobotRegistry::create_default(scenario.robot_model());
    let traj = scenario.trajectory();

    // ── 2. Analyze BEFORE (with optional constraints) ────
    let cons = scenario.constraints();
    let evaluator = DefaultConstraintEvaluator;
    let analyzer = if cons.is_empty() {
        TrajectoryAnalyzer::new(&chain, None)
    } else {
        TrajectoryAnalyzer::new(&chain, None).with_constraints(&cons, &evaluator)
    };
    // Pasa único: análisis técnico + observaciones canónicas (PR 7a).
    let artifact = ArtifactRef::MotionPlan(MotionPlanId("pbm".to_string()));
    let (_analysis_before, observations_before) = analyzer
        .analyze_with_observations(artifact, &traj)
        .expect("before-analysis failed");
    let metrics_before = PlanEvaluator::compute_metrics_from_joints(&traj);

    // ── 3. Detect problem regions ─────────────────────────
    // Dueño único de la agrupación: RegionGrouper sobre observaciones.
    let regions = RegionGrouper::default().group(&observations_before);
    let regions_detected = regions.len();

    // ── 4. Build optimization context ─────────────────────
    let lower: Vec<f64> = chain
        .segments
        .iter()
        .map(|s| s.joint.limits().min)
        .collect();
    let upper: Vec<f64> = chain
        .segments
        .iter()
        .map(|s| s.joint.limits().max)
        .collect();
    let ctx = OptimizationContext {
        joint_limits: JointLimits {
            lower,
            upper,
            velocity: None,
            acceleration: None,
        },
        config: PipelineConfig::default(),
        tool_frame: None,
    };

    // ── 5. Create operators ───────────────────────────────
    let jc = JointCenteringOperator::new(JointCenteringOperator::DEFAULT_FACTOR);
    let ns = NullSpaceOptimization::new(
        NullSpaceOptimization::DEFAULT_FACTOR,
        1e-6, // tolerance
        0.1,  // dt
    );
    let retime = Retime::new(Retime::DEFAULT_VELOCITY, Retime::DEFAULT_MAX_DURATION_SCALE);
    let sampling = AdaptiveSampling::new(
        500,  // max_points
        0.5,  // error_threshold
        0.3,  // curvature_threshold
        0.01, // min_segment_length
    );
    let orient = OrientationRelaxation::new(
        0.1,  // max_angle
        1e-6, // tolerance
        0.1,  // dt
        1e-4, // position_tolerance
    );

    let operators: [&dyn TrajectoryOperator; 5] = [&jc, &ns, &retime, &sampling, &orient];

    // ── 6. Run optimization pipeline ──────────────────────
    let pipeline = OptimizationPipeline::new(PipelineConfig::default());
    let result = pipeline
        .optimize_regions(
            &operators,
            &chain,
            &traj,
            &regions,
            &metrics_before,
            &ctx,
            None,
        )
        .expect("pipeline optimization failed");

    // ── 7. Analyze AFTER ──────────────────────────────────
    let analysis_after = analyzer
        .analyze_plan(&result.trajectory)
        .expect("after-analysis failed");
    let metrics_after = PlanEvaluator::compute_metrics_from_joints(&result.trajectory);

    // ── 8. Build pipeline report ──────────────────────────
    let report = PipelineReport::from_optimization_report(&result.report, regions_detected);

    // ── 9. Assert expected improvements ───────────────────
    let deltas = compare_metrics(&metrics_before, &metrics_after, &traj, &result.trajectory);
    assert_improvements(&scenario.expected_improvements(), &deltas);

    report
}

/// Validate that a scenario has at least one expected improvement.
///
/// Returns `Ok(())` if the scenario is valid, or an error message
/// describing what is missing.
pub fn validate_scenario(scenario: &dyn BenchmarkScenario) -> Result<(), String> {
    let expected = scenario.expected_improvements();
    if expected.is_empty() {
        return Err(format!(
            "Scenario '{}' has zero expected improvements — at least one is required",
            scenario.name()
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummyScenario;

    impl BenchmarkScenario for DummyScenario {
        fn name(&self) -> &'static str {
            "dummy"
        }
        fn robot_model(&self) -> RobotModel {
            RobotModel::Planar2R
        }
        fn trajectory(&self) -> Trajectory {
            Trajectory::new(vec![])
        }
        fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
            vec![ExpectedImprovement {
                operator_id: "joint_centering",
                metric: MetricKind::JointMargin,
                direction: ImprovementDirection::Increase,
            }]
        }
    }

    #[test]
    fn benchmark_scenario_trait_is_object_safe() {
        let s: &dyn BenchmarkScenario = &DummyScenario;
        assert_eq!(s.name(), "dummy");
        assert_eq!(s.robot_model(), RobotModel::Planar2R);
        assert_eq!(s.expected_improvements().len(), 1);
    }

    #[test]
    fn validate_scenario_accepts_valid() {
        let s = DummyScenario;
        assert!(validate_scenario(&s).is_ok());
    }

    #[test]
    fn validate_scenario_rejects_empty_expected_improvements() {
        struct EmptyScenario;
        impl BenchmarkScenario for EmptyScenario {
            fn name(&self) -> &'static str {
                "empty"
            }
            fn robot_model(&self) -> RobotModel {
                RobotModel::Planar2R
            }
            fn trajectory(&self) -> Trajectory {
                Trajectory::new(vec![])
            }
            fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
                vec![]
            }
        }

        let s = EmptyScenario;
        let result = validate_scenario(&s);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("zero expected improvements"));
    }
}
