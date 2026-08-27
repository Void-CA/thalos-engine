use crate::{
    ProblemRegion, RegionId, RegionKind, RegionSeverity,
    domain::{
        OptimizationContext, OptimizationReport, OptimizationStep, PipelineConfig,
        TrajectoryOperator,
    },
    error::OptimizationError,
    pipeline::{
        OperatorSelector, acceptance::AcceptancePolicy, trajectory_composer::compose_trajectory,
    },
};
use thalos_core::{
    analysis::{AnalysisReport, RegionGrouper},
    evaluation::PlanMetrics,
    operation::ConstraintQuery,
    robot::serial_chain::SerialChain,
    trajectory::Trajectory,
};

/// The result of a full pipeline optimization run.
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    /// Detailed report of all optimization steps performed.
    pub report: OptimizationReport,
    /// The final optimized trajectory.
    pub trajectory: Trajectory,
}

/// Iterative optimization pipeline that processes problem regions
/// sequentially, applying the highest-ranked operator to each region.
///
/// For each region the pipeline:
/// 1. Ranks available operators by composite score
/// 2. Attempts the top-ranked operator → produces a **candidate**
/// 3. Blends the modified segment with the original trajectory at boundaries
/// 4. **Evaluates** the candidate with `AcceptancePolicy` — if metrics
///    degraded, rejects and tries the next operator
/// 5. If accepted, moves to the next region
/// 6. If all operators fail or are rejected, records a failed step
///    with the rejection reason from the last attempted operator.
///
/// After all geometric regions are processed, runs a **temporal post-pass**
/// (Retime) on the full trajectory if the operator is available.
#[derive(Debug, Clone)]
pub struct OptimizationPipeline {
    config: PipelineConfig,
}

impl OptimizationPipeline {
    /// Create a new pipeline with the given configuration.
    pub fn new(config: PipelineConfig) -> Self {
        Self { config }
    }

    /// Run the optimization pipeline over an [`AnalysisReport`], deriving the
    /// problem regions INTERNALLY from the report's observations via
    /// [`RegionGrouper`] (spec trajectory-optimization-pipeline: "Direct report
    /// consumption"). The public API takes the report directly — callers never
    /// pre-derive regions. Operator behavior is unchanged: the derived regions
    /// drive the exact same per-region ranking/apply/acceptance loop as
    /// [`Self::optimize_regions`].
    ///
    /// # Parameters
    /// - `operators`: Slice of operator trait objects to consider
    /// - `robot`: The robot model (passed through to operators)
    /// - `trajectory`: The initial trajectory to optimize
    /// - `report`: The canonical analysis report (regions derived from
    ///   `report.observations`)
    /// - `metrics`: Current plan metrics for scoring
    /// - `ctx`: Optimization context (joint limits, config)
    /// - `constraints`: Optional `ConstraintQuery` forwarded to every
    ///   operator `apply()` call in both the geometric pass and the
    ///   temporal post-pass. `None` preserves the legacy behavior.
    ///
    /// Returns an `OptimizationResult` containing the report and
    /// the final optimized trajectory.
    pub fn optimize(
        &self,
        operators: &[&dyn TrajectoryOperator],
        robot: &SerialChain,
        trajectory: &Trajectory,
        report: &AnalysisReport,
        metrics: &PlanMetrics,
        ctx: &OptimizationContext,
        constraints: Option<&dyn ConstraintQuery>,
    ) -> Result<OptimizationResult, OptimizationError> {
        let regions = RegionGrouper::default().group(&report.observations);
        self.optimize_regions(
            operators,
            robot,
            trajectory,
            &regions,
            metrics,
            ctx,
            constraints,
        )
    }

    /// LEGACY regions-based entry point — the same per-region loop, fed with an
    /// explicit region slice instead of a report.
    ///
    /// Retained for the repair-session flow (`TrajectoryOptimizer` wrapper) and
    /// region-level tests, per the design's legacy fallback paths. New callers
    /// MUST use [`Self::optimize`] with an `&AnalysisReport`.
    ///
    /// # Parameters
    /// - `operators`: Slice of operator trait objects to consider
    /// - `robot`: The robot model (passed through to operators)
    /// - `trajectory`: The initial trajectory to optimize
    /// - `regions`: Problem regions detected in the trajectory
    /// - `metrics`: Current plan metrics for scoring
    /// - `ctx`: Optimization context (joint limits, config)
    /// - `constraints`: Optional `ConstraintQuery` forwarded to every
    ///   operator `apply()` call in both the geometric pass and the
    ///   temporal post-pass. `None` preserves the legacy behavior.
    ///
    /// Returns an `OptimizationResult` containing the report and
    /// the final optimized trajectory.
    pub fn optimize_regions(
        &self,
        operators: &[&dyn TrajectoryOperator],
        robot: &SerialChain,
        trajectory: &Trajectory,
        regions: &[ProblemRegion],
        metrics: &PlanMetrics,
        ctx: &OptimizationContext,
        constraints: Option<&dyn ConstraintQuery>,
    ) -> Result<OptimizationResult, OptimizationError> {
        let mut current = trajectory.clone();
        let mut steps = Vec::new();
        let total_improvement = 0.0;
        let policy = AcceptancePolicy::default();

        // ── Phase 1: Geometric optimization (per-region, with acceptance) ──
        for region in regions {
            let ranked = OperatorSelector::rank(operators, region, metrics);
            if ranked.is_empty() {
                continue;
            }

            let mut accepted_step: Option<OptimizationStep> = None;
            let mut last_rejection: Option<OptimizationStep> = None;

            for (op, _assessment) in ranked {
                // Skip temporal operators in the geometric pass — they run
                // as a mandatory post-pass.
                if op.family() == crate::domain::operator::OperatorFamily::Temporal {
                    last_rejection = Some(OptimizationStep {
                        region_id: region.id,
                        operator_id: op.id(),
                        improvement: 0.0,
                        accepted: false,
                        iteration: 0,
                        rejection_reason: Some("deferred to temporal post-pass".into()),
                    });
                    continue;
                }

                match op.apply(robot, &current, region, ctx, constraints) {
                    Ok(candidate_raw) => {
                        let blended = compose_trajectory(
                            &current,
                            &candidate_raw,
                            &region.waypoint_range,
                            self.config.blend_window,
                            self.config.blend_policy,
                        );

                        let evaluation = policy.evaluate(&current, &blended, ctx);

                        if evaluation.accepted {
                            accepted_step = Some(OptimizationStep {
                                region_id: region.id,
                                operator_id: op.id(),
                                improvement: 0.0,
                                accepted: true,
                                iteration: 0,
                                rejection_reason: None,
                            });
                            current = blended;
                            break;
                        } else {
                            last_rejection = Some(OptimizationStep {
                                region_id: region.id,
                                operator_id: op.id(),
                                improvement: 0.0,
                                accepted: false,
                                iteration: 0,
                                rejection_reason: Some(format!("rejected: {}", evaluation.reason)),
                            });
                        }
                    }
                    Err(e) => {
                        last_rejection = Some(OptimizationStep {
                            region_id: region.id,
                            operator_id: op.id(),
                            improvement: 0.0,
                            accepted: false,
                            iteration: 0,
                            rejection_reason: Some(format!("error: {}", e)),
                        });
                    }
                }
            }

            // Push exactly ONE step per region
            if let Some(accepted) = accepted_step {
                steps.push(accepted);
            } else if let Some(rejected) = last_rejection {
                steps.push(rejected);
            } else {
                steps.push(OptimizationStep {
                    region_id: region.id,
                    operator_id: "none",
                    improvement: 0.0,
                    accepted: false,
                    iteration: 0,
                    rejection_reason: None,
                });
            }
        }

        // ── Phase 2: Temporal post-pass (Retime on full trajectory) ──
        if let Some(retime_op) = operators.iter().find(|op| op.id() == "retime") {
            let full_range = ProblemRegion::new(
                RegionId(usize::MAX),
                RegionKind::Velocity,
                RegionSeverity::Info,
                0..current.len(),
            );

            match retime_op.apply(robot, &current, &full_range, ctx, constraints) {
                Ok(retimed) => {
                    let blended = compose_trajectory(
                        &current,
                        &retimed,
                        &full_range.waypoint_range,
                        self.config.blend_window,
                        self.config.blend_policy,
                    );
                    let eval = policy.evaluate(&current, &blended, ctx);
                    if eval.accepted {
                        steps.push(OptimizationStep {
                            region_id: full_range.id,
                            operator_id: "retime",
                            improvement: 0.0,
                            accepted: true,
                            iteration: 0,
                            rejection_reason: None,
                        });
                        current = blended;
                    } else {
                        steps.push(OptimizationStep {
                            region_id: full_range.id,
                            operator_id: "retime",
                            improvement: 0.0,
                            accepted: false,
                            iteration: 0,
                            rejection_reason: Some(eval.reason),
                        });
                    }
                }
                Err(e) => {
                    steps.push(OptimizationStep {
                        region_id: full_range.id,
                        operator_id: "retime",
                        improvement: 0.0,
                        accepted: false,
                        iteration: 0,
                        rejection_reason: Some(format!("error: {}", e)),
                    });
                }
            }
        }

        Ok(OptimizationResult {
            report: OptimizationReport {
                steps,
                final_trajectory: Some(current.clone()),
                total_improvement,
            },
            trajectory: current,
        })
    }
}
