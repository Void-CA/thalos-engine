//! REAL usability test suite for the intelligence / AI-recommendation module.
//!
//! These are integration tests over the REAL pipeline with REAL IK — no mock
//! solvers, no mock materializers. They exercise exactly what a user does:
//! compile a motion program, analyze it, read the score and grade, consume the
//! advisor's recommendations, apply an edit, recompile and re-analyze.
//!
//! The suite is written against the CORRECT behavior. Three known bugs mean
//! several tests FAIL on current code — that failure IS the evidence:
//!
//! - **BUG 1** (`score_reality`): `quality_index` is a saturated severity
//!   count (`scoring.rs:47`), so the continuous quality metrics (smoothness,
//!   manipulability, joint safety, collision) never feed the score.
//! - **BUG 2** (`recommendation_applicability`): `RotateToolMaterializer`
//!   does NO IK verification, so a recommendation is marked `available` even
//!   when its edited program cannot be recompiled (the D8 gate lies), and
//!   `MoveLPlanner::plan` solves every intermediate waypoint with
//!   `IKGoal::Pose` (no position fallback), so the error surfaces as
//!   "segment N failed: Inverse kinematics failed for target pose".
//! - **BUG 3** (`edits_improve`): remediation changes geometry but not the
//!   joint-space phenomenon, and the saturated score hides the delta.

use thalos_collision::NaiveCollisionChecker;
use thalos_core::{
    analysis::{
        Aggregator,
        aggregator::DefaultAggregator,
        observation::{ArtifactRef, ObservationKind},
        scoring::DefaultScoringPolicy,
        summary::Grade,
    },
    collision::CollisionMatrix,
    ids::{MotionPlanId, OperationId},
    kinematics::{forward::ForwardKinematics, inverse::DampedLeastSquaresSolver},
    models::{RobotModel, RobotRegistry},
    motion::segment::MotionSegment,
    prelude::RobotState,
    robot::serial_chain::SerialChain,
    spatial::{frame::FrameId, pose::Pose},
    trajectory::{Trajectory, TrajectoryPoint},
};
use thalos_math::Transform3D;
use thalos_planning::{
    advisor::PlanAdvisor,
    analysis::TrajectoryAnalyzer,
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
        program::PlanningProgram,
    },
    recommendation::RecommendationStatus,
};

// ────────────────────────────────────────────────────────────────────────────
// Real-pipeline harness (mirrors the runtime's PlanAnalysisService exactly:
// real analyzer + NaiveCollisionChecker + DefaultAggregator/DefaultScoringPolicy)
// ────────────────────────────────────────────────────────────────────────────

fn chain(model: RobotModel) -> SerialChain {
    RobotRegistry::create_default(model)
}

fn real_solver(chain: &SerialChain) -> DampedLeastSquaresSolver {
    let fk = ForwardKinematics::new(chain.clone());
    DampedLeastSquaresSolver::new(fk, chain.end_effector().clone(), 500, 1e-6, 0.1)
}

/// The runtime's canonical analysis: `TrajectoryAnalyzer` with the
/// `NaiveCollisionChecker` always enabled (runtime `plan_analysis.rs:87`),
/// aggregated with the default scoring policy, and `report.metrics` populated
/// from the technical analysis exactly as `PlanAnalysisService` does (S1).
fn analyze(
    chain: &SerialChain,
    trajectory: &Trajectory,
) -> thalos_core::analysis::report::AnalysisReport {
    let checker = NaiveCollisionChecker;
    let matrix = CollisionMatrix::new();
    let analyzer = TrajectoryAnalyzer::new(chain, None).with_collision_checker(&checker, &matrix);
    let artifact = ArtifactRef::MotionPlan(MotionPlanId("usability-intelligence".to_string()));
    let (analysis, observations) = analyzer
        .analyze_with_observations(artifact.clone(), trajectory)
        .expect("real analysis must succeed");
    // Metrics flow INTO the aggregator (design ADR-1): they populate
    // `report.metrics` and feed the summary's continuous-quality component.
    DefaultAggregator::new(DefaultScoringPolicy).aggregate_with_metrics(
        artifact,
        observations,
        analysis.metrics.to_btree_map(),
    )
}

/// Compile a program with the REAL IK solver from a given start configuration.
fn compile(
    chain: &SerialChain,
    start: &[f64],
    program: &PlanningProgram,
) -> Result<thalos_core::trajectory::Trajectory, String> {
    let solver = real_solver(chain);
    let state = RobotState::new(start.to_vec());
    let ctx = SegmentPlanningContext {
        robot: chain,
        current_state: &state,
        ik_solver: &solver,
        tcp: None,
    };
    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
    compiler
        .compile(program, &ctx)
        .map(|p| p.merged_trajectory)
        .map_err(|e| e.to_string())
}

/// The full real loop used by the web UI: compile → analyze → recommend.
fn analyze_and_recommend(
    chain: &SerialChain,
    start: &[f64],
    program: &PlanningProgram,
) -> (
    f64,
    Vec<thalos_core::analysis::observation::Observation>,
    Vec<thalos_planning::recommendation::Recommendation>,
) {
    let trajectory =
        compile(chain, start, program).expect("program must compile for recommendation flow");
    let report = analyze(chain, &trajectory);
    let recommendations =
        PlanAdvisor.recommend(&report.observations, program, &real_solver(chain), start);
    (
        report.summary.quality_index,
        report.observations,
        recommendations,
    )
}

fn hand_trajectory(waypoints: &[Vec<f64>]) -> Trajectory {
    Trajectory::new(
        waypoints
            .iter()
            .enumerate()
            .map(|(i, joints)| TrajectoryPoint::new(joints.clone(), i as f64))
            .collect(),
    )
}

fn count_observations(
    observations: &[thalos_core::analysis::observation::Observation],
    kind: ObservationKind,
) -> usize {
    observations.iter().filter(|o| o.kind == kind).count()
}

fn singular_errors(observations: &[thalos_core::analysis::observation::Observation]) -> usize {
    count_observations(observations, ObservationKind::Singularity)
}

fn movej(target: Vec<f64>) -> MotionSegment {
    MotionSegment::MoveJ {
        origin: OperationId("op-j".to_string()),
        target,
        max_velocity: None,
        max_acceleration: None,
    }
}

fn movel(target: [f64; 3]) -> MotionSegment {
    MotionSegment::MoveL {
        origin: OperationId("op-l".to_string()),
        frame: FrameId::World,
        target_pose: Pose::new(
            FrameId::World,
            FrameId::Id(1),
            Transform3D::from_translation(thalos_math::Vector3::new(
                target[0], target[1], target[2],
            )),
        ),
        max_velocity: None,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// TEST GROUP 1 — Score reality (BUG 1)
// ────────────────────────────────────────────────────────────────────────────
//
// Real analyzer on Planar2R. A waypoint at `[0.0, 0.0]` (full extension) has
// Jacobian condition number >= 1000 → exactly one `Singularity` Error. Inter-
// leaving high-manipulability waypoints keeps `avg_manipulability >= 0.3` so
// no `LowManipulability` warning pollutes the count.

const MANIP_GOOD: [f64; 2] = [0.5, 1.57]; // sin(q2) ≈ 1.0 → yoshikawa ≈ 1.0
const SINGULAR: [f64; 2] = [0.0, 0.0]; // fully extended → singular

mod score_reality {
    use super::*;

    #[test]
    fn perfect_plan_scores_excellent() {
        // A trajectory with no problematic waypoints must score > 0.9 and
        // grade Excellent.
        let robot = chain(RobotModel::Planar2R);
        let traj = hand_trajectory(&[
            MANIP_GOOD.to_vec(),
            [0.5, 1.2].to_vec(),
            [0.5, 0.9].to_vec(),
        ]);
        let report = analyze(&robot, &traj);
        assert!(
            report.observations.is_empty(),
            "perfect trajectory must produce no observations, got {:?}",
            report
                .observations
                .iter()
                .map(|o| o.kind)
                .collect::<Vec<_>>()
        );
        assert!(
            report.summary.quality_index > 0.9,
            "perfect plan must score > 0.9, got {}",
            report.summary.quality_index
        );
        assert_eq!(report.summary.grade, Grade::Excellent);
    }

    #[test]
    fn error_counts_differentiate_scores() {
        // 0,1,2,3,4 Singularity Errors must produce DIFFERENT scores, and a
        // plan with EXACTLY ONE singularity Error must score ≈ 0.70
        // (1 − 0.30 Error penalty).
        let robot = chain(RobotModel::Planar2R);
        let cases: Vec<(usize, f64)> =
            [(0, 1.0), (1, 0.70), (2, 0.40), (3, 0.10), (4, 0.0)].to_vec();

        let mut observed = Vec::new();
        for (n_errors, expected) in &cases {
            // n_errors singular waypoints surrounded by good waypoints so the
            // only observations are the Singularity Errors.
            let mut wps: Vec<Vec<f64>> = vec![MANIP_GOOD.to_vec(); 4];
            for _ in 0..*n_errors {
                wps.push(SINGULAR.to_vec());
            }
            for _ in 0..4 {
                wps.push(MANIP_GOOD.to_vec());
            }
            let report = analyze(&robot, &hand_trajectory(&wps));
            let errs = singular_errors(&report.observations);
            assert_eq!(
                errs, *n_errors,
                "scenario with {n_errors} errors must analyze to exactly {n_errors} Singularity Errors, got {errs}"
            );
            assert!(
                (report.summary.quality_index - expected).abs() < 1e-9,
                "{n_errors} Singularity Errors must score ≈ {expected}, got {}",
                report.summary.quality_index
            );
            observed.push(report.summary.quality_index);
        }

        // All five must be distinct (no premature saturation).
        let distinct: std::collections::HashSet<u64> =
            observed.iter().map(|s| (s * 1e6) as u64).collect();
        assert_eq!(
            distinct.len(),
            observed.len(),
            "0..=4 Error counts must produce different scores, got {observed:?}"
        );
    }

    #[test]
    fn identical_severity_counts_but_different_quality_must_score_differently() {
        // BUG 1 (dead metrics): `quality_index` sums ONLY severity counts
        // (`scoring.rs:47-60`); `PlanMetrics` / `MetricKind::default_weight`
        // (`metrics.rs`) never feed the score. Two plans with the same
        // observations but very different continuous quality MUST score
        // differently — on current code they both score 0.40, so this FAILS.
        let robot = chain(RobotModel::Planar2R);

        // Plan A — dexterous: singular waypoints surrounded by max-
        // manipulability configurations (yoshikawa ≈ 1.0).
        let dexterous = hand_trajectory(&[
            MANIP_GOOD.to_vec(),
            SINGULAR.to_vec(),
            MANIP_GOOD.to_vec(),
            SINGULAR.to_vec(),
            MANIP_GOOD.to_vec(),
            MANIP_GOOD.to_vec(),
            MANIP_GOOD.to_vec(),
            MANIP_GOOD.to_vec(),
        ]);

        // Plan B — stiff: the SAME two singular waypoints, but surrounded by
        // low (yet above-threshold) manipulability configurations
        // (q2 = 0.5 → yoshikawa ≈ 0.48, avg stays >= 0.3 so no warning).
        let stiff = hand_trajectory(&[
            [0.5, 0.5].to_vec(),
            SINGULAR.to_vec(),
            [0.5, 0.5].to_vec(),
            SINGULAR.to_vec(),
            [0.5, 0.5].to_vec(),
            [0.5, 0.5].to_vec(),
            [0.5, 0.5].to_vec(),
            [0.5, 0.5].to_vec(),
        ]);

        let report_a = analyze(&robot, &dexterous);
        let report_b = analyze(&robot, &stiff);

        // Precondition: identical severity counts (both exactly 2 Errors).
        assert_eq!(singular_errors(&report_a.observations), 2);
        assert_eq!(singular_errors(&report_b.observations), 2);
        assert_eq!(report_a.observations.len(), report_b.observations.len());

        // The underlying continuous quality genuinely differs.
        let avg_a = report_a.metrics["avg_manipulability"];
        let avg_b = report_b.metrics["avg_manipulability"];
        assert!(
            avg_a > avg_b,
            "precondition: dexterous plan must have higher average manipulability (A={avg_a:.3}, B={avg_b:.3})"
        );

        // The score MUST reflect that difference.
        assert!(
            (report_a.summary.quality_index - report_b.summary.quality_index).abs() > 1e-9,
            "BUG 1: identical severity counts produce identical scores ({}) despite avg_manipulability {:.3} vs {:.3} — the continuous metrics are dead code",
            report_a.summary.quality_index,
            avg_a,
            avg_b
        );
    }

    #[test]
    fn four_or_more_errors_clamp_to_zero() {
        // Documented saturation behavior (BUG 1): 4 × Error penalty 0.30 = 1.2
        // > 1.0, so `max(0, 1 − Σ)` clamps to exactly 0.0. A 5th error is
        // indistinguishable from the 4th.
        let robot = chain(RobotModel::Planar2R);
        let mut wps: Vec<Vec<f64>> = vec![MANIP_GOOD.to_vec(); 4];
        for _ in 0..4 {
            wps.push(SINGULAR.to_vec());
        }
        let report = analyze(&robot, &hand_trajectory(&wps));
        assert_eq!(
            report.summary.quality_index, 0.0,
            "4 Error observations must clamp the score to exactly 0.0 (saturation)"
        );
    }
}

// ────────────────────────────────────────────────────────────────────────────
// TEST GROUP 2 — Recommendations real applicability (BUG 2)
// ────────────────────────────────────────────────────────────────────────────
//
// Real program: Planar3R, segment 0 MoveJ then segment 1 MoveL (NOT segment
// 0). The second segment travels near full reach (r ≈ 2.35 / 3.0) → the
// trajectory is singular, and the advisor emits a RotateTool (Singularity)
// recommendation for segment 1.

const P3R_SEG0_TARGET: [f64; 3] = [0.5, -0.3, 0.1];
const P3R_NEAR_REACH_TARGET: [f64; 3] = [2.3, 0.5, 0.0];
const P3R_START: [f64; 3] = [0.0, 0.0, 0.0];

fn planar3r_near_reach_program() -> PlanningProgram {
    PlanningProgram::new(vec![
        movej(P3R_SEG0_TARGET.to_vec()),
        movel(P3R_NEAR_REACH_TARGET),
    ])
}

mod recommendation_applicability {
    use super::*;

    #[test]
    fn every_available_recommendation_must_recompile() {
        // For EVERY recommendation marked `available`, applying its edit to a
        // clone of the program and recompiling with the REAL IK solver must
        // succeed. A recommendation the user can click must never produce a
        // plan that cannot be compiled.
        let robot = chain(RobotModel::Planar3R);
        let program = planar3r_near_reach_program();
        let (score, _observations, recommendations) =
            analyze_and_recommend(&robot, &P3R_START, &program);

        let available: Vec<_> = recommendations
            .iter()
            .filter(|r| r.status == Some(RecommendationStatus::Available))
            .collect();
        assert!(
            !available.is_empty(),
            "precondition: the singular segment must yield an available recommendation (score {score:.2})"
        );

        for rec in &available {
            let edited = rec
                .edit
                .apply(&program)
                .expect("an available edit must apply to the program clone");
            let result = compile(&robot, &P3R_START, &edited);
            assert!(
                result.is_ok(),
                "BUG 2: recommendation {} (kind {:?}) is marked `available` but its edited program does not recompile: {}",
                rec.id.0,
                rec.action.kind,
                result.err().unwrap()
            );
        }
    }

    #[test]
    fn no_available_recommendation_may_point_at_an_uncompilable_edit() {
        // The D8 gate must be honest: a recommendation is `available` ONLY if
        // its edited program can actually be compiled. On current code
        // `RotateToolMaterializer::materialize` does no IK verification
        // (`materializer.rs:261-298`), so the gate lies.
        let robot = chain(RobotModel::Planar3R);
        let program = planar3r_near_reach_program();
        let (_score, _observations, recommendations) =
            analyze_and_recommend(&robot, &P3R_START, &program);

        let lying: Vec<String> = recommendations
            .iter()
            .filter(|r| r.status == Some(RecommendationStatus::Available))
            .filter_map(|r| {
                let edited = r.edit.apply(&program).ok()?;
                compile(&robot, &P3R_START, &edited).err().map(|e| {
                    format!(
                        "recommendation {} (kind {:?}) is available but recompile fails: {e}",
                        r.id.0, r.action.kind
                    )
                })
            })
            .collect();

        assert!(
            lying.is_empty(),
            "BUG 2 (D8 gate lies): {} available recommendation(s) point at edits that cannot be compiled:\n  {}",
            lying.len(),
            lying.join("\n  ")
        );
    }

    #[test]
    fn movel_on_planar_2r_needs_the_position_only_fallback() {
        // `MoveLPlanner::plan` (`move_l.rs:99-110`) IK-solves EVERY
        // intermediate Cartesian waypoint with `IKGoal::Pose` and fails on
        // `MaxIterations` — there is NO position-only fallback for inter-
        // mediates (the fallback exists only in `plan_position`, which drives
        // every waypoint with `IKGoal::Position`).
        //
        // Capability baseline first: the SAME robot reaches the SAME
        // translation through the explicit position-only path (`MoveLPosition`
        // → `plan_position`). This proves the geometry is capable and the
        // failure below is purely the missing intermediate fallback.
        let robot = chain(RobotModel::Planar2R);
        let start = vec![0.5, 1.2];
        let translation = [1.5, 0.5, 0.0];

        let position_only = PlanningProgram::new(vec![
            movej(vec![0.5, 1.2]),
            MotionSegment::MoveLPosition {
                origin: OperationId("op-lp".to_string()),
                frame: FrameId::World,
                target_position: translation,
                max_velocity: None,
            },
        ]);
        let position_result = compile(&robot, &start, &position_only);
        assert!(
            position_result.is_ok(),
            "precondition: the position-only path must reach the translation, failed: {}",
            position_result.err().unwrap()
        );

        // A user-authored MoveL for that same translation MUST compile through
        // the position fallback for its intermediates. On current code the
        // pose-constrained intermediates die with the exact user-facing error.
        let pose_program = PlanningProgram::new(vec![movej(vec![0.5, 1.2]), movel(translation)]);
        let pose_result = compile(&robot, &start, &pose_program);
        assert!(
            pose_result.is_ok(),
            "BUG 2: a MoveL to a reachable translation must compile via the position-only intermediate fallback, but failed: {}",
            pose_result.err().unwrap()
        );
    }
}

// ────────────────────────────────────────────────────────────────────────────
// TEST GROUP 3 — Edits actually improve (BUG 3)
// ────────────────────────────────────────────────────────────────────────────

mod edits_improve {
    use super::*;

    #[test]
    fn exactly_one_singularity_error_scores_070_and_removing_it_restores_100() {
        // Baseline for the edit-improvement contract: a plan with exactly one
        // Singularity Error scores 0.70, and removing that ONE observation
        // must recover exactly its 0.30 penalty (score → 1.0).
        let robot = chain(RobotModel::Planar2R);
        let traj = hand_trajectory(&[MANIP_GOOD.to_vec(), SINGULAR.to_vec(), [0.5, 1.2].to_vec()]);
        let report = analyze(&robot, &traj);
        assert_eq!(singular_errors(&report.observations), 1);
        assert!(
            (report.summary.quality_index - 0.70).abs() < 1e-9,
            "a plan with exactly one Singularity Error must score 0.70, got {}",
            report.summary.quality_index
        );

        // Re-aggregation without the singular observation must recover the
        // full 0.30 penalty — the exact delta a real remediation should earn.
        let without_singular: Vec<_> = report
            .observations
            .iter()
            .filter(|o| o.kind != ObservationKind::Singularity)
            .cloned()
            .collect();
        let healed = DefaultAggregator::new(DefaultScoringPolicy)
            .aggregate(report.artifact.clone(), without_singular);
        assert!(
            healed.summary.quality_index > 0.99,
            "removing the only Error must restore score to ~1.0, got {}",
            healed.summary.quality_index
        );
    }

    #[test]
    fn singularity_recommendation_removes_observation_and_improves_score() {
        // A SCARA MoveJ that CROSSES the full extension (elbow goes from the
        // bent home to a positive-elbow target) has an interior singularity.
        // The re-solve materializer re-solves IK from the home (same side) to
        // the alternate elbow posture, which must: (a) compile, (b) remove the
        // Singularity observation, and (c) strictly improve the score.
        let robot = chain(RobotModel::Scara);
        let program = PlanningProgram::new(vec![
            movej(vec![0.5, 0.6, -0.15, 0.0]),
        ]);
        // Non-singular home: elbow bent to the negative side, base ~0.
        let start = vec![0.0, -1.31, -0.1, 0.0];

        let trajectory = compile(&robot, &start, &program).expect("original must compile");
        let report = analyze(&robot, &trajectory);
        let errors_before = singular_errors(&report.observations);
        assert!(
            errors_before > 0,
            "precondition: scenario must produce Singularity Errors (crossing the extension)"
        );

        let recommendations =
            PlanAdvisor.recommend(&report.observations, &program, &real_solver(&robot), &start);
        let singularity = recommendations
            .iter()
            .find(|r| {
                r.action.kind == thalos_core::analysis::action::ActionKind::Singularity
                    && r.status == Some(RecommendationStatus::Available)
            })
            .expect("a Singularity remediation must be available");

        // (a) The edited program must compile.
        let edited = singularity.edit.apply(&program).expect("edit must apply");
        let edited_trajectory = compile(&robot, &start, &edited)
            .unwrap_or_else(|e| panic!("(a) edited program must recompile: {e}"));

        // (b) The Singularity observation must be gone after re-analysis.
        let healed = analyze(&robot, &edited_trajectory);
        let errors_after = singular_errors(&healed.observations);
        assert!(
            errors_after < errors_before,
            "(b) the Singularity remediation must remove the Singularity observation(s): {errors_before} -> {errors_after}"
        );

        // (c) The score must strictly improve.
        assert!(
            healed.summary.quality_index > report.summary.quality_index,
            "(c) the score must strictly improve: {} -> {}",
            report.summary.quality_index,
            healed.summary.quality_index
        );
    }

    #[test]
    fn applying_full_recommendation_set_improves_score_by_the_removed_penalty() {
        // A SCARA program crossing the extension: applying the recommendation
        // set must improve the score by removing the Singularity observation.
        let robot = chain(RobotModel::Scara);
        let program = PlanningProgram::new(vec![
            movej(vec![0.5, 0.6, -0.15, 0.0]),
        ]);
        let start = vec![0.0, -1.31, -0.1, 0.0];

        let trajectory = compile(&robot, &start, &program).expect("original must compile");
        let report = analyze(&robot, &trajectory);
        let errors_before = singular_errors(&report.observations);
        assert!(
            errors_before > 0,
            "precondition: scenario must produce Singularity Errors"
        );

        let recommendations =
            PlanAdvisor.recommend(&report.observations, &program, &real_solver(&robot), &start);
        let available: Vec<_> = recommendations
            .iter()
            .filter(|r| r.status == Some(RecommendationStatus::Available))
            .collect();
        assert!(
            !available.is_empty(),
            "precondition: at least one available recommendation"
        );

        let mut edited = program.clone();
        for rec in &available {
            edited = rec.edit.apply(&edited).expect("edit must apply");
        }
        let healed_trajectory = compile(&robot, &start, &edited)
            .unwrap_or_else(|e| panic!("edited program must compile: {e}"));
        let healed = analyze(&robot, &healed_trajectory);

        assert!(
            healed.summary.quality_index > report.summary.quality_index,
            "applying the recommendation set must improve the score, but it stayed at {} ({} Errors) -> {} ({} Errors)",
            report.summary.quality_index,
            errors_before,
            healed.summary.quality_index,
            singular_errors(&healed.observations),
        );
    }
}
