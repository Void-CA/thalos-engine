use thalos_core::{
    kinematics::{
        forward::ForwardKinematics,
        inverse::{IKGoal, IKStatus},
    },
    spatial::pose::Pose,
};

use thalos_core::trajectory::{Trajectory, TrajectoryPoint};

use crate::{
    error::PlanningError,
    goal::{ResolvedPoseGoal, ResolvedPositionGoal, ValidatedGoal},
    interpolate::cartesian,
    motion::{
        planner::{PlanningContext, PlanningResult, SegmentPlanner},
        profile,
    },
};
use thalos_math::Vector3;

#[derive(Debug, Clone)]
pub struct MoveLConfig {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub time_step: f64,
    pub cartesian_step: f64,
}

impl Default for MoveLConfig {
    fn default() -> Self {
        Self {
            max_velocity: 0.25,
            max_acceleration: 0.125,
            time_step: 0.01,
            cartesian_step: 0.01,
        }
    }
}

pub struct MoveLPlanner {
    pub config: MoveLConfig,
}

impl MoveLPlanner {
    pub fn new(config: MoveLConfig) -> Self {
        Self { config }
    }
}

impl Default for MoveLPlanner {
    fn default() -> Self {
        Self::new(MoveLConfig::default())
    }
}

impl SegmentPlanner for MoveLPlanner {
    type Goal = ValidatedGoal<ResolvedPoseGoal>;

    fn plan(
        &self,
        ctx: &PlanningContext,
        goal: &ValidatedGoal<ResolvedPoseGoal>,
    ) -> PlanningResult {
        let target_pose = &goal.goal.pose;

        let q_start = ctx.current_state.positions().ok_or_else(|| {
            PlanningError::InvalidContext("Current state missing joint positions".into())
        })?;

        let fk = ForwardKinematics::new(ctx.robot.clone());
        let fk_result = fk.evaluate(&q_start);
        let start_pose = fk_result.ee_pose().ok_or_else(|| {
            PlanningError::InvalidGoal("End-effector pose not found in FK result".into())
        })?;
        let start_transform = start_pose.transform().clone();
        let end_transform = target_pose.transform().clone();

        // Cartesian travel distance along the straight path.
        let distance = (end_transform.translation - start_transform.translation).magnitude();

        // Spec move-l-velocity-profile: sample TIME from the trapezoidal
        // cartesian profile (time → distance(t) → position(t)) — waypoints and
        // timestamps share ONE source of truth, and `max_acceleration` is
        // consumed here (short moves fall back to the triangular profile).
        // The cadence is uniform (`total_time / num_points`): the final sample
        // lands EXACTLY on the profile end — no degenerate sub-step interval.
        let total_time = profile::total_time(
            distance,
            self.config.max_velocity,
            self.config.max_acceleration,
        );
        let time_step = self.config.time_step.max(1e-9);
        let num_points = if total_time < 1e-12 {
            0
        } else {
            (total_time / time_step).ceil() as usize
        };

        let mut q_current = q_start.clone();
        let mut trajectory = Trajectory::new(Vec::with_capacity(num_points + 1));
        // Distance (along the path) of the last EMITTED waypoint. Used to
        // detect sub-resolution waypoints at workspace-boundary dead zones.
        let mut last_emitted_travelled = 0.0_f64;

        for i in 0..=num_points {
            let t = if num_points == 0 {
                0.0
            } else {
                i as f64 * (total_time / num_points as f64)
            };
            let is_last = i == num_points;
            let travelled = if is_last {
                distance
            } else {
                profile::distance_at(
                    t,
                    distance,
                    self.config.max_velocity,
                    self.config.max_acceleration,
                    total_time,
                )
            };

            if is_last {
                // Use the validated resolved state — the resolver already paid IK + analysis
                q_current = goal.goal.state.positions().ok_or_else(|| {
                    PlanningError::InvalidContext("Goal state missing joint positions".into())
                })?;
            } else {
                let progress = if distance > 1e-12 {
                    travelled / distance
                } else {
                    0.0
                };
                let transform =
                    cartesian::lerp_transform(&start_transform, &end_transform, progress);
                let waypoint_pose = Pose::new(
                    target_pose.reference_id(),
                    target_pose.target_id(),
                    transform,
                );

                let ik_result = ctx
                    .ik_solver
                    .solve(&q_current, IKGoal::Pose(waypoint_pose.clone()))?;

                match ik_result.status {
                    IKStatus::Converged => {
                        q_current = ik_result.q;
                    }
                    IKStatus::MaxIterations => {
                        // Semantic fallback (design ADR-4, spec
                        // semantic-ik-fallback "Position fallback when
                        // operation allows"): a MoveL intermediate whose FULL
                        // pose is unreachable retries translation-only IK —
                        // gated by the operation type (MoveL allows it;
                        // MoveLPosition drives Position from the start). If
                        // the position is ALSO unreachable, the failure is
                        // preserved as IkFailed (orientation-mandatory path)
                        // — UNLESS the waypoint lies within `cartesian_step`
                        // of the last emitted one: that sub-resolution
                        // failure is the pre-existing DLS radial dead zone at
                        // workspace boundaries (the damped Jacobian cannot
                        // reduce the radial error at full extension). The
                        // former spatial sampling never created such close
                        // waypoints; skipping them keeps the profile timing
                        // and the next sample (≥ cartesian_step ahead) kicks
                        // the solver out of the dead zone.
                        let position = waypoint_pose.translation();
                        let position_result = ctx
                            .ik_solver
                            .solve(&q_current, IKGoal::Position(position))?;
                        match position_result.status {
                            IKStatus::Converged => {
                                q_current = position_result.q;
                            }
                            IKStatus::MaxIterations => {
                                if travelled - last_emitted_travelled < self.config.cartesian_step {
                                    continue;
                                }
                                return Err(PlanningError::IkFailed {
                                    target_pose: target_pose.clone(),
                                    reason: crate::error::IkFailureReason::NoSolution,
                                });
                            }
                        }
                    }
                }
            }

            trajectory.push(TrajectoryPoint::new(q_current.clone(), t));
            last_emitted_travelled = travelled;
        }

        Ok(trajectory)
    }
}

impl MoveLPlanner {
    /// Plan a translation-only Cartesian move for a [`ResolvedPositionGoal`].
    ///
    /// Interpolates the translation-only path and drives every intermediate
    /// waypoint with `IKGoal::Position` — never `IKGoal::Pose`. This is what
    /// lets a SCARA (4 DOF, yaw-only) execute MoveL: a full 6-DOF pose goal
    /// leaves irreducible roll/pitch error and dies at `MaxIterations`.
    ///
    /// Position sampling comes from the trapezoidal cartesian profile (spec
    /// `move-l-velocity-profile`): `position(t) = start + (distance(t)/d)·(end
    /// − start)`, timestamp = t — one source of truth.
    pub fn plan_position(
        &self,
        ctx: &PlanningContext,
        goal: &ValidatedGoal<ResolvedPositionGoal>,
    ) -> PlanningResult {
        let target = goal.goal.position;

        let q_start = ctx.current_state.positions().ok_or_else(|| {
            PlanningError::InvalidContext("Current state missing joint positions".into())
        })?;

        let fk = ForwardKinematics::new(ctx.robot.clone());
        let fk_result = fk.evaluate(&q_start);
        let start_pose = fk_result.ee_pose().ok_or_else(|| {
            PlanningError::InvalidGoal("End-effector pose not found in FK result".into())
        })?;
        let start = start_pose.translation();
        let direction = target - start;
        let distance = direction.magnitude();
        let unit = if distance > 1e-12 {
            direction * (1.0 / distance)
        } else {
            Vector3::zero()
        };

        let total_time = profile::total_time(
            distance,
            self.config.max_velocity,
            self.config.max_acceleration,
        );
        let time_step = self.config.time_step.max(1e-9);
        let num_points = if total_time < 1e-12 {
            0
        } else {
            (total_time / time_step).ceil() as usize
        };

        let mut q_current = q_start.clone();
        let mut trajectory = Trajectory::new(Vec::with_capacity(num_points + 1));
        // Distance (along the path) of the last EMITTED waypoint. Used to
        // detect sub-resolution waypoints at workspace-boundary dead zones.
        let mut last_emitted_travelled = 0.0_f64;

        for i in 0..=num_points {
            let t = if num_points == 0 {
                0.0
            } else {
                i as f64 * (total_time / num_points as f64)
            };
            let is_last = i == num_points;
            let travelled = if is_last {
                distance
            } else {
                profile::distance_at(
                    t,
                    distance,
                    self.config.max_velocity,
                    self.config.max_acceleration,
                    total_time,
                )
            };

            if is_last {
                // Use the validated resolved state — the resolver already paid IK + analysis
                q_current = goal.goal.state.positions().ok_or_else(|| {
                    PlanningError::InvalidContext("Goal state missing joint positions".into())
                })?;
            } else {
                let position = start + unit * travelled;

                let ik_result = ctx
                    .ik_solver
                    .solve(&q_current, IKGoal::Position(position))?;

                match ik_result.status {
                    IKStatus::Converged => {
                        q_current = ik_result.q;
                    }
                    IKStatus::MaxIterations => {
                        // Same dead-zone guard as `plan`: a waypoint within
                        // `cartesian_step` of the last emitted one that cannot
                        // be solved is the pre-existing DLS radial dead zone
                        // at workspace boundaries — skip it (bounded position
                        // error < cartesian_step) instead of failing the
                        // whole move. Targets ≥ cartesian_step away still fail
                        // as IkFailedPosition.
                        if travelled - last_emitted_travelled < self.config.cartesian_step {
                            continue;
                        }
                        return Err(PlanningError::IkFailedPosition {
                            target_position: [target.x, target.y, target.z],
                            reason: crate::error::IkFailureReason::NoSolution,
                        });
                    }
                }
            }

            trajectory.push(TrajectoryPoint::new(q_current.clone(), t));
            last_emitted_travelled = travelled;
        }

        Ok(trajectory)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::goal::{GoalMetadata, PlanningAssessment, ResolvedPoseGoal, ResolvedPositionGoal};
    use thalos_core::{
        kinematics::inverse::{DampedLeastSquaresSolver, IKResult, IKSolver, IkError},
        models::{RobotModel, RobotRegistry},
        robot::state::RobotState,
    };
    use thalos_math::Vector3;

    struct NoopIKSolver;

    impl IKSolver for NoopIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
    }

    #[test]
    fn plan_with_noop_ik_returns_trajectory() {
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let state = RobotState::zero(2);
        let ik = NoopIKSolver;
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &ik,
            tcp: None,
        };

        let planner = MoveLPlanner::default();
        let fk = ForwardKinematics::new(robot.clone());
        let result = fk.evaluate(&[0.5, 0.3]);
        let target_pose = result.ee_pose().cloned().unwrap();

        let goal = ValidatedGoal {
            goal: ResolvedPoseGoal {
                pose: target_pose,
                state: RobotState::from_positions(vec![0.5, 0.3]),
            },
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };

        let traj = planner.plan(&ctx, &goal).expect("plan should succeed");
        assert!(!traj.is_empty(), "trajectory should have waypoints");
    }

    /// Position-only MoveL on a SCARA: the planner must interpolate the
    /// translation-only path with `IKGoal::Position` (never `IKGoal::Pose`)
    /// so the 4-DOF, yaw-only robot converges — the exact failure that
    /// produced `422 segment_n_failed` before this fix.
    ///
    /// The move starts from a REACHABLE mid-workspace configuration. (The
    /// previous setup started at `RobotState::zero(4)`, whose FK puts the
    /// SCARA at full extension — a workspace-boundary singular config where
    /// DLS cannot reduce the radial error component. That dead zone is a
    /// pre-existing solver limitation, unrelated to the profile sampling;
    /// this test's intent is the Position-vs-Pose IK mode.)
    #[test]
    fn plan_position_converges_on_scara() {
        let robot = RobotRegistry::create_default(RobotModel::Scara);
        // Well within the SCARA workspace (r_xy = 0.78 > r_min 0.50).
        let start = Vector3::new(0.6, 0.5, 0.25);
        let target = Vector3::new(0.62, 0.5, 0.25);

        let fk = ForwardKinematics::new(robot.clone());
        let solver =
            DampedLeastSquaresSolver::new(fk, robot.end_effector().clone(), 500, 1e-6, 0.1);
        let q_start = solver
            .solve(&[0.0, 0.0, 0.0, 0.0], IKGoal::Position(start))
            .expect("position IK must converge on SCARA")
            .q;
        let q_end = solver
            .solve(&q_start, IKGoal::Position(target))
            .expect("target IK must converge on SCARA")
            .q;

        let state = RobotState::from_positions(q_start.clone());
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &solver,
            tcp: None,
        };

        let goal = ValidatedGoal {
            goal: ResolvedPositionGoal {
                position: target,
                state: RobotState::from_positions(q_end.clone()),
            },
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };

        let planner = MoveLPlanner::default();
        let traj = planner
            .plan_position(&ctx, &goal)
            .expect("position-only MoveL must converge on SCARA");
        assert!(!traj.is_empty(), "trajectory should have waypoints");

        // The last waypoint is the exact goal state (never re-solved).
        assert_eq!(
            traj.waypoints().last().unwrap().joints(),
            q_end.as_slice(),
            "final waypoint must be the goal state"
        );
        let last = traj.waypoints().last().unwrap().joints().to_vec();
        let fk2 = ForwardKinematics::new(robot.clone());
        let ee = fk2.evaluate(&last).ee_pose().unwrap().translation();
        let error = (ee - target).magnitude();
        assert!(
            error < 0.02,
            "EE position error {error:.4} (expected {target:?}, got {ee:?})"
        );
    }

    // ── T9 (M2): semantic intermediate fallback (design ADR-4) ──────────────
    //
    // Spec semantic-ik-fallback "Position fallback when operation allows": a
    // MoveL intermediate whose full pose exhausts `MaxIterations` falls back
    // to translation-only IK for THAT intermediate when the position itself
    // converges. The final pose is resolved before planning (dispatcher), so
    // this fallback covers the path between start and goal only.

    /// Mock solver with the SCARA profile: full-pose IK exhausts
    /// `MaxIterations`, translation-only IK converges.
    struct PoseFailsPositionConvergesIKSolver;

    impl IKSolver for PoseFailsPositionConvergesIKSolver {
        fn solve(&self, q0: &[f64], goal: IKGoal) -> Result<IKResult, IkError> {
            match goal {
                IKGoal::Pose(_) => Ok(IKResult::max_iterations(q0.to_vec(), 100, 1.5, None)),
                IKGoal::Position(_) => Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None)),
            }
        }
    }

    #[test]
    fn plan_falls_back_to_position_ik_for_unreachable_intermediates() {
        // RED (BUG 2): on current code every intermediate is solved with
        // `IKGoal::Pose` and the FIRST MaxIterations kills the plan. With the
        // fallback the same intermediates converge through `IKGoal::Position`.
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let state = RobotState::zero(2);
        let ik = PoseFailsPositionConvergesIKSolver;
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &ik,
            tcp: None,
        };

        let planner = MoveLPlanner::default();
        let fk = ForwardKinematics::new(robot.clone());
        let result = fk.evaluate(&[0.5, 0.3]);
        let target_pose = result.ee_pose().cloned().unwrap();

        let goal = ValidatedGoal {
            goal: ResolvedPoseGoal {
                pose: target_pose,
                state: RobotState::from_positions(vec![0.5, 0.3]),
            },
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };

        let traj = planner
            .plan(&ctx, &goal)
            .expect("intermediate pose failure must fall back to position IK");
        assert!(!traj.is_empty(), "trajectory should have waypoints");

        // The last waypoint is the RESOLVED final state (never re-solved by
        // the planner); the intermediates rode the position fallback.
        let last = traj.waypoints().last().unwrap().joints().to_vec();
        assert_eq!(
            last,
            vec![0.5, 0.3],
            "final waypoint must be the goal state"
        );
    }

    #[test]
    fn plan_still_fails_when_position_fallback_also_fails() {
        // Spec semantic-ik-fallback "Orientation mandatory + unreachable":
        // when BOTH pose and position IK exhaust MaxIterations, the failure
        // is preserved as `PlanningError::IkFailed` — never silently degraded.
        struct FailingIKSolver;
        impl IKSolver for FailingIKSolver {
            fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
                Ok(IKResult::max_iterations(q0.to_vec(), 100, 1.5, None))
            }
        }

        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let state = RobotState::zero(2);
        let ik = FailingIKSolver;
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &ik,
            tcp: None,
        };

        let planner = MoveLPlanner::default();
        let fk = ForwardKinematics::new(robot.clone());
        let result = fk.evaluate(&[0.5, 0.3]);
        let target_pose = result.ee_pose().cloned().unwrap();

        let goal = ValidatedGoal {
            goal: ResolvedPoseGoal {
                pose: target_pose,
                state: RobotState::from_positions(vec![0.5, 0.3]),
            },
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };

        match planner.plan(&ctx, &goal) {
            Err(PlanningError::IkFailed { .. }) => {}
            other => panic!("expected IkFailed when pose AND position fail, got {other:?}"),
        }
    }

    // ── Spec move-l-velocity-profile: trapezoidal cartesian timing ─────────
    //
    // MoveL MUST generate positions FROM the trapezoidal time profile
    // (time → distance(t) → position(t)) and consume max_acceleration. The
    // RED assertions use the MATHEMATICALLY EXPECTED durations (never a weak
    // ">= 100ms" floor): 20mm @ 0.1 m/s, 0.5 m/s² is the triangular boundary
    // (2·d_acc == d) with T = 2·sqrt(d/a) = 0.4s and v_peak = v_max = 0.1.

    /// Real-solver SCARA chain + position solver (DLS, analysis tolerance).
    fn scara_pose_solver() -> (
        thalos_core::robot::serial_chain::SerialChain,
        DampedLeastSquaresSolver,
    ) {
        let robot = RobotRegistry::create_default(RobotModel::Scara);
        let fk = ForwardKinematics::new(robot.clone());
        let solver =
            DampedLeastSquaresSolver::new(fk, robot.end_effector().clone(), 500, 1e-6, 0.1);
        (robot, solver)
    }

    /// Plan a straight cartesian move of `offset` from `start` through the
    /// real SCARA pipeline. Returns the trajectory and the exact target.
    fn plan_position_move(
        start: Vector3,
        offset: Vector3,
        config: MoveLConfig,
    ) -> (Trajectory, Vector3) {
        let (robot, solver) = scara_pose_solver();
        let q_start = solver
            .solve(&[0.0, 0.0, 0.0, 0.0], IKGoal::Position(start))
            .expect("start IK must converge")
            .q;
        let end = start + offset;
        let q_end = solver
            .solve(&q_start, IKGoal::Position(end))
            .expect("end IK must converge")
            .q;

        let state = RobotState::from_positions(q_start.clone());
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &solver,
            tcp: None,
        };
        let goal = ValidatedGoal {
            goal: ResolvedPositionGoal {
                position: end,
                state: RobotState::from_positions(q_end),
            },
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };
        let traj = MoveLPlanner::new(config)
            .plan_position(&ctx, &goal)
            .expect("plan_position must succeed");
        (traj, end)
    }

    /// 20mm @ max_velocity=0.1, max_acceleration=0.5 — the spec boundary case.
    fn twenty_mm_config() -> MoveLConfig {
        MoveLConfig {
            max_velocity: 0.1,
            max_acceleration: 0.5,
            time_step: 0.01,
            cartesian_step: 0.01,
        }
    }

    /// Implied cartesian velocities between consecutive trajectory samples.
    fn implied_velocities(
        robot: &thalos_core::robot::serial_chain::SerialChain,
        traj: &Trajectory,
    ) -> Vec<f64> {
        let fk = ForwardKinematics::new(robot.clone());
        let wps = traj.waypoints();
        let positions: Vec<Vector3> = wps
            .iter()
            .map(|w| fk.evaluate(w.joints()).ee_pose().unwrap().translation())
            .collect();
        positions
            .windows(2)
            .zip(wps.windows(2))
            .map(|(p, w)| (p[1] - p[0]).magnitude() / (w[1].timestamp() - w[0].timestamp()))
            .collect()
    }

    /// (a) 20mm @ 0.1 m/s, 0.5 m/s² → ≈ 400ms (2·sqrt(d/a) = 0.4s), NOT the
    /// constant-velocity 0.2s. Asserted with a ±10ms tolerance — never a weak
    /// ">= 100ms" floor.
    #[test]
    fn twenty_mm_move_takes_about_400ms() {
        let (traj, _) = plan_position_move(
            Vector3::new(0.6, 0.5, 0.25),
            Vector3::new(0.02, 0.0, 0.0),
            twenty_mm_config(),
        );
        let duration = traj.waypoints().last().expect("waypoints").timestamp();
        assert!(
            (duration - 0.4).abs() < 0.01,
            "20mm @ 0.1/0.5 must take ≈0.4s, got {duration:.4}s"
        );
    }

    /// (b) Triangular fallback when 2·d_acc >= d (5mm: 2·d_acc = 0.02 >= d):
    /// T = 2·sqrt(d/a) = 0.2s and the implied peak velocity never exceeds
    /// v_peak = sqrt(d·a) = 0.05 <= v_max.
    #[test]
    fn short_move_uses_triangular_profile() {
        let (traj, _) = plan_position_move(
            Vector3::new(0.6, 0.5, 0.25),
            Vector3::new(0.005, 0.0, 0.0),
            twenty_mm_config(),
        );
        let duration = traj.waypoints().last().expect("waypoints").timestamp();
        assert!(
            (duration - 0.2).abs() < 0.01,
            "triangular T must be 2·sqrt(d/a) = 0.2s, got {duration:.4}s"
        );
        let (robot, _) = scara_pose_solver();
        let velocities = implied_velocities(&robot, &traj);
        let peak = velocities.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            peak <= 0.05 + 2e-3,
            "triangular peak velocity must stay at sqrt(d·a) = 0.05 <= v_max, got {peak:.4}"
        );
    }

    /// (c) Trapezoidal cruise timing when the distance is large enough
    /// (0.1m: 2·d_acc = 0.02 < d): T = 2·(v/a) + (d − 2·d_acc)/v = 1.2s.
    #[test]
    fn long_move_uses_trapezoidal_cruise() {
        let (traj, _) = plan_position_move(
            Vector3::new(0.6, 0.5, 0.25),
            Vector3::new(0.1, 0.0, 0.0),
            twenty_mm_config(),
        );
        let duration = traj.waypoints().last().expect("waypoints").timestamp();
        assert!(
            (duration - 1.2).abs() < 0.01,
            "trapezoidal T must be 1.2s, got {duration:.4}s"
        );
    }

    /// (d) Negative: timestamps are NOT progress·distance/max_velocity. The
    /// constant-velocity schedule would finish in d/v = 0.2s with a linear mid
    /// timestamp of 0.1s — the profile trajectory must deviate from both.
    #[test]
    fn timestamps_are_not_constant_velocity_schedule() {
        let (traj, _) = plan_position_move(
            Vector3::new(0.6, 0.5, 0.25),
            Vector3::new(0.02, 0.0, 0.0),
            twenty_mm_config(),
        );
        let wps = traj.waypoints();
        let duration = wps.last().expect("waypoints").timestamp();
        assert!(
            (duration - 0.02 / 0.1).abs() > 0.05,
            "duration must differ from the constant-velocity d/v = 0.2s"
        );
        let mid = wps[wps.len() / 2].timestamp();
        let linear_mid = 0.5 * 0.02 / 0.1;
        assert!(
            (mid - linear_mid).abs() > 0.02,
            "mid timestamp {mid:.4} must deviate from the linear schedule {linear_mid:.4}"
        );
    }

    /// (e) The last waypoint equals the exact target (goal state, not an IK
    /// re-solve) — FK at it lands on the target to solver precision.
    #[test]
    fn last_waypoint_reaches_the_exact_target() {
        let (traj, end) = plan_position_move(
            Vector3::new(0.6, 0.5, 0.25),
            Vector3::new(0.02, 0.0, 0.0),
            twenty_mm_config(),
        );
        let last = traj.waypoints().last().expect("waypoints");
        let (robot, _) = scara_pose_solver();
        let fk = ForwardKinematics::new(robot.clone());
        let ee = fk.evaluate(last.joints()).ee_pose().unwrap().translation();
        assert!(
            (ee - end).magnitude() < 1e-6,
            "final EE must land on the exact target, error {:.2e}",
            (ee - end).magnitude()
        );
    }

    /// (f) Sampling bounds (spec move-l-profile-sampling-bounds): implied
    /// velocity between samples never exceeds v_max, implied acceleration
    /// never exceeds a_max. Exercised on the 0.1m trapezoidal move where
    /// cruise actually reaches v_max.
    #[test]
    fn implied_velocity_and_acceleration_stay_within_bounds() {
        let (traj, _) = plan_position_move(
            Vector3::new(0.6, 0.5, 0.25),
            Vector3::new(0.1, 0.0, 0.0),
            twenty_mm_config(),
        );
        let wps = traj.waypoints();
        assert!(
            wps.len() >= 10,
            "trajectory must carry real samples, got {}",
            wps.len()
        );
        let (robot, _) = scara_pose_solver();
        let velocities = implied_velocities(&robot, &traj);
        for v in &velocities {
            assert!(
                *v <= 0.1 + 2e-3,
                "implied velocity {v:.4} m/s exceeds v_max"
            );
        }
        let dt = 0.01;
        for w in velocities.windows(2) {
            let a = (w[1] - w[0]).abs() / dt;
            assert!(
                a <= 0.5 + 0.05,
                "implied acceleration {a:.4} m/s² exceeds a_max"
            );
        }
    }

    /// (g) v(0) = v(T) = 0: the first and last sample intervals imply near-zero
    /// velocity (≤ a·dt), never the cruise velocity.
    #[test]
    fn initial_and_final_velocity_are_zero() {
        let (traj, _) = plan_position_move(
            Vector3::new(0.6, 0.5, 0.25),
            Vector3::new(0.1, 0.0, 0.0),
            twenty_mm_config(),
        );
        let (robot, _) = scara_pose_solver();
        let velocities = implied_velocities(&robot, &traj);
        let bound = 0.5 * 0.01 * 2.0; // a·dt with margin
        assert!(
            velocities[0] < bound,
            "v(0) must be near zero, got {:.4}",
            velocities[0]
        );
        assert!(
            velocities[velocities.len() - 1] < bound,
            "v(T) must be near zero, got {:.4}",
            velocities[velocities.len() - 1]
        );
    }
}
