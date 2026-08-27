use thalos_core::trajectory::Trajectory;

use crate::{
    goal::{JointGoal, ValidatedGoal},
    interpolate::joint,
    motion::planner::{PlanningContext, PlanningResult, SegmentPlanner},
};

#[derive(Debug, Clone)]
pub struct MoveJConfig {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub time_step: f64,
}

impl Default for MoveJConfig {
    fn default() -> Self {
        Self {
            max_velocity: 1.0,
            max_acceleration: 0.5,
            time_step: 0.01,
        }
    }
}

pub struct MoveJPlanner {
    pub config: MoveJConfig,
}

impl MoveJPlanner {
    pub fn new(config: MoveJConfig) -> Self {
        Self { config }
    }
}

impl Default for MoveJPlanner {
    fn default() -> Self {
        Self::new(MoveJConfig::default())
    }
}

impl SegmentPlanner for MoveJPlanner {
    type Goal = ValidatedGoal<JointGoal>;

    fn plan(&self, ctx: &PlanningContext, goal: &ValidatedGoal<JointGoal>) -> PlanningResult {
        let q_start = ctx.current_state.positions().ok_or_else(|| {
            crate::error::PlanningError::InvalidContext("Current state missing joint positions".into())
        })?;
        let target = &goal.goal.as_slice();

        let waypoints = joint::trapezoidal_profile(
            &q_start,
            target,
            self.config.max_velocity,
            self.config.max_acceleration,
            self.config.time_step,
        );

        Ok(Trajectory::new(waypoints))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::goal::{GoalMetadata, PlanningAssessment};
    use thalos_core::{
        kinematics::inverse::{IKGoal, IKResult, IKSolver, IkError},
        models::{RobotModel, RobotRegistry},
        robot::state::RobotState,
    };

    struct NoopIKSolver;

    impl IKSolver for NoopIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
    }

    #[test]
    fn plan_returns_trajectory_with_waypoints() {
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let state = RobotState::zero(2);
        let ik = NoopIKSolver;
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &ik,
            tcp: None,
        };
        let planner = MoveJPlanner::default();
        let goal = ValidatedGoal {
            goal: JointGoal(vec![1.0, 1.0]),
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };
        let traj = planner.plan(&ctx, &goal).expect("plan should succeed");
        assert!(!traj.is_empty(), "trajectory should have waypoints");
    }

    #[test]
    fn plan_starts_and_ends_at_correct_positions() {
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let state = RobotState::zero(2);
        let ik = NoopIKSolver;
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &ik,
            tcp: None,
        };
        let planner = MoveJPlanner::default();
        let target = vec![1.5, -0.5];
        let goal = ValidatedGoal {
            goal: JointGoal(target.clone()),
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };
        let traj = planner.plan(&ctx, &goal).expect("plan should succeed");
        let first = &traj.waypoints()[0];
        let last = &traj.waypoints()[traj.len() - 1];
        let start_positions = ctx.current_state.positions().unwrap();
        for (j, s) in first.joints().iter().zip(start_positions.iter()) {
            assert!((j - s).abs() < 1e-10);
        }
        for (j, t) in last.joints().iter().zip(target.iter()) {
            assert!((j - t).abs() < 1e-10);
        }
    }

    // ── Duration regression (follow-up fix) ────────────────────────────────
    //
    // MoveJ plans in RADIANS. The joint-space default (1.0 rad/s, 0.5 rad/s²)
    // keeps a 1.5 rad move at the pre-change triangular ~3.5s; the CARTESIAN
    // demo default (0.1 m/s) copied into a MoveJ would mean 0.1 rad/s and a
    // ~15s move — the defect the joint/cartesian profile split prevents.

    fn plan_duration(config: MoveJConfig, target: Vec<f64>) -> f64 {
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let state = RobotState::zero(2);
        let ik = NoopIKSolver;
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &ik,
            tcp: None,
        };
        let planner = MoveJPlanner::new(config);
        let goal = ValidatedGoal {
            goal: JointGoal(target),
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };
        let traj = planner.plan(&ctx, &goal).expect("plan should succeed");
        traj.waypoints().last().expect("waypoints").timestamp()
    }

    #[test]
    fn movej_duration_at_joint_profile_is_triangular_3_5s_for_1_5_rad() {
        // 1.5 rad @ 1.0/0.5: 2·d_acc = 2.0 >= 1.5 → triangular
        // T = 2·sqrt(d/a) = 2·sqrt(3) ≈ 3.46s — the pre-change behavior.
        let duration = plan_duration(
            MoveJConfig {
                max_velocity: 1.0,
                max_acceleration: 0.5,
                time_step: 0.01,
            },
            vec![1.5, 0.0],
        );
        let expected = 2.0 * (1.5_f64 / 0.5_f64).sqrt();
        assert!(
            (duration - expected).abs() < 0.05,
            "1.5 rad @ 1.0/0.5 must be ≈3.46s (triangular), got {duration:.3}s"
        );
        assert!(
            duration < 5.0,
            "must NOT regress to the ~15s cartesian-rate duration"
        );
    }

    #[test]
    fn movej_duration_at_cartesian_profile_is_15s_for_1_5_rad() {
        // The defect the follow-up prevents: 0.1 rad/s over 1.5 rad with a
        // 0.5 rad/s² limit is trapezoidal — T = 2·(v/a) + (d − 2·d_acc)/v
        // = 0.4 + 14.8 = 15.2s.
        let duration = plan_duration(
            MoveJConfig {
                max_velocity: 0.1,
                max_acceleration: 0.5,
                time_step: 0.01,
            },
            vec![1.5, 0.0],
        );
        let d_acc = (0.1 * 0.1) / (2.0 * 0.5);
        let expected = 2.0 * (0.1 / 0.5) + (1.5 - 2.0 * d_acc) / 0.1;
        assert!(
            (duration - expected).abs() < 0.05,
            "1.5 rad @ 0.1/0.5 must be ≈15.2s (trapezoidal), got {duration:.3}s"
        );
    }
}
