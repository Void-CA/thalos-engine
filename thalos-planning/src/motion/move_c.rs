use thalos_core::{
    kinematics::{
        forward::ForwardKinematics,
        inverse::{IKGoal, IKStatus},
    },
    trajectory::{Trajectory, TrajectoryPoint},
};
use thalos_math::Vector3;
use thalos_math::traits::{Cross, Dot};

use crate::{
    error::PlanningError,
    goal::{ResolvedPositionGoal, ValidatedGoal},
    motion::{
        planner::{PlanningContext, PlanningResult},
        profile,
    },
};

#[derive(Debug, Clone)]
pub struct MoveCConfig {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub time_step: f64,
    pub cartesian_step: f64,
}

impl Default for MoveCConfig {
    fn default() -> Self {
        Self {
            max_velocity: 0.25,
            max_acceleration: 0.125,
            time_step: 0.01,
            cartesian_step: 0.01,
        }
    }
}

/// Plans a cartesian **circular** move through a via point to a target.
///
/// The arc is the unique circle through (current TCP, `via`, `target`),
/// traversed so that it passes through the via point. Positions are sampled
/// along the arc and resolved to joints with position-only IK (like
/// `MoveLPosition`), timed with the same trapezoidal cartesian profile.
pub struct MoveCPlanner {
    pub config: MoveCConfig,
}

impl MoveCPlanner {
    pub fn new(config: MoveCConfig) -> Self {
        Self { config }
    }

    pub fn plan(
        &self,
        ctx: &PlanningContext,
        via: Vector3,
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

        // Circumcenter of the triangle (start, via, target).
        let a = start - target;
        let b = via - target;
        let n = a.cross(b);
        let n2 = n.dot(n);
        if n2 < 1e-18 {
            return Err(PlanningError::InvalidGoal(
                "movec: start, via and target are collinear (no unique arc)".into(),
            ));
        }
        let a2 = a.dot(a);
        let b2 = b.dot(b);
        let center = target + ((b * a2 - a * b2).cross(n)) * (1.0 / (2.0 * n2));

        let radius = (start - center).magnitude();
        if radius < 1e-12 {
            return Err(PlanningError::InvalidGoal(
                "movec: degenerate arc (zero radius)".into(),
            ));
        }

        let w = n
            .normalized()
            .map_err(|_| PlanningError::InvalidGoal("movec: degenerate arc plane".into()))?;
        let u = (start - center)
            .normalized()
            .map_err(|_| PlanningError::InvalidGoal("movec: degenerate start radius".into()))?;
        let v = w.cross(u);

        let angle_of = |p: Vector3| -> f64 {
            let d = p - center;
            d.dot(v).atan2(d.dot(u))
        };

        // Signed sweep so the arc passes through the via point (theta_start = 0).
        let two_pi = std::f64::consts::TAU;
        let wrap = |x: f64| {
            let mut y = x % two_pi;
            if y < 0.0 {
                y += two_pi;
            }
            y
        };
        let d_via = wrap(angle_of(via));
        let mut d_end = wrap(angle_of(target));
        if d_via > d_end {
            d_end -= two_pi; // clockwise
        }
        let sweep = d_end;
        let arc_length = radius * sweep.abs();
        let sweep_sign = if sweep < 0.0 { -1.0 } else { 1.0 };

        let total_time = profile::total_time(
            arc_length,
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
        let mut last_emitted_travelled = 0.0_f64;

        for i in 0..=num_points {
            let t = if num_points == 0 {
                0.0
            } else {
                i as f64 * (total_time / num_points as f64)
            };
            let is_last = i == num_points;
            let travelled = if is_last {
                arc_length
            } else {
                profile::distance_at(
                    t,
                    arc_length,
                    self.config.max_velocity,
                    self.config.max_acceleration,
                    total_time,
                )
            };

            if is_last {
                // Use the validated resolved target state — IK was already paid.
                q_current = goal.goal.state.positions().ok_or_else(|| {
                    PlanningError::InvalidContext("Goal state missing joint positions".into())
                })?;
            } else {
                let angle = (travelled / radius) * sweep_sign;
                let position = center + u * (radius * angle.cos()) + v * (radius * angle.sin());

                let ik_result = ctx
                    .ik_solver
                    .solve(&q_current, IKGoal::Position(position))?;
                match ik_result.status {
                    IKStatus::Converged => {
                        q_current = ik_result.q;
                    }
                    IKStatus::MaxIterations => {
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
