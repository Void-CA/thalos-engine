//! Temporal resolution: `MotionConstraints` (intent) → `EffectiveMotionProfile`
//! (what the planner will actually sample).
//!
//! The planner owns the RESOLUTION, not the capability. Today there is no robot
//! profile and no safety authority, so a missing request falls back to the
//! historical PLANNER defaults below — which are NOT robot capabilities. This
//! step introduces the model without changing behaviour: with every request
//! absent, the effective profile equals the legacy constants exactly.

use thalos_core::motion::MotionConstraints;

/// Planning time resolution (seconds). This is a sampling cadence, NOT a
/// velocity, and it is independent of the execution/observation/render rates.
pub const DEFAULT_TIME_STEP: f64 = 0.01;

/// Historical planner defaults. Used ONLY when the movement does not request a
/// value. These are PLANNER defaults, deliberately NOT exposed as robot
/// capabilities: a robot profile is a separate, future concept.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannerDefaults {
    pub velocity: f64,
    pub acceleration: f64,
}

impl PlannerDefaults {
    pub const fn new(velocity: f64, acceleration: f64) -> Self {
        Self {
            velocity,
            acceleration,
        }
    }
}

/// Defaults currently in effect for a JOINT-space move (MoveJ family).
pub const MOVE_J_DEFAULTS: PlannerDefaults = PlannerDefaults::new(1.0, 0.5);

/// Defaults currently in effect for a CARTESIAN move (MoveL / MoveC family).
pub const MOVE_CARTESIAN_DEFAULTS: PlannerDefaults = PlannerDefaults::new(0.25, 0.125);

/// The profile the planner will actually sample — always concrete.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EffectiveMotionProfile {
    pub velocity: f64,
    pub acceleration: f64,
    pub time_step: f64,
}

/// Resolve the movement's requested constraints against planner defaults.
///
/// A missing request takes the default; a present request is used verbatim.
/// Capability and safety clamping are intentionally NOT part of this step —
/// they belong to a robot profile / safety authority that does not exist yet.
pub fn resolve_profile(
    constraints: MotionConstraints,
    defaults: PlannerDefaults,
) -> EffectiveMotionProfile {
    EffectiveMotionProfile {
        velocity: constraints.velocity.unwrap_or(defaults.velocity),
        acceleration: constraints.acceleration.unwrap_or(defaults.acceleration),
        time_step: DEFAULT_TIME_STEP,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn absent_request_resolves_to_the_legacy_joint_defaults() {
        let profile = resolve_profile(MotionConstraints::NONE, MOVE_J_DEFAULTS);
        assert_eq!(profile.velocity, 1.0);
        assert_eq!(profile.acceleration, 0.5);
        assert_eq!(profile.time_step, DEFAULT_TIME_STEP);
    }

    #[test]
    fn absent_request_resolves_to_the_legacy_cartesian_defaults() {
        let profile = resolve_profile(MotionConstraints::NONE, MOVE_CARTESIAN_DEFAULTS);
        assert_eq!(profile.velocity, 0.25);
        assert_eq!(profile.acceleration, 0.125);
        assert_eq!(profile.time_step, DEFAULT_TIME_STEP);
    }

    #[test]
    fn requested_values_override_the_defaults_verbatim() {
        let constraints = MotionConstraints::new(Some(0.8), Some(0.4));
        let profile = resolve_profile(constraints, MOVE_CARTESIAN_DEFAULTS);
        assert_eq!(profile.velocity, 0.8);
        assert_eq!(profile.acceleration, 0.4);
    }

    #[test]
    fn partial_request_keeps_the_default_for_the_missing_field() {
        let velocity_only = MotionConstraints::new(Some(0.8), None);
        let profile = resolve_profile(velocity_only, MOVE_CARTESIAN_DEFAULTS);
        assert_eq!(profile.velocity, 0.8);
        assert_eq!(profile.acceleration, 0.125);

        let acceleration_only = MotionConstraints::new(None, Some(0.4));
        let profile = resolve_profile(acceleration_only, MOVE_CARTESIAN_DEFAULTS);
        assert_eq!(profile.velocity, 0.25);
        assert_eq!(profile.acceleration, 0.4);
    }

    #[test]
    fn planner_defaults_are_not_capabilities_they_are_plain_values() {
        // Guards the intent: the defaults are just numbers the planner falls
        // back to, not a robot capability contract.
        assert_eq!(MOVE_J_DEFAULTS, PlannerDefaults::new(1.0, 0.5));
        assert_eq!(MOVE_CARTESIAN_DEFAULTS, PlannerDefaults::new(0.25, 0.125));
    }
}
