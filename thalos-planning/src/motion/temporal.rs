//! Temporal resolution: `MotionConstraints` (intent) → `EffectiveMotionProfile`
//! (what the planner will actually sample).
//!
//! The planner owns the RESOLUTION, not the capability. A movement that does
//! not request a value falls back to the planner's [`MotionDefaults`] — which
//! are planner configuration, deliberately NOT robot capabilities. The planner
//! does not know where the defaults come from; it only receives them, so a
//! robot profile / safety authority can be introduced later without touching
//! [`EffectiveMotionProfile`].

use thalos_core::motion::MotionConstraints;

/// Planning time resolution (seconds). This is a sampling cadence, NOT a
/// velocity, and it is independent of the execution/observation/render rates.
pub const DEFAULT_TIME_STEP: f64 = 0.01;

/// Default motion values for one motion space (joint or cartesian).
///
/// These are PLANNER defaults: the values used when the program does not
/// request a constraint. They are intentionally NOT exposed as robot
/// capabilities — a robot profile is a separate, future concept.
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

/// The planner's configurable motion defaults, one set per motion space.
///
/// The joint and cartesian spaces are separate because `movej` and `movel`
/// do not share a velocity concept (articular vs cartesian). Changing one must
/// not change the other.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionDefaults {
    /// Defaults for joint-space moves (MoveJ family).
    pub joint: PlannerDefaults,
    /// Defaults for cartesian moves (MoveL / MoveC family).
    pub cartesian: PlannerDefaults,
}

impl MotionDefaults {
    pub const fn new(joint: PlannerDefaults, cartesian: PlannerDefaults) -> Self {
        Self { joint, cartesian }
    }
}

impl Default for MotionDefaults {
    /// The historical planner defaults, preserved verbatim so that introducing
    /// the configuration does not change any existing execution.
    fn default() -> Self {
        Self {
            joint: PlannerDefaults::new(1.0, 0.5),
            cartesian: PlannerDefaults::new(0.25, 0.125),
        }
    }
}

/// The profile the planner will actually sample — always concrete.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EffectiveMotionProfile {
    pub velocity: f64,
    pub acceleration: f64,
    pub time_step: f64,
}

/// Resolve the movement's requested constraints against a set of planner
/// defaults.
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
    fn default_motion_defaults_are_the_legacy_values() {
        let defaults = MotionDefaults::default();
        assert_eq!(defaults.joint, PlannerDefaults::new(1.0, 0.5));
        assert_eq!(defaults.cartesian, PlannerDefaults::new(0.25, 0.125));
    }

    #[test]
    fn absent_request_resolves_to_the_legacy_joint_defaults() {
        let profile = resolve_profile(MotionConstraints::NONE, MotionDefaults::default().joint);
        assert_eq!(profile.velocity, 1.0);
        assert_eq!(profile.acceleration, 0.5);
        assert_eq!(profile.time_step, DEFAULT_TIME_STEP);
    }

    #[test]
    fn absent_request_resolves_to_the_legacy_cartesian_defaults() {
        let profile =
            resolve_profile(MotionConstraints::NONE, MotionDefaults::default().cartesian);
        assert_eq!(profile.velocity, 0.25);
        assert_eq!(profile.acceleration, 0.125);
        assert_eq!(profile.time_step, DEFAULT_TIME_STEP);
    }

    #[test]
    fn custom_defaults_apply_only_to_their_own_space() {
        let custom = MotionDefaults::new(
            PlannerDefaults::new(2.0, 1.0),
            PlannerDefaults::new(0.5, 0.25),
        );

        let joint = resolve_profile(MotionConstraints::NONE, custom.joint);
        assert_eq!(joint.velocity, 2.0);
        assert_eq!(joint.acceleration, 1.0);

        let cartesian = resolve_profile(MotionConstraints::NONE, custom.cartesian);
        assert_eq!(cartesian.velocity, 0.5);
        assert_eq!(cartesian.acceleration, 0.25);
    }

    #[test]
    fn requested_values_override_the_defaults_verbatim() {
        let constraints = MotionConstraints::new(Some(0.8), Some(0.4));
        let profile = resolve_profile(constraints, MotionDefaults::default().cartesian);
        assert_eq!(profile.velocity, 0.8);
        assert_eq!(profile.acceleration, 0.4);
    }

    #[test]
    fn partial_request_keeps_the_default_for_the_missing_field() {
        let defaults = MotionDefaults::default().cartesian;

        let velocity_only = MotionConstraints::new(Some(0.8), None);
        let profile = resolve_profile(velocity_only, defaults);
        assert_eq!(profile.velocity, 0.8);
        assert_eq!(profile.acceleration, 0.125);

        let acceleration_only = MotionConstraints::new(None, Some(0.4));
        let profile = resolve_profile(acceleration_only, defaults);
        assert_eq!(profile.velocity, 0.25);
        assert_eq!(profile.acceleration, 0.4);
    }

    #[test]
    fn explicit_constraints_take_precedence_over_custom_defaults() {
        let custom = MotionDefaults::new(
            PlannerDefaults::new(10.0, 10.0),
            PlannerDefaults::new(10.0, 10.0),
        );
        let constraints = MotionConstraints::new(Some(0.1), Some(0.2));
        let profile = resolve_profile(constraints, custom.joint);
        assert_eq!(profile.velocity, 0.1);
        assert_eq!(profile.acceleration, 0.2);
    }
}
