use serde::{Deserialize, Serialize};

/// Temporal REQUEST of a movement — the program's intent.
///
/// `None` means "the program did not specify this value". It is NOT a robot
/// capability, NOT a safety limit, and NOT a planner default: those are
/// resolved downstream. Keeping the request separate from the capability is
/// what prevents the planner from owning things that belong to the robot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct MotionConstraints {
    pub velocity: Option<f64>,
    pub acceleration: Option<f64>,
}

impl MotionConstraints {
    /// No temporal request — the movement is fully delegated to the planner.
    pub const NONE: Self = Self {
        velocity: None,
        acceleration: None,
    };

    pub const fn new(velocity: Option<f64>, acceleration: Option<f64>) -> Self {
        Self {
            velocity,
            acceleration,
        }
    }

    pub const fn is_unconstrained(&self) -> bool {
        self.velocity.is_none() && self.acceleration.is_none()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_is_unconstrained() {
        assert!(MotionConstraints::NONE.is_unconstrained());
        assert!(MotionConstraints::default().is_unconstrained());
    }

    #[test]
    fn any_requested_value_is_not_unconstrained() {
        assert!(!MotionConstraints::new(Some(1.0), None).is_unconstrained());
        assert!(!MotionConstraints::new(None, Some(0.5)).is_unconstrained());
    }
}
