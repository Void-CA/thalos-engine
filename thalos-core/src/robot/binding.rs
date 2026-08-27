use serde::{Deserialize, Serialize};

use crate::robot::capability::{JointObservationCapability, RobotCapability};
use crate::robot::observation::{ObservationAssessment, ObservationQuality};
use crate::robot::policy::{ObservationResponsePolicy, PolicyDecision};
use crate::robot::state::{JointState, RobotState, StateDeviation};

/// Physical calibration mapping raw driver readings to engineering units (radians / meters).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EncoderCalibration {
    /// Scale factor converting raw counts/ticks to engineering units (rad/tick or m/tick).
    pub scale: f64,
    /// Physical zero-reference offset in engineering units (rad or m).
    pub offset: f64,
    /// Whether direction of rotation is inverted relative to standard kinematic frame.
    pub inverted: bool,
}

impl Default for EncoderCalibration {
    fn default() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
            inverted: false,
        }
    }
}

impl EncoderCalibration {
    pub fn raw_to_physical(&self, raw_ticks: f64) -> f64 {
        let val = (raw_ticks * self.scale) + self.offset;
        if self.inverted {
            -val
        } else {
            val
        }
    }
}

/// Binding sources available for joint state components.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum JointSourceBinding {
    /// Incremental or absolute rotary/linear encoder.
    Encoder {
        device_id: String,
        channel: u8,
        calibration: EncoderCalibration,
    },
    /// Analog voltage input (e.g. potentiometer).
    Analog {
        device_id: String,
        channel: u8,
        v_min: f64,
        v_max: f64,
        q_min: f64,
        q_max: f64,
    },
    /// Virtual / simulated software source.
    Virtual,
}

/// Declarative hardware binding for a single joint's state variables.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointStateBinding {
    pub joint_index: usize,
    pub position_source: Option<JointSourceBinding>,
    pub velocity_source: Option<JointSourceBinding>,
    pub effort_source: Option<JointSourceBinding>,
}

impl JointStateBinding {
    pub fn new(joint_index: usize) -> Self {
        Self {
            joint_index,
            position_source: None,
            velocity_source: None,
            effort_source: None,
        }
    }

    pub fn with_position(mut self, source: JointSourceBinding) -> Self {
        self.position_source = Some(source);
        self
    }

    pub fn with_velocity(mut self, source: JointSourceBinding) -> Self {
        self.velocity_source = Some(source);
        self
    }

    pub fn with_effort(mut self, source: JointSourceBinding) -> Self {
        self.effort_source = Some(source);
        self
    }

    /// Derive static `JointObservationCapability` from the configured hardware bindings.
    pub fn to_capability(&self) -> JointObservationCapability {
        JointObservationCapability {
            position: self.position_source.is_some(),
            velocity: self.velocity_source.is_some(),
            effort: self.effort_source.is_some(),
        }
    }
}

/// Declarative hardware configuration binding for an entire robot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct RobotHardwareBinding {
    pub joint_bindings: Vec<JointStateBinding>,
    pub sensors: Vec<SensorContract>,
}

impl RobotHardwareBinding {
    pub fn new(joint_bindings: Vec<JointStateBinding>, sensors: Vec<SensorContract>) -> Self {
        Self {
            joint_bindings,
            sensors,
        }
    }

    /// Derive global `RobotCapability` automatically from physical hardware configuration bindings.
    pub fn to_capability(&self) -> RobotCapability {
        let joint_caps = self
            .joint_bindings
            .iter()
            .map(|b| b.to_capability())
            .collect();
        RobotCapability::new(joint_caps)
    }
}

/// Classification of independent physical sensors (decoupled from joint state variables).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SensorKind {
    IMU,
    ForceTorque,
    Temperature,
    Vibration,
    Pressure,
    Camera,
}

/// Formal contract describing an independent physical sensor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorContract {
    pub sensor_id: String,
    pub kind: SensorKind,
    pub update_rate_hz: f64,
}

/// A raw or processed measurement sample originating from a hardware state source or sensor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObservationSample {
    pub source_id: String,
    pub timestamp: f64,
    pub raw_value: f64,
    pub physical_value: f64,
    pub quality: ObservationQuality,
}

/// Abstraction for hardware drivers reading physical sensors or state sources.
pub trait StateSource {
    fn read_sample(&mut self) -> Option<ObservationSample>;
}

/// State aggregator that compiles raw physical samples into a unified domain `RobotState`.
#[derive(Debug, Clone)]
pub struct StateAggregator {
    pub binding: RobotHardwareBinding,
}

impl StateAggregator {
    pub fn new(binding: RobotHardwareBinding) -> Self {
        Self { binding }
    }

    /// Build a `RobotState` snapshot from position readings for each joint.
    pub fn aggregate_positions(&self, timestamp: f64, raw_positions: &[Option<f64>]) -> RobotState {
        let mut joints = Vec::with_capacity(self.binding.joint_bindings.len());

        for (i, j_binding) in self.binding.joint_bindings.iter().enumerate() {
            let raw_opt = raw_positions.get(i).copied().flatten();
            let pos_val = match (&j_binding.position_source, raw_opt) {
                (Some(JointSourceBinding::Encoder { calibration, .. }), Some(raw)) => {
                    Some(calibration.raw_to_physical(raw))
                }
                (Some(JointSourceBinding::Virtual), Some(raw)) => Some(raw),
                _ => None,
            };

            joints.push(JointState {
                position: pos_val,
                velocity: None,
                effort: None,
            });
        }

        RobotState::new(timestamp, joints)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use crate::robot::capability::{JointObservationRequirement, ObservationConstraint, ObservationRequirement};

    struct MockEncoderHardware {
        ticks: f64,
    }

    impl StateSource for MockEncoderHardware {
        fn read_sample(&mut self) -> Option<ObservationSample> {
            Some(ObservationSample {
                source_id: "encoder_j0".into(),
                timestamp: 0.1,
                raw_value: self.ticks,
                physical_value: self.ticks * 0.001, // e.g. 1000 ticks/rad
                quality: ObservationQuality::Valid,
            })
        }
    }

    #[test]
    fn end_to_end_hardware_binding_to_policy_decision() {
        // 1. Configure Hardware Binding (Encoder on J0 with scale & offset)
        let j0_binding = JointStateBinding::new(0).with_position(JointSourceBinding::Encoder {
            device_id: "enc_0".into(),
            channel: 0,
            calibration: EncoderCalibration {
                scale: 0.001, // 1000 ticks = 1 rad
                offset: 0.1,  // +0.1 rad mounting offset
                inverted: false,
            },
        });
        let hw_binding = RobotHardwareBinding::new(vec![j0_binding], vec![]);

        // 2. Automatically derive RobotCapability from Hardware Binding
        let cap = hw_binding.to_capability();
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(10))),
                velocity: None,
                effort: None,
            },
        );
        assert!(cap.matches(&req).is_satisfied());

        // 3. Simulate Hardware Reading & State Aggregation (1000 ticks -> 1.0 rad + 0.1 offset = 1.1 rad)
        let aggregator = StateAggregator::new(hw_binding);
        let observed_state = aggregator.aggregate_positions(0.1, &[Some(1000.0)]);
        assert_eq!(observed_state.joints[0].position, Some(1.1));

        // 4. Expected State is 1.0 rad -> Deviation is 0.1 rad
        let expected_state = RobotState::from_positions(vec![1.0]);
        let dev = StateDeviation::compute(&expected_state, &observed_state);

        // 5. Evaluate Observation Quality Assessment (fresh sample age 2ms)
        let assessment = ObservationAssessment::evaluate(&req, &observed_state, Duration::from_millis(2));
        assert!(assessment.is_valid());

        // 6. Policy decision under strict tolerance (max 0.05 rad) -> Exceeds deviation -> PAUSE!
        let strict_policy = ObservationResponsePolicy::new(
            crate::robot::policy::DeviationThresholds::position(0.05),
        );
        let decision = strict_policy.evaluate(&assessment, dev.as_ref());
        assert_eq!(decision, PolicyDecision::Pause);

        // 7. Policy decision under lenient tolerance (max 0.20 rad) -> Within deviation -> CONTINUE!
        let lenient_policy = ObservationResponsePolicy::new(
            crate::robot::policy::DeviationThresholds::position(0.20),
        );
        let decision = lenient_policy.evaluate(&assessment, dev.as_ref());
        assert_eq!(decision, PolicyDecision::Continue);
    }
}
