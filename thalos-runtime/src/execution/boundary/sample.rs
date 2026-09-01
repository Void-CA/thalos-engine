//! Execution sample type — a single point in a collected execution trace.
//!
//! These samples are collected by the hardware backend during execution.

/// A single execution sample collected from hardware during a trajectory run.
///
/// Each sample records the robot's joint positions at an absolute microsecond
/// timestamp (measured from execution start on the hardware side).
#[derive(Debug, Clone, PartialEq)]
pub struct ExecutionSample {
    /// Microseconds since execution start (hardware-local clock).
    pub timestamp_us: u64,
    /// Joint positions in radians at this instant.
    pub joints: Vec<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn execution_sample_roundtrip() {
        let sample = ExecutionSample {
            timestamp_us: 1_234_567,
            joints: vec![0.5, -0.3, 0.0, 1.0, 0.2, -0.1],
        };

        assert_eq!(sample.timestamp_us, 1_234_567);
        assert_eq!(sample.joints.len(), 6);
        assert!((sample.joints[0] - 0.5).abs() < 1e-12);
        assert!((sample.joints[3] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn execution_sample_zero_timestamp() {
        let sample = ExecutionSample {
            timestamp_us: 0,
            joints: vec![],
        };

        assert_eq!(sample.timestamp_us, 0);
        assert!(sample.joints.is_empty());
    }
}
