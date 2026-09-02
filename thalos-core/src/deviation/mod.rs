pub mod analyzer;
pub mod detector;
pub mod event;
pub mod expected;
pub mod kinematic;
pub mod policy;

pub use analyzer::{DeviationAnalysisError, DeviationAnalyzer};
pub use detector::{DetectionPolicy, DetectorError, DetectorOutput, DetectorStatus, KinematicDeviationDetector};
pub use event::{DeviationEvent, DeviationEventId, DeviationEventKind};
pub use expected::{ExpectedState, ExpectedTrajectory, ObservedState};
pub use kinematic::{DeviationSeverity, EnvelopeStatus, KinematicDeviation, KinematicError};
pub use policy::{JointTolerance, StaticTolerancePolicy, TolerancePolicy};

use crate::trajectory::Trajectory;

impl ExpectedTrajectory for Trajectory {
    fn sample_at(&self, timestamp_ns: u64) -> Option<ExpectedState> {
        let waypoints = self.waypoints();
        if waypoints.is_empty() {
            return None;
        }

        let t_sec = (timestamp_ns as f64) / 1_000_000_000.0;

        let start_time = waypoints.first().unwrap().timestamp();
        let end_time = waypoints.last().unwrap().timestamp();

        if t_sec < start_time || t_sec > end_time {
            return None;
        }

        if (t_sec - start_time).abs() < 1e-12 {
            let first = waypoints.first().unwrap();
            let v = if waypoints.len() > 1 {
                let dt = waypoints[1].timestamp() - start_time;
                if dt > 1e-12 {
                    first
                        .joints()
                        .iter()
                        .zip(waypoints[1].joints().iter())
                        .map(|(q0, q1)| (q1 - q0) / dt)
                        .collect()
                } else {
                    vec![0.0; first.joints().len()]
                }
            } else {
                vec![0.0; first.joints().len()]
            };
            return Some(ExpectedState::new(
                timestamp_ns,
                first.joints().to_vec(),
                v,
                None,
            ));
        }

        if (t_sec - end_time).abs() < 1e-12 {
            let last = waypoints.last().unwrap();
            let len = waypoints.len();
            let v = if len > 1 {
                let prev = &waypoints[len - 2];
                let dt = end_time - prev.timestamp();
                if dt > 1e-12 {
                    prev.joints()
                        .iter()
                        .zip(last.joints().iter())
                        .map(|(q0, q1)| (q1 - q0) / dt)
                        .collect()
                } else {
                    vec![0.0; last.joints().len()]
                }
            } else {
                vec![0.0; last.joints().len()]
            };
            return Some(ExpectedState::new(
                timestamp_ns,
                last.joints().to_vec(),
                v,
                None,
            ));
        }

        for window in waypoints.windows(2) {
            let w0 = &window[0];
            let w1 = &window[1];
            let t0 = w0.timestamp();
            let t1 = w1.timestamp();

            if t_sec >= t0 && t_sec <= t1 {
                let dt = t1 - t0;
                if dt < 1e-12 {
                    return Some(ExpectedState::new(
                        timestamp_ns,
                        w0.joints().to_vec(),
                        vec![0.0; w0.joints().len()],
                        None,
                    ));
                }

                let alpha = (t_sec - t0) / dt;

                let joint_positions: Vec<f64> = w0
                    .joints()
                    .iter()
                    .zip(w1.joints().iter())
                    .map(|(q0, q1)| q0 + alpha * (q1 - q0))
                    .collect();

                let joint_velocities: Vec<f64> = w0
                    .joints()
                    .iter()
                    .zip(w1.joints().iter())
                    .map(|(q0, q1)| (q1 - q0) / dt)
                    .collect();

                return Some(ExpectedState::new(
                    timestamp_ns,
                    joint_positions,
                    joint_velocities,
                    None,
                ));
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trajectory::{Trajectory, TrajectoryPoint};

    #[test]
    fn test_exact_linear_interpolation_middle() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);

        let sampled = traj.sample_at(500_000_000).expect("Should sample at 0.5s");
        assert_eq!(sampled.timestamp_ns, 500_000_000);
        assert!((sampled.joint_positions[0] - 5.0).abs() < 1e-9);
        assert!((sampled.joint_velocities[0] - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_multi_joint_interpolation() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0, 10.0, 20.0], 0.0),
            TrajectoryPoint::new(vec![10.0, 20.0, 40.0], 2.0),
        ];
        let traj = Trajectory::new(waypoints);

        let sampled = traj.sample_at(1_000_000_000).expect("Should sample at 1.0s");
        assert_eq!(sampled.joint_positions, vec![5.0, 15.0, 30.0]);
        assert_eq!(sampled.joint_velocities, vec![5.0, 5.0, 10.0]);
    }

    #[test]
    fn test_exact_waypoint_matching() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![1.5, 2.5], 0.0),
            TrajectoryPoint::new(vec![3.5, 4.5], 1.0),
        ];
        let traj = Trajectory::new(waypoints);

        let start_sample = traj.sample_at(0).expect("Should match t0");
        assert_eq!(start_sample.joint_positions, vec![1.5, 2.5]);

        let end_sample = traj.sample_at(1_000_000_000).expect("Should match t1");
        assert_eq!(end_sample.joint_positions, vec![3.5, 4.5]);
    }

    #[test]
    fn test_out_of_bounds_returns_none() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 1.0),
            TrajectoryPoint::new(vec![10.0], 5.0),
        ];
        let traj = Trajectory::new(waypoints);

        assert!(traj.sample_at(500_000_000).is_none());
        assert!(traj.sample_at(6_000_000_000).is_none());
    }

    #[test]
    fn test_irregular_timestamps() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![7.0], 7.0),
            TrajectoryPoint::new(vec![19.0], 19.0),
            TrajectoryPoint::new(vec![41.0], 41.0),
        ];
        let traj = Trajectory::new(waypoints);

        let sampled = traj.sample_at(10_000_000_000).expect("Should sample at 10s");
        assert!((sampled.joint_positions[0] - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_analyzer_dimension_mismatch_returns_error() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0, 1.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("r-01", 500_000_000, vec![0.5, 0.5, 0.5, 0.5], vec![0.0; 4], None);
        let policy = StaticTolerancePolicy::uniform(4, 1.0, 1.0);

        let res = DeviationAnalyzer::analyze(&traj, &obs, &policy);
        assert_eq!(
            res,
            Err(DeviationAnalysisError::DimensionMismatch {
                expected_dof: 3,
                observed_dof: 4
            })
        );
    }

    #[test]
    fn test_analyzer_out_of_bounds_returns_error() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 1.0),
            TrajectoryPoint::new(vec![10.0], 2.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("r-01", 500_000_000, vec![0.0], vec![0.0], None);
        let policy = StaticTolerancePolicy::uniform(1, 1.0, 1.0);

        let res = DeviationAnalyzer::analyze(&traj, &obs, &policy);
        assert_eq!(
            res,
            Err(DeviationAnalysisError::OutOfBoundsTimestamp {
                timestamp_ns: 500_000_000
            })
        );
    }

    #[test]
    fn test_analyzer_perfect_match() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0, 10.0], 0.0),
            TrajectoryPoint::new(vec![10.0, 20.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("r-01", 500_000_000, vec![5.0, 15.0], vec![10.0, 10.0], None);
        let policy = StaticTolerancePolicy::uniform(2, 0.1, 0.1);

        let deviation = DeviationAnalyzer::analyze(&traj, &obs, &policy).unwrap();
        assert_eq!(deviation.envelope, EnvelopeStatus::WithinTolerance);
        assert_eq!(deviation.error.joint_position_errors, vec![0.0, 0.0]);
        assert_eq!(deviation.error.joint_velocity_errors, vec![0.0, 0.0]);
        assert_eq!(deviation.error.joint_position_error_norm(), 0.0);
    }

    #[test]
    fn test_analyzer_within_envelope() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("r-01", 500_000_000, vec![5.5], vec![10.0], None);
        let policy = StaticTolerancePolicy::uniform(1, 1.0, 1.0);

        let deviation = DeviationAnalyzer::analyze(&traj, &obs, &policy).unwrap();
        assert_eq!(deviation.envelope, EnvelopeStatus::WithinTolerance);
        assert!((deviation.error.joint_position_errors[0] - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_analyzer_exact_tolerance_boundary_is_within_tolerance() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("r-01", 500_000_000, vec![6.0], vec![10.0], None);
        let policy = StaticTolerancePolicy::uniform(1, 1.0, 1.0);

        let deviation = DeviationAnalyzer::analyze(&traj, &obs, &policy).unwrap();
        assert_eq!(deviation.envelope, EnvelopeStatus::WithinTolerance);
    }

    #[test]
    fn test_analyzer_exceeding_tolerance_violates_envelope() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("r-01", 500_000_000, vec![6.01], vec![10.0], None);
        let policy = StaticTolerancePolicy::uniform(1, 1.0, 1.0);

        let deviation = DeviationAnalyzer::analyze(&traj, &obs, &policy).unwrap();
        assert_eq!(deviation.envelope, EnvelopeStatus::Violated);
        assert!((deviation.error.joint_position_errors[0] - 1.01).abs() < 1e-9);
    }

    #[test]
    fn test_deviation_event_construction_and_traceability() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("robot-scara-01", 500_000_000, vec![5.2], vec![10.0], None);
        let policy = StaticTolerancePolicy::uniform(1, 0.5, 0.5);

        let deviation = DeviationAnalyzer::analyze(&traj, &obs, &policy).unwrap();
        let event_id = DeviationEventId::from_sequence("robot-scara-01", 1042);

        let event = DeviationEvent::new(
            event_id.clone(),
            1042,
            DeviationEventKind::ViolationConfirmed { onset_ns: 480_000_000 },
            deviation.clone(),
        );

        assert_eq!(event.event_id, event_id);
        assert_eq!(event.robot_id, "robot-scara-01");
        assert_eq!(event.observed_at_ns, 500_000_000);
        assert_eq!(event.onset_ns(), 480_000_000);
        assert_eq!(event.observation_sequence, 1042);
        assert_eq!(event.deviation, deviation);
    }

    #[test]
    fn test_deviation_event_serde_roundtrip() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        let obs = ObservedState::new("robot-scara-01", 500_000_000, vec![6.5], vec![10.0], None);
        let policy = StaticTolerancePolicy::uniform(1, 1.0, 1.0);

        let deviation = DeviationAnalyzer::analyze(&traj, &obs, &policy).unwrap();
        let event = DeviationEvent::new(
            DeviationEventId::from_sequence("robot-scara-01", 2048),
            2048,
            DeviationEventKind::ViolationConfirmed { onset_ns: 470_000_000 },
            deviation,
        );

        let json = serde_json::to_string(&event).expect("Serialization failed");
        assert!(json.contains("dev_robot-scara-01_2048"));
        assert!(json.contains("ViolationConfirmed"));

        let deserialized: DeviationEvent = serde_json::from_str(&json).expect("Deserialization failed");
        assert_eq!(event, deserialized);
    }

    fn mock_deviation(ts_ns: u64, envelope: EnvelopeStatus) -> KinematicDeviation {
        KinematicDeviation {
            robot_id: "r-01".to_string(),
            sampled_at_ns: ts_ns,
            expected: ExpectedState::new(ts_ns, vec![0.0], vec![0.0], None),
            observed: ObservedState::new("r-01", ts_ns, vec![0.0], vec![0.0], None),
            error: KinematicError {
                joint_position_errors: vec![0.0],
                joint_velocity_errors: vec![0.0],
                cartesian_position_error: None,
            },
            envelope,
            severity: None,
        }
    }

    #[test]
    fn test_detector_isolated_noise_does_not_trigger_confirmation() {
        let policy = DetectionPolicy::new(30_000_000, 20_000_000);
        let mut detector = KinematicDeviationDetector::new(policy);

        assert_eq!(
            detector.observe(&mock_deviation(1_000_000_000, EnvelopeStatus::WithinTolerance)).unwrap(),
            DetectorOutput::NoChange
        );
        assert_eq!(
            detector.observe(&mock_deviation(1_010_000_000, EnvelopeStatus::Violated)).unwrap(),
            DetectorOutput::NoChange
        );
        assert_eq!(
            detector.observe(&mock_deviation(1_020_000_000, EnvelopeStatus::WithinTolerance)).unwrap(),
            DetectorOutput::NoChange
        );

        assert_eq!(detector.status(), DetectorStatus::Normal);
    }

    #[test]
    fn test_detector_sustained_violation_triggers_confirmation() {
        let policy = DetectionPolicy::new(30_000_000, 20_000_000);
        let mut detector = KinematicDeviationDetector::new(policy);

        detector.observe(&mock_deviation(1_000_000_000, EnvelopeStatus::WithinTolerance)).unwrap();

        assert_eq!(
            detector.observe(&mock_deviation(1_010_000_000, EnvelopeStatus::Violated)).unwrap(),
            DetectorOutput::NoChange
        );
        assert_eq!(
            detector.observe(&mock_deviation(1_025_000_000, EnvelopeStatus::Violated)).unwrap(),
            DetectorOutput::NoChange
        );
        assert_eq!(
            detector.observe(&mock_deviation(1_040_000_000, EnvelopeStatus::Violated)).unwrap(),
            DetectorOutput::ViolationConfirmed {
                onset_ns: 1_010_000_000,
                confirmed_at_ns: 1_040_000_000,
            }
        );

        assert_eq!(detector.status(), DetectorStatus::Violating);
    }

    #[test]
    fn test_detector_sustained_recovery_triggers_recovery_event() {
        let policy = DetectionPolicy::new(30_000_000, 20_000_000);
        let mut detector = KinematicDeviationDetector::new(policy);

        detector.observe(&mock_deviation(1_000_000_000, EnvelopeStatus::Violated)).unwrap();
        detector.observe(&mock_deviation(1_030_000_000, EnvelopeStatus::Violated)).unwrap();
        assert_eq!(detector.status(), DetectorStatus::Violating);

        assert_eq!(
            detector.observe(&mock_deviation(1_040_000_000, EnvelopeStatus::WithinTolerance)).unwrap(),
            DetectorOutput::NoChange
        );
        assert_eq!(
            detector.observe(&mock_deviation(1_050_000_000, EnvelopeStatus::WithinTolerance)).unwrap(),
            DetectorOutput::NoChange
        );
        assert_eq!(
            detector.observe(&mock_deviation(1_060_000_000, EnvelopeStatus::WithinTolerance)).unwrap(),
            DetectorOutput::ViolationRecovered {
                onset_ns: 1_040_000_000,
                recovered_at_ns: 1_060_000_000,
            }
        );

        assert_eq!(detector.status(), DetectorStatus::Normal);
    }

    #[test]
    fn test_detector_flicker_resets_pending_timers() {
        let policy = DetectionPolicy::new(30_000_000, 20_000_000);
        let mut detector = KinematicDeviationDetector::new(policy);

        detector.observe(&mock_deviation(1_000_000_000, EnvelopeStatus::Violated)).unwrap();
        detector.observe(&mock_deviation(1_010_000_000, EnvelopeStatus::WithinTolerance)).unwrap();

        detector.observe(&mock_deviation(1_020_000_000, EnvelopeStatus::Violated)).unwrap();
        assert_eq!(
            detector.observe(&mock_deviation(1_040_000_000, EnvelopeStatus::Violated)).unwrap(),
            DetectorOutput::NoChange
        );
        assert_eq!(
            detector.observe(&mock_deviation(1_050_000_000, EnvelopeStatus::Violated)).unwrap(),
            DetectorOutput::ViolationConfirmed {
                onset_ns: 1_020_000_000,
                confirmed_at_ns: 1_050_000_000,
            }
        );
    }

    #[test]
    fn test_detector_irregular_timestamps_calculate_exact_time_deltas() {
        let policy = DetectionPolicy::new(25_000_000, 20_000_000);
        let mut detector = KinematicDeviationDetector::new(policy);

        detector.observe(&mock_deviation(1_000_000_000, EnvelopeStatus::Violated)).unwrap();
        detector.observe(&mock_deviation(1_007_000_000, EnvelopeStatus::Violated)).unwrap();
        detector.observe(&mock_deviation(1_019_000_000, EnvelopeStatus::Violated)).unwrap();
        assert_eq!(
            detector.observe(&mock_deviation(1_026_000_000, EnvelopeStatus::Violated)).unwrap(),
            DetectorOutput::ViolationConfirmed {
                onset_ns: 1_000_000_000,
                confirmed_at_ns: 1_026_000_000,
            }
        );
    }

    #[test]
    fn test_detector_out_of_order_timestamp_returns_error() {
        let policy = DetectionPolicy::new(30_000_000, 20_000_000);
        let mut detector = KinematicDeviationDetector::new(policy);

        detector.observe(&mock_deviation(1_010_000_000, EnvelopeStatus::WithinTolerance)).unwrap();
        let res = detector.observe(&mock_deviation(1_005_000_000, EnvelopeStatus::WithinTolerance));

        assert_eq!(
            res,
            Err(DetectorError::OutOfOrderTimestamp {
                timestamp_ns: 1_005_000_000,
                last_seen_ns: 1_010_000_000,
            })
        );
    }
}
