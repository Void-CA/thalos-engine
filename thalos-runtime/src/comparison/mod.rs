pub mod alignment;
pub mod comparison;
pub mod metrics;
pub mod pipeline;

pub use alignment::Alignment;
pub use comparison::{PlanExecutionComparison, compare};
pub use metrics::ComparisonMetrics;
pub use pipeline::{ComparePipeline, ComparePipelineError, ComparePipelineOutput};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use thalos_core::device::SignalQuality;
    use thalos_core::deviation::{
        DetectionPolicy, EnvelopeStatus, KinematicDeviationDetector, StaticTolerancePolicy,
    };
    use thalos_core::robot::RobotObservation;
    use thalos_core::trajectory::{Trajectory, TrajectoryPoint};

    fn make_obs(seq: u64, ts_ns: u64, q: Vec<f64>, q_dot: Vec<f64>) -> RobotObservation {
        RobotObservation {
            sampled_at_ns: ts_ns,
            sequence: seq,
            joint_positions_rad: q,
            joint_velocities_rad_s: q_dot,
            tcp_pose: None,
            signal_quality: SignalQuality::Nominal,
        }
    }

    #[test]
    fn test_compare_pipeline_no_trajectory_returns_not_comparable() {
        let policy = StaticTolerancePolicy::uniform(1, 1.0, 1.0);
        let detector = KinematicDeviationDetector::new(DetectionPolicy::new(30_000_000, 20_000_000));
        let mut pipeline: ComparePipeline<Trajectory, StaticTolerancePolicy> =
            ComparePipeline::new(policy, detector);

        let obs = make_obs(1, 500_000_000, vec![0.0], vec![0.0]);
        let res = pipeline.process("r-01", &obs).unwrap();
        assert_eq!(res, ComparePipelineOutput::NotComparable);
    }

    #[test]
    fn test_compare_pipeline_out_of_bounds_returns_not_comparable() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 1.0),
            TrajectoryPoint::new(vec![10.0], 2.0),
        ];
        let traj = Trajectory::new(waypoints);
        let policy = StaticTolerancePolicy::uniform(1, 1.0, 1.0);
        let detector = KinematicDeviationDetector::new(DetectionPolicy::new(30_000_000, 20_000_000));
        let mut pipeline = ComparePipeline::with_trajectory(traj, policy, detector);

        // Sample at t=0.5s (trajectory starts at t=1.0s)
        let obs = make_obs(1, 500_000_000, vec![0.0], vec![0.0]);
        let res = pipeline.process("r-01", &obs).unwrap();
        assert_eq!(res, ComparePipelineOutput::NotComparable);
    }

    #[test]
    fn test_compare_pipeline_end_to_end_sustained_violation_triggers_event() {
        let waypoints = vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 1.0),
        ];
        let traj = Trajectory::new(waypoints);
        // Expecting q = 10.0 * t_sec. At t=0.5s, expected q = 5.0 rad.
        // Tolerance = 0.5 rad.
        let policy = StaticTolerancePolicy::uniform(1, 0.5, 0.5);
        let detector = KinematicDeviationDetector::new(DetectionPolicy::new(30_000_000, 20_000_000));
        let mut pipeline = ComparePipeline::with_trajectory(traj, policy, detector);

        // t=0.0s (Within tolerance)
        let obs0 = make_obs(100, 0, vec![0.0], vec![10.0]);
        assert_eq!(pipeline.process("scara-01", &obs0).unwrap(), ComparePipelineOutput::NoEventEmitted);

        // t=0.500s: Expected=5.0, Observed=6.0 (Error = 1.0 > 0.5 tolerance) -> Violated onset (t=500ms)
        let obs1 = make_obs(101, 500_000_000, vec![6.0], vec![10.0]);
        assert_eq!(pipeline.process("scara-01", &obs1).unwrap(), ComparePipelineOutput::NoEventEmitted);

        // t=0.515s: Violated (15ms elapsed)
        let obs2 = make_obs(102, 515_000_000, vec![6.15], vec![10.0]);
        assert_eq!(pipeline.process("scara-01", &obs2).unwrap(), ComparePipelineOutput::NoEventEmitted);

        // t=0.530s: Violated (30ms elapsed >= confirmation_duration_ns) -> Event Emitted!
        let obs3 = make_obs(103, 530_000_000, vec![6.30], vec![10.0]);
        let output = pipeline.process("scara-01", &obs3).unwrap();

        if let ComparePipelineOutput::EventEmitted(event) = output {
            assert_eq!(event.robot_id, "scara-01");
            assert_eq!(event.observation_sequence, 103);
            assert_eq!(event.observed_at_ns, 530_000_000);
            assert_eq!(event.onset_ns(), 500_000_000);
            assert_eq!(event.deviation.envelope, EnvelopeStatus::Violated);
            assert!((event.deviation.error.joint_position_errors[0] - 1.0).abs() < 1e-9);
        } else {
            panic!("Expected EventEmitted, got {:?}", output);
        }
    }
}
