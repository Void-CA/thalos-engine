use thalos_core::deviation::{
    DeviationAnalysisError, DeviationAnalyzer, DeviationEvent, DeviationEventId,
    DeviationEventKind, ExpectedTrajectory, KinematicDeviationDetector, ObservedState,
    TolerancePolicy,
};
use thalos_core::robot::RobotObservation;
use thiserror::Error;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ComparePipelineError {
    #[error("Analysis error: {0}")]
    AnalysisError(String),

    #[error("Detector error: {0}")]
    DetectorError(String),
}

#[derive(Debug, PartialEq)]
pub enum ComparePipelineOutput {
    NotComparable,
    NoEventEmitted,
    EventEmitted(DeviationEvent),
}

/// Orchestrates real-time comparison between telemetry RobotObservations and an ExpectedTrajectory.
pub struct ComparePipeline<T: ExpectedTrajectory, P: TolerancePolicy> {
    trajectory: Option<T>,
    tolerance_policy: P,
    detector: KinematicDeviationDetector,
}

impl<T: ExpectedTrajectory, P: TolerancePolicy> ComparePipeline<T, P> {
    pub fn new(tolerance_policy: P, detector: KinematicDeviationDetector) -> Self {
        Self {
            trajectory: None,
            tolerance_policy,
            detector,
        }
    }

    pub fn with_trajectory(
        trajectory: T,
        tolerance_policy: P,
        detector: KinematicDeviationDetector,
    ) -> Self {
        Self {
            trajectory: Some(trajectory),
            tolerance_policy,
            detector,
        }
    }

    pub fn set_trajectory(&mut self, trajectory: Option<T>) {
        self.trajectory = trajectory;
        self.detector.reset();
    }

    pub fn process(
        &mut self,
        robot_id: &str,
        observation: &RobotObservation,
    ) -> Result<ComparePipelineOutput, ComparePipelineError> {
        let trajectory = match &self.trajectory {
            Some(t) => t,
            None => return Ok(ComparePipelineOutput::NotComparable),
        };

        let observed_state = ObservedState::new(
            robot_id,
            observation.sampled_at_ns,
            observation.joint_positions_rad.clone(),
            observation.joint_velocities_rad_s.clone(),
            None,
        );

        let deviation = match DeviationAnalyzer::analyze(trajectory, &observed_state, &self.tolerance_policy) {
            Ok(dev) => dev,
            Err(DeviationAnalysisError::OutOfBoundsTimestamp { .. }) => {
                return Ok(ComparePipelineOutput::NotComparable);
            }
            Err(err) => return Err(ComparePipelineError::AnalysisError(err.to_string())),
        };

        let output = self
            .detector
            .observe(&deviation)
            .map_err(|e| ComparePipelineError::DetectorError(e.to_string()))?;

        match output {
            thalos_core::deviation::DetectorOutput::NoChange => Ok(ComparePipelineOutput::NoEventEmitted),
            thalos_core::deviation::DetectorOutput::ViolationConfirmed { onset_ns, confirmed_at_ns: _ } => {
                let event_id = DeviationEventId::from_sequence(robot_id, observation.sequence);
                let event = DeviationEvent::new(
                    event_id,
                    observation.sequence,
                    DeviationEventKind::ViolationConfirmed { onset_ns },
                    deviation,
                );
                Ok(ComparePipelineOutput::EventEmitted(event))
            }
            thalos_core::deviation::DetectorOutput::ViolationRecovered { onset_ns, recovered_at_ns: _ } => {
                let event_id = DeviationEventId::from_sequence(robot_id, observation.sequence);
                let event = DeviationEvent::new(
                    event_id,
                    observation.sequence,
                    DeviationEventKind::ViolationRecovered { onset_ns },
                    deviation,
                );
                Ok(ComparePipelineOutput::EventEmitted(event))
            }
        }
    }
}
