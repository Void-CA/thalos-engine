use serde::{Deserialize, Serialize};
use thiserror::Error;
use super::kinematic::{EnvelopeStatus, KinematicDeviation};

/// Configuration policy for temporal deviation detection.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DetectionPolicy {
    /// Duration in nanoseconds a violation must persist to trigger confirmation.
    pub confirmation_duration_ns: u64,
    /// Duration in nanoseconds within-tolerance samples must persist to trigger recovery.
    pub recovery_duration_ns: u64,
}

impl DetectionPolicy {
    pub fn new(confirmation_duration_ns: u64, recovery_duration_ns: u64) -> Self {
        Self {
            confirmation_duration_ns,
            recovery_duration_ns,
        }
    }
}

/// Operational status of the temporal deviation detector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DetectorStatus {
    Normal,
    Violating,
}

/// Output emitted when the temporal detector undergoes a status transition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DetectorOutput {
    NoChange,
    ViolationConfirmed { onset_ns: u64, confirmed_at_ns: u64 },
    ViolationRecovered { onset_ns: u64, recovered_at_ns: u64 },
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum DetectorError {
    #[error("Out of order timestamp received: {timestamp_ns} <= last_seen {last_seen_ns}")]
    OutOfOrderTimestamp { timestamp_ns: u64, last_seen_ns: u64 },
}

/// Stateful temporal processor tracking sustained kinematic deviations over time.
#[derive(Debug, Clone)]
pub struct KinematicDeviationDetector {
    policy: DetectionPolicy,
    status: DetectorStatus,
    last_seen_ns: Option<u64>,
    pending_violation_started_at_ns: Option<u64>,
    pending_recovery_started_at_ns: Option<u64>,
}

impl KinematicDeviationDetector {
    pub fn new(policy: DetectionPolicy) -> Self {
        Self {
            policy,
            status: DetectorStatus::Normal,
            last_seen_ns: None,
            pending_violation_started_at_ns: None,
            pending_recovery_started_at_ns: None,
        }
    }

    pub fn status(&self) -> DetectorStatus {
        self.status
    }

    pub fn reset(&mut self) {
        self.status = DetectorStatus::Normal;
        self.last_seen_ns = None;
        self.pending_violation_started_at_ns = None;
        self.pending_recovery_started_at_ns = None;
    }

    pub fn observe(
        &mut self,
        deviation: &KinematicDeviation,
    ) -> Result<DetectorOutput, DetectorError> {
        let ts = deviation.sampled_at_ns;

        if let Some(last) = self.last_seen_ns {
            if ts <= last {
                return Err(DetectorError::OutOfOrderTimestamp {
                    timestamp_ns: ts,
                    last_seen_ns: last,
                });
            }
        }
        self.last_seen_ns = Some(ts);

        match self.status {
            DetectorStatus::Normal => {
                if deviation.envelope == EnvelopeStatus::Violated {
                    self.pending_recovery_started_at_ns = None;
                    let onset = *self.pending_violation_started_at_ns.get_or_insert(ts);

                    if ts.saturating_sub(onset) >= self.policy.confirmation_duration_ns {
                        self.status = DetectorStatus::Violating;
                        self.pending_violation_started_at_ns = None;
                        Ok(DetectorOutput::ViolationConfirmed {
                            onset_ns: onset,
                            confirmed_at_ns: ts,
                        })
                    } else {
                        Ok(DetectorOutput::NoChange)
                    }
                } else {
                    self.pending_violation_started_at_ns = None;
                    Ok(DetectorOutput::NoChange)
                }
            }
            DetectorStatus::Violating => {
                if deviation.envelope == EnvelopeStatus::WithinTolerance {
                    self.pending_violation_started_at_ns = None;
                    let onset = *self.pending_recovery_started_at_ns.get_or_insert(ts);

                    if ts.saturating_sub(onset) >= self.policy.recovery_duration_ns {
                        self.status = DetectorStatus::Normal;
                        self.pending_recovery_started_at_ns = None;
                        Ok(DetectorOutput::ViolationRecovered {
                            onset_ns: onset,
                            recovered_at_ns: ts,
                        })
                    } else {
                        Ok(DetectorOutput::NoChange)
                    }
                } else {
                    self.pending_recovery_started_at_ns = None;
                    Ok(DetectorOutput::NoChange)
                }
            }
        }
    }
}
