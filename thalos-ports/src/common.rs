use serde::{Deserialize, Serialize};

/// SignalQuality (ADR-014)
/// Quality indicator for stream observations from hardware, sensor, or simulator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalQuality {
    Nominal,
    Degraded,
    Lost,
}
