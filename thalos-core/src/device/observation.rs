use serde::{Deserialize, Serialize};

/// Discrete signal quality for telemetry & observation events (L0 Domain).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalQuality {
    Nominal,
    Degraded,
    Lost,
}

pub type ChannelId = String;

/// Strongly typed channel telemetry value (L0 Domain).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChannelValue {
    Scalar(f64),
    Boolean(bool),
    Integer(i64),
}

/// Telemetry observation event from an IIoT channel or sensor (L0 Domain).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChannelObservation {
    pub channel_id: ChannelId,
    pub sampled_at_ns: u64,
    pub received_at_ns: u64,
    pub value: ChannelValue,
    pub unit: Option<String>,
    pub quality: SignalQuality,
}
