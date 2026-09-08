use serde::{Deserialize, Serialize};
use thalos_ports::device::ChannelId;

/// Desired sampling policy for an observation stream.
///
/// `target_hz` expresses the desired observation rate. Actual delivery
/// frequency is constrained by the Interconnection provider, transport,
/// protocol, and source capabilities.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SamplingPolicy {
    Continuous { target_hz: u32 },
    OnDemand,
}

/// Operational requirement for an observation stream.
///
/// Describes what a consumer needs from the interconnection layer:
/// which channel to observe, with what sampling policy, and whether
/// it is mandatory.
///
/// Note: `channel_id` references the observable endpoint known to the
/// consumer. Migration from `ChannelId` to `SignalId` is deferred until
/// Signal identity becomes operational (ADR-016).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationRequirement {
    pub channel_id: ChannelId,
    pub sampling: SamplingPolicy,
    pub mandatory: bool,
}
