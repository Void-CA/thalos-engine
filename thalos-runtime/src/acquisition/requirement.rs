use serde::{Deserialize, Serialize};
use thalos_ports::device::ChannelId;

/// Desired sampling policy for an acquisition stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SamplingRequirement {
    Continuous { target_hz: u32 },
    OnDemand,
}

/// Operational requirement for a channel observation stream.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AcquisitionRequirement {
    pub channel_id: ChannelId,
    pub sampling: SamplingRequirement,
    pub required: bool,
}
