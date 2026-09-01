use serde::{Deserialize, Serialize};
use thalos_ports::device::ChannelId;

/// Unique identifier for an active acquisition lease.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LeaseId(pub u64);

/// RAII / handle representing an operational right to an active acquisition stream.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AcquisitionLease {
    pub id: LeaseId,
    pub channel_id: ChannelId,
    pub target_hz: u32,
}
