use std::fmt;
pub use thalos_core::device::{ChannelId, ChannelObservation, ChannelValue, SignalQuality};
use crate::robot::transport::TransportState;

/// Integration subscription target for a device telemetry signal (L1 Port).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ChannelSubscription {
    pub channel_id: ChannelId,
    pub target_hz: u32,
}

/// Generic errors from device transport acquisition (L1 Port).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DeviceTransportError {
    Disconnected,
    ChannelNotFound(ChannelId),
    Transport(String),
}

impl fmt::Display for DeviceTransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Disconnected => write!(f, "device transport disconnected"),
            Self::ChannelNotFound(ch) => write!(f, "channel not found on device: {}", ch),
            Self::Transport(msg) => write!(f, "device transport error: {}", msg),
        }
    }
}

impl std::error::Error for DeviceTransportError {}

/// Abstract DeviceTransport Domain Port (L1 port)
pub trait DeviceTransport: Send + Sync {
    fn state(&self) -> TransportState;
    fn subscribe(&mut self, subscription: ChannelSubscription) -> Result<(), DeviceTransportError>;
    fn unsubscribe(&mut self, channel_id: &ChannelId) -> Result<(), DeviceTransportError>;
    fn try_receive(&mut self) -> Result<Option<ChannelObservation>, DeviceTransportError>;
}

/// A boxed transport is itself a transport.
///
/// Lets a consumer hold the transport chosen at runtime
/// (`Box<dyn DeviceTransport>`) without naming a concrete mechanism, so a
/// simulated and a physical device are interchangeable behind the same port.
impl<T: DeviceTransport + ?Sized> DeviceTransport for Box<T> {
    fn state(&self) -> TransportState {
        (**self).state()
    }

    fn subscribe(&mut self, subscription: ChannelSubscription) -> Result<(), DeviceTransportError> {
        (**self).subscribe(subscription)
    }

    fn unsubscribe(&mut self, channel_id: &ChannelId) -> Result<(), DeviceTransportError> {
        (**self).unsubscribe(channel_id)
    }

    fn try_receive(&mut self) -> Result<Option<ChannelObservation>, DeviceTransportError> {
        (**self).try_receive()
    }
}
