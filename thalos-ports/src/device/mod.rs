#[cfg(feature = "test-support")]
pub mod fake;

pub mod transport;

pub use thalos_core::device::{ChannelId, ChannelObservation, ChannelValue, SignalQuality};
pub use transport::{ChannelSubscription, DeviceTransport, DeviceTransportError};

#[cfg(feature = "test-support")]
pub use fake::FakeDeviceTransport;

