pub mod observation;
pub mod signal;

pub use observation::{ChannelId, ChannelObservation, ChannelValue, SignalQuality};
pub use signal::{
    ConnectionState, DerivedSignal, Endpoint, EndpointKind, Signal, SignalBinding,
    SignalDirection, SignalExpression, SignalKind,
};
