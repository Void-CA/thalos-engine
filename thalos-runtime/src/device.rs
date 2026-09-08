//! Type facade for thalos-core device types.
//!
//! This module re-exports domain types from thalos-core so that application
//! crates (thalos-tauri) can depend on thalos-runtime as the single facade,
//! without importing thalos-core directly.
//!
//! # Facade Rules
//!
//! - `thalos-tauri` depends on `thalos-runtime` for types and services.
//! - `thalos-tauri` must NOT become a second domain access layer over `thalos-core`.
//! - This module is a **type facade** (re-exports), not an operational facade.
//! - Operational logic lives in services/runtime, not in this module.

pub use thalos_core::device::{
    ChannelId, ChannelObservation, ChannelValue, ConnectionState, DerivedSignal, Endpoint,
    EndpointKind, Signal, SignalBinding, SignalDirection, SignalExpression, SignalKind,
    SignalQuality,
};
