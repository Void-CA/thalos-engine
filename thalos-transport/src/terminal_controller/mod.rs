//! Terminal Controller encoding — Fase 4.0 external provider conformance.
//!
//! A line-oriented, TCP-only encoding of the provider contract described in
//! `thalos-industrial/docs/plans/13.0-external-provider-conformance.md`.
//!
//! This is an **encoding**, not a protocol authority: `thalos-protocol/`
//! (protobuf, experimental) remains untouched. The engine never depends on this
//! module; the runtime provider owns the adapter.

pub mod codec;
pub mod transport;
