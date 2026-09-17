#[allow(clippy::module_inception)]
pub mod frame;
pub mod registry;

pub use frame::{Frame, FrameId};
pub use registry::FrameRegistry;
