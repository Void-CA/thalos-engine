pub mod motion_resolver;
pub mod types;

pub use motion_resolver::{replan_suffix, MotionResolver, PlannedSuffix};
pub use types::{MotionResolution, ResolutionError};
