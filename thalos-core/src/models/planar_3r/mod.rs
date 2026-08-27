pub mod factory;
pub mod spec;

pub use spec::{DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_PLANAR_3R, Planar3RSpec};

#[cfg(test)]
pub mod tests;
