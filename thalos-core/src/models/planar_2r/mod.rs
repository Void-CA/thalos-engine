pub mod factory;
pub mod spec;

pub use spec::{DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_PLANAR_2R, Planar2RSpec};

#[cfg(test)]
pub mod tests;
