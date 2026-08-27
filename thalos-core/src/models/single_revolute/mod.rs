pub mod factory;
pub mod spec;

pub use spec::{DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_SINGLE_REVOLUTE, SingleRevoluteSpec};

#[cfg(test)]
pub mod tests;
