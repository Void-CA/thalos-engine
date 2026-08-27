pub mod factory;
pub mod spec;

pub use spec::{DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_SCARA, ScaraSpec};

#[cfg(test)]
pub mod tests;
