pub mod factory;
pub mod spec;

pub use spec::{CylindricalRPPSpec, DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_CYLINDRICAL_RPP};

#[cfg(test)]
pub mod tests;
