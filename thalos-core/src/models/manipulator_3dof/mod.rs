pub mod factory;
pub mod spec;

pub use spec::{DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_MANIPULATOR_3DOF, Manipulator3DOFSpec};

#[cfg(test)]
pub mod tests;
