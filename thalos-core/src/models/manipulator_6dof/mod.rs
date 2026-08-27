pub mod spec;

pub use spec::{DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_MANIPULATOR_6DOF, Manipulator6DOFSpec};

#[cfg(test)]
pub mod tests;
