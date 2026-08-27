pub mod factory;
pub mod spec;

pub use spec::{
    DEFAULT as DEFAULT_SPEC, JOINTS as JOINTS_SPHERICAL_POLAR_RRP, SphericalPolarRRPSpec,
};

#[cfg(test)]
pub mod tests;
