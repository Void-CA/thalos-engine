//! Thalos Engine — unified robotics domain facade crate.

pub use thalos_collision as collision;
pub use thalos_core as core;
pub use thalos_intelligence as intelligence;
pub use thalos_math as math;
pub use thalos_models as models;
pub use thalos_optimization as optimization;
pub use thalos_planning as planning;
pub use thalos_semantic as semantic;

/// Convenient re-exports of common types and traits across Thalos Engine.
pub mod prelude {
    pub use thalos_core::prelude::*;
    pub use thalos_math::*;
}

/// Returns the Engine version, inherited from the workspace package.
pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}


#[cfg(test)]
mod tests {
    #[test]
    fn it_compiles() {
        assert!(!crate::version().is_empty());
    }

    #[test]
    fn reports_workspace_version() {
        assert_eq!(crate::version(), "0.1.0");
    }
}