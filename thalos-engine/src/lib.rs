//! Thalos Engine facade crate. Internal crates are re-exported here once extracted.

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