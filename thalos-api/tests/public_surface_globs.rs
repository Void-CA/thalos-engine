//! Ratchet guard for glob re-exports in the public boundary.
//!
//! `thalos-api` must expose named items, not whole crates/modules. The current
//! globs are recorded as debt: no new glob may appear, and removing one
//! requires deleting its baseline entry in the same change.

use std::fs;
use std::path::Path;

const BASELINE: &[&str] = &[
    "pub use thalos_core::analysis::*;",
    "pub use thalos_models::*;",
    "pub use thalos_core::models::*;",
    "pub use thalos_math::*;",
    "pub use thalos_planning::*;",
    "pub use thalos_importer::*;",
    "pub use thalos_ports::*;",
    "pub use thalos_document::*;",
    "pub use thalos_visual::*;",
    "pub use thalos_telemetry::*;",
];

#[test]
fn public_boundary_does_not_add_glob_reexports() {
    let lib = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/lib.rs");
    let content =
        fs::read_to_string(&lib).unwrap_or_else(|e| panic!("read {}: {e}", lib.display()));
    let actual: Vec<&str> = content
        .lines()
        .map(str::trim)
        .filter(|line| line.starts_with("pub use thalos_") && line.ends_with("::*;"))
        .collect();

    let mut failures = Vec::new();
    for line in &actual {
        if !BASELINE.contains(line) {
            failures.push(format!("new glob re-export in thalos-api: {line}"));
        }
    }
    for line in BASELINE {
        if !actual.contains(line) {
            failures.push(format!(
                "stale baseline glob (now removed); delete its entry: {line}"
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "Public-surface glob ratchet failed:\n{}",
        failures.join("\n")
    );
}
