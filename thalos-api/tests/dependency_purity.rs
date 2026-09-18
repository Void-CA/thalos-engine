//! Dependency purity guard (ADR-020, Fase 1).
//!
//! `thalos-engine` is not a uniformly pure workspace: some crates legitimately
//! touch infrastructure today (`thalos-transport` serial/TCP IO, `thalos-visual`
//! mesh files, `thalos-language-service` profile files, `thalos-importer` asset
//! resolution). A blanket "the engine has no I/O" assertion would therefore be
//! false and useless.
//!
//! This guard instead enforces a **conscious classification**:
//!
//! 1. Every workspace member MUST be declared either as a *pure-domain* crate or
//!    as an explicitly excepted crate (with a reason). A newly added crate that
//!    is not classified fails the test — there is no silent default.
//! 2. A *pure-domain* crate MUST NOT declare a dependency on any infrastructure
//!    / I/O crate.
//!
//! The allowlist is intentional (a positive classification), not a global
//! blacklist: adding `reqwest`, `tokio`, or `rusqlite` to `thalos-core` fails,
//! and any exception has to be declared here on purpose.
//!
//! Concept owned by ADR-020: the engine owns domain + supervision **semantics**;
//! infrastructure and mechanism stay in the application.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

/// Crates that MUST remain free of infrastructure / I/O dependencies.
const PURE_DOMAIN_CRATES: &[&str] = &[
    "thalos-core",
    "thalos-analysis",
    "thalos-math",
    "thalos-models",
    "thalos-planning",
    "thalos-lang",
    "thalos-ports",
    "thalos-document",
];

/// Crates explicitly excepted from the purity guard, with the reason.
///
/// Review planned in Fase 3 (post-extraction cleanup).
const EXCEPTED_CRATES: &[(&str, &str)] = &[
    (
        "thalos-api",
        "public boundary facade: re-exports engine crates, not classified as pure domain",
    ),
    (
        "thalos-transport",
        "concrete serial/TCP/ESP32 IO (documented exception, ADR-018)",
    ),
    (
        "thalos-visual",
        "mesh_loader reads mesh files from disk",
    ),
    (
        "thalos-language-service",
        "profile/loader reads a config file from disk",
    ),
    (
        "thalos-importer",
        "URDF asset resolution touches the filesystem",
    ),
];

/// Infrastructure / I/O crates forbidden in any pure-domain crate.
const FORBIDDEN_INFRA: &[&str] = &[
    "tokio",
    "tokio-serial",
    "async-std",
    "serialport",
    "sqlx",
    "rusqlite",
    "sqlite",
    "diesel",
    "tauri",
    "reqwest",
    "hyper",
    "axum",
    "actix-web",
    "warp",
    "tonic",
    "redis",
    "mongodb",
    "lapin",
    "rdkafka",
];

fn normalize(name: &str) -> String {
    name.trim().replace('_', "-")
}

/// Workspace root: `thalos-engine/` (parent of this crate).
fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("thalos-api lives inside the workspace")
        .to_path_buf()
}

fn collect_quoted(text: &str, out: &mut Vec<String>) {
    let mut rest = text;
    while let Some(start) = rest.find('"') {
        let after = &rest[start + 1..];
        match after.find('"') {
            Some(end) => {
                out.push(after[..end].to_string());
                rest = &after[end + 1..];
            }
            None => break,
        }
    }
}

/// Parse the `members = [...]` array from the workspace manifest.
fn workspace_members(root: &Path) -> Vec<String> {
    let manifest =
        fs::read_to_string(root.join("Cargo.toml")).expect("read workspace Cargo.toml");
    let mut members = Vec::new();
    let mut in_members = false;

    for line in manifest.lines() {
        let trimmed = line.trim();
        if !in_members {
            if let Some(rest) = trimmed.strip_prefix("members") {
                let rest = rest.trim_start();
                if let Some(rest) = rest.strip_prefix('=') {
                    let rest = rest.trim_start();
                    if let Some(inner) = rest.strip_prefix('[') {
                        in_members = true;
                        match inner.find(']') {
                            Some(end) => {
                                collect_quoted(&inner[..end], &mut members);
                                in_members = false;
                            }
                            None => collect_quoted(inner, &mut members),
                        }
                    }
                }
            }
        } else if let Some(end) = trimmed.find(']') {
            collect_quoted(&trimmed[..end], &mut members);
            in_members = false;
        } else {
            collect_quoted(trimmed, &mut members);
        }
    }

    members
}

/// Dependency names declared under `[dependencies]` or `[build-dependencies]`.
///
/// `[dev-dependencies]` are intentionally ignored: test-only tooling does not
/// make a domain crate impure at runtime.
fn declared_dependencies(manifest: &str) -> Vec<String> {
    let mut deps = Vec::new();
    let mut in_deps = false;

    for line in manifest.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('[') {
            in_deps = matches!(trimmed, "[dependencies]" | "[build-dependencies]");
            continue;
        }
        if !in_deps || trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        if let Some((key, _)) = trimmed.split_once('=') {
            deps.push(key.trim().to_string());
        }
    }

    deps
}

#[test]
fn workspace_members_are_classified() {
    let root = workspace_root();
    let members = workspace_members(&root);
    assert!(
        !members.is_empty(),
        "failed to parse workspace members from {}",
        root.join("Cargo.toml").display()
    );

    let pure: BTreeSet<String> = PURE_DOMAIN_CRATES.iter().map(|c| normalize(c)).collect();
    let excepted: BTreeSet<String> = EXCEPTED_CRATES.iter().map(|(c, _)| normalize(c)).collect();

    let unclassified: Vec<&String> = members
        .iter()
        .filter(|m| {
            let n = normalize(m);
            !pure.contains(&n) && !excepted.contains(&n)
        })
        .collect();

    assert!(
        unclassified.is_empty(),
        "Workspace members are not classified for the dependency purity guard: {unclassified:?}.\n\
         Declare each new crate in PURE_DOMAIN_CRATES or in EXCEPTED_CRATES (with a reason) \
         in thalos-api/tests/dependency_purity.rs — per ADR-020."
    );

    // Stale entries: classifications that no longer match a workspace member.
    let member_set: BTreeSet<String> = members.iter().map(|m| normalize(m)).collect();
    let stale: Vec<String> = pure
        .iter()
        .chain(excepted.iter())
        .filter(|c| !member_set.contains(*c))
        .cloned()
        .collect();
    assert!(
        stale.is_empty(),
        "Stale purity classifications (crate no longer a workspace member): {stale:?}."
    );
}

#[test]
fn pure_domain_crates_have_no_infrastructure_dependencies() {
    let root = workspace_root();
    let members = workspace_members(&root);

    let pure: BTreeSet<String> = PURE_DOMAIN_CRATES.iter().map(|c| normalize(c)).collect();
    let forbidden: BTreeSet<String> = FORBIDDEN_INFRA.iter().map(|c| normalize(c)).collect();

    let mut violations = Vec::new();
    for member in &members {
        if !pure.contains(&normalize(member)) {
            continue;
        }
        let manifest = fs::read_to_string(root.join(member).join("Cargo.toml"))
            .unwrap_or_else(|e| panic!("read {member}/Cargo.toml: {e}"));
        for dep in declared_dependencies(&manifest) {
            if forbidden.contains(&normalize(&dep)) {
                violations.push(format!("{member} -> {dep}"));
            }
        }
    }

    assert!(
        violations.is_empty(),
        "Pure-domain engine crates declare infrastructure/I-O dependencies:\n{}\n\
         Per ADR-020, pure domain crates must not depend on infrastructure. Either remove the \
         dependency, or move the crate to EXCEPTED_CRATES (with a reason).",
        violations.join("\n")
    );
}
