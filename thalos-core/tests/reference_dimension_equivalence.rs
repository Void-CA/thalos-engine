//! Integration test: catalog and URDF sources of the same robot must yield
//! the same chain-side `L_ref` (spec reference-dimension-fix "Catalog-URDF
//! Equivalence", ε < 1e-9) — for BOTH reference-dimension concepts
//! (remediation): the scene factor (all segments) and the manipulability
//! factor (moving segments only).
//!
//! The URDF fixture (`thalos-models/tests/fixtures/scara.urdf`) matches the
//! canonical `ScaraSpec`: base_height=0.5, a1=1.0, a2=0.8. The URDF adapter
//! stores lengths on joint origins (links are identity), the catalog stores
//! them on link transforms — the shared functions must reconcile both
//! sources. Scene L_ref = 2.3 (base 0.5 + a1 1.0 + a2 0.8); moving L_ref =
//! 1.8 (fixed base excluded). Regression guard for the old visual inline
//! ref_dim that only summed link translations and degenerated to 0.01 for
//! URDF robots.

use std::fs;
use std::path::PathBuf;

use thalos_core::models::scara::ScaraSpec;
use thalos_core::robot::adapter;
use thalos_core::robot::scale::{
    manipulability_reference_dimension, scene_reference_dimension,
};
use thalos_models::urdf::parser::parse_robot;

fn fixture_path() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .parent()
        .unwrap() // workspace root (thalos-engine/)
        .join("thalos-models/tests/fixtures/scara.urdf")
}

fn load_urdf_scara_chain() -> thalos_core::robot::serial_chain::SerialChain {
    let source = fs::read_to_string(fixture_path()).expect("SCARA fixture file not found");
    let robot = parse_robot(&source).expect("SCARA should parse");
    adapter::from_tip(&robot, "tool0").expect("from_tip with tool0")
}

#[test]
fn catalog_and_urdf_scara_share_scene_l_ref() {
    // Scene definition (all segments): catalog = URDF = 2.3.
    let catalog_chain = ScaraSpec::canonical().build();
    let urdf_chain = load_urdf_scara_chain();

    let catalog_l_ref = scene_reference_dimension(&catalog_chain);
    let urdf_l_ref = scene_reference_dimension(&urdf_chain);
    assert!(
        (catalog_l_ref - urdf_l_ref).abs() < 1e-9,
        "scene L_ref: catalog {catalog_l_ref} vs URDF {urdf_l_ref} must agree within 1e-9"
    );
    assert!(
        (catalog_l_ref - 2.3).abs() < 1e-9,
        "scene SCARA L_ref must be 2.3, got {catalog_l_ref}"
    );
}

#[test]
fn catalog_and_urdf_scara_share_manipulability_l_ref() {
    // Moving-only definition (dof > 0): catalog = URDF = 1.8 (fixed base
    // excluded from the normalization divisor).
    let catalog_chain = ScaraSpec::canonical().build();
    let urdf_chain = load_urdf_scara_chain();

    let catalog_l_ref = manipulability_reference_dimension(&catalog_chain);
    let urdf_l_ref = manipulability_reference_dimension(&urdf_chain);
    assert!(
        (catalog_l_ref - urdf_l_ref).abs() < 1e-9,
        "manipulability L_ref: catalog {catalog_l_ref} vs URDF {urdf_l_ref} must agree within 1e-9"
    );
    assert!(
        (catalog_l_ref - 1.8).abs() < 1e-9,
        "moving SCARA L_ref must be 1.8, got {catalog_l_ref}"
    );
}

#[test]
fn urdf_scara_l_ref_is_not_the_broken_floor() {
    // Regression: the old inline ref_dim (link translations only) collapsed
    // URDF robots to 0.01 (identity link transforms + .max(0.01) floor).
    // Both chain-side functions recover the true scale.
    let chain = load_urdf_scara_chain();

    let scene = scene_reference_dimension(&chain);
    assert!(
        (scene - 2.3).abs() < 1e-9,
        "URDF scene L_ref must be 2.3, got {scene}"
    );
    let moving = manipulability_reference_dimension(&chain);
    assert!(
        (moving - 1.8).abs() < 1e-9,
        "URDF moving L_ref must be 1.8, got {moving}"
    );
}
