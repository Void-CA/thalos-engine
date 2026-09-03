use std::collections::HashMap;
use std::path::PathBuf;

use thalos_importer::assets::{AssetKind, AssetReference, AssetRole};
use thalos_importer::assets::resolver::{Resolution, UriResolver};
use thalos_importer::assets::resolve_candidate;
use thalos_importer::{import_urdf_resolved, urdf};

const ABB_IRB140: &str = include_str!("fixtures/robots/abb_irb140/robot.urdf");
const PLANAR_2R: &str = include_str!("fixtures/robots/planar_2r/robot.urdf");

fn sample_references() -> Vec<AssetReference> {
    vec![
        AssetReference { uri: "package://abb_irb140_support/meshes/irb140/visual/base_link.stl".into(), kind: AssetKind::Mesh, role: AssetRole::Visual },
        AssetReference { uri: "package://abb_irb140_support/meshes/irb140/visual/link_1.stl".into(), kind: AssetKind::Mesh, role: AssetRole::Visual },
        AssetReference { uri: "package://abb_irb140_support/meshes/irb140/visual/link_2.stl".into(), kind: AssetKind::Mesh, role: AssetRole::Visual },
        AssetReference { uri: "package://abb_irb140_support/meshes/irb140/visual/link_3.stl".into(), kind: AssetKind::Mesh, role: AssetRole::Visual },
        AssetReference { uri: "package://abb_irb140_support/meshes/irb140/visual/link_4.stl".into(), kind: AssetKind::Mesh, role: AssetRole::Visual },
    ]
}

fn rewrite_test_resolution() -> Resolution {
    let mut resolved = HashMap::new();
    resolved.insert(
        "package://abb_irb140_support/meshes/irb140/visual/base_link.stl".into(),
        PathBuf::from("/data/robots/abb/meshes/visual/base_link.stl"),
    );
    Resolution { resolved, missing: vec![] }
}

#[test]
fn resolve_candidate_rewrites_mesh_filenames() {
    let candidate = urdf::parse(ABB_IRB140).unwrap();
    let resolution = rewrite_test_resolution();

    let (mut candidate, diags) = resolve_candidate(candidate, &resolution);

    // Other meshes not in the resolution should generate warnings
    let warnings: Vec<_> = diags.iter().filter(|d| matches!(d, thalos_importer::ImportDiagnostic::Warning { .. })).collect();
    assert_eq!(warnings.len(), 13, "should emit 13 warnings for 13 unresolved meshes");

    // Walk visual/collision and find the base_link mesh — it should be rewritten
    let mut found = false;
    for body in &mut candidate.raw_bodies {
        for visual in &mut body.visual {
            if let thalos_models::geometry::Geometry::Mesh { filename, .. } = &mut visual.geometry {
                if filename.contains("base_link") && filename.contains("/visual/") {
                    assert_eq!(filename, "/data/robots/abb/meshes/visual/base_link.stl");
                    found = true;
                }
            }
        }
    }
    assert!(found, "base_link visual mesh should have been rewritten");

    // Other meshes should remain as original URIs
    for body in &candidate.raw_bodies {
        for visual in &body.visual {
            if let thalos_models::geometry::Geometry::Mesh { filename, .. } = &visual.geometry {
                if !filename.contains("base_link") || !filename.contains("/visual/") {
                    assert!(filename.starts_with("package://"), "unresolved mesh should keep original URI: {filename}");
                }
            }
        }
    }
}

#[test]
fn resolve_candidate_emits_warnings_for_missing() {
    let candidate = urdf::parse(ABB_IRB140).unwrap();

    // Empty resolution — all meshes missing
    let resolution = Resolution::default();
    let (_, diags) = resolve_candidate(candidate, &resolution);

    // Should get one warning per unique mesh URI (14 total)
    let warnings: Vec<_> = diags.iter().filter(|d| matches!(d, thalos_importer::ImportDiagnostic::Warning { .. })).collect();
    assert_eq!(warnings.len(), 14, "should emit 14 warnings for 14 unique missing meshes");
}

#[test]
fn resolve_candidate_preserves_primitives() {
    let candidate = urdf::parse(PLANAR_2R).unwrap();
    let resolution = Resolution::default();
    let (candidate, diags) = resolve_candidate(candidate, &resolution);

    // No meshes in planar_2r → no diagnostics
    assert!(diags.is_empty());

    // Verify box/cylinder geometries are untouched
    for body in &candidate.raw_bodies {
        for visual in &body.visual {
            match &visual.geometry {
                thalos_models::geometry::Geometry::Box { .. } |
                thalos_models::geometry::Geometry::Cylinder { .. } => {}
                other => panic!("unexpected geometry type: {other:?}"),
            }
        }
    }
}

#[test]
fn import_urdf_resolved_produces_robot_with_resolved_paths() {
    let mut resolved = HashMap::new();
    // Simulate full resolution for all ABB IRB 140 visual meshes
    for i in 0..=6 {
        let name = if i == 0 { "base_link".to_string() } else { format!("link_{i}") };
        resolved.insert(
            format!("package://abb_irb140_support/meshes/irb140/visual/{name}.stl"),
            PathBuf::from(format!("/data/robots/abb/meshes/visual/{name}.stl")),
        );
    }
    let resolution = Resolution { resolved, missing: vec![] };

    let result = import_urdf_resolved(ABB_IRB140, &resolution).unwrap();

    // Robot imported successfully
    assert_eq!(result.robot.links.len(), 9);

    // At least some meshes should now have absolute paths
    let mut has_resolved = false;
    for link in result.robot.links.values() {
        for visual in &link.visual {
            if let thalos_models::geometry::Geometry::Mesh { filename, .. } = &visual.geometry {
                if filename.starts_with("/data/") {
                    has_resolved = true;
                    break;
                }
            }
        }
    }
    assert!(has_resolved, "at least one mesh should have a resolved absolute path");
}

#[test]
fn import_urdf_with_empty_resolution_matches_import_urdf() {
    let resolution = Resolution::default();
    let result_resolved = import_urdf_resolved(PLANAR_2R, &resolution).unwrap();
    let result_original = urdf::import_urdf(PLANAR_2R).unwrap();

    // Both should produce identical robots (no meshes = no difference)
    assert_eq!(result_resolved.robot.links.len(), result_original.links.len());
    assert_eq!(result_resolved.robot.joints.len(), result_original.joints.len());
}

#[test]
fn uri_resolver_batch_resolve() {
    let refs = sample_references();
    let resolver = UriResolver::new();

    // No base_dir, no package mappings → all URIs can't resolve
    let resolution = resolver.resolve(&refs);
    assert_eq!(resolution.resolved.len(), 0);
    assert_eq!(resolution.missing.len(), 5);
}
