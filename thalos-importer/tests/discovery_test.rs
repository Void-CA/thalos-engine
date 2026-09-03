use thalos_importer::assets::AssetKind;
use thalos_importer::assets::discovery::{UrdfAssetDiscovery, collect_asset_references};
use thalos_importer::assets::AssetDiscovery;
use thalos_importer::{UriResolver, urdf};

const ABB_IRB140: &str = include_str!("fixtures/robots/abb_irb140/robot.urdf");
const PLANAR_2R: &str = include_str!("fixtures/robots/planar_2r/robot.urdf");

#[test]
fn discover_mesh_refs_in_industrial_robot() {
    let discovery = UrdfAssetDiscovery::new();
    let refs = discovery.discover(ABB_IRB140).unwrap();

    // ABB IRB 140 has 14 unique mesh URIs:
    // 7 visual (base_link..link_6) + 7 collision (base_link..link_6)
    assert_eq!(refs.len(), 14);

    // All should be Mesh kind
    for asset_ref in &refs {
        assert_eq!(asset_ref.kind, AssetKind::Mesh);
    }

    // Check visual and collision URIs are both present
    let uris: Vec<&str> = refs.iter().map(|r| r.uri.as_str()).collect();
    assert!(uris.iter().any(|u| u.contains("/visual/base_link.stl")));
    assert!(uris.iter().any(|u| u.contains("/collision/base_link.stl")));
}

#[test]
fn discover_returns_empty_for_primitives_only_robot() {
    let discovery = UrdfAssetDiscovery::new();
    let refs = discovery.discover(PLANAR_2R).unwrap();
    assert!(refs.is_empty());
}

#[test]
fn discover_deduplicates_across_visual_and_collision() {
    let candidate = urdf::parse(ABB_IRB140).unwrap();
    let refs = collect_asset_references(&candidate);

    // The same mesh URI referenced in both visual and collision should appear once.
    let uris: Vec<&str> = refs.iter().map(|r| r.uri.as_str()).collect();
    let unique_count = uris.iter().collect::<std::collections::HashSet<_>>().len();
    assert_eq!(uris.len(), unique_count, "URIs should be deduplicated");
}

#[test]
fn discover_infers_mesh_kind_from_extension() {
    let candidate = urdf::parse(ABB_IRB140).unwrap();
    let refs = collect_asset_references(&candidate);

    for asset_ref in &refs {
        assert_eq!(asset_ref.kind, AssetKind::Mesh, "STL files should infer Mesh kind");
    }
}

#[test]
fn discover_handles_invalid_urdf() {
    let discovery = UrdfAssetDiscovery::new();
    // Truncated XML should fail to parse
    let result = discovery.discover("<robot name=\"test\"><link name=\"a\"/><joint name=\"j");
    assert!(result.is_err());
}

#[test]
fn uri_resolver_finds_abb_irb140_support_from_robots_dir() {
    // The fixture lives at: tests/fixtures/robots/abb_irb140_support/meshes/irb140/visual/...
    // The resolver should find it when given the robots directory as base_dir
    let robots_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("robots");

    let resolver = UriResolver::new().with_base_dir(&robots_dir);

    // Try to resolve a known mesh URI from the ABB IRB 140 URDF
    let uri = "package://abb_irb140_support/meshes/irb140/visual/base_link.stl";
    let result = resolver.resolve_uri_strict(uri);

    assert!(result.is_ok(), "should resolve abb_irb140_support mesh from robots dir: {:?}", result.err());
    let path = result.unwrap();
    assert!(path.exists(), "resolved path should exist on disk: {}", path.display());
    assert!(path.to_string_lossy().contains("base_link.stl"), "should resolve to base_link.stl");
}

#[test]
fn uri_resolver_batch_resolve_abb_irb140_visual_meshes() {
    let robots_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("robots");

    let resolver = UriResolver::new().with_base_dir(&robots_dir);

    // All visual meshes from the ABB IRB 140 URDF
    let refs: Vec<thalos_importer::assets::AssetReference> = (0..=6)
        .map(|i| {
            let name = if i == 0 { "base_link".to_string() } else { format!("link_{i}") };
            thalos_importer::assets::AssetReference {
                uri: format!("package://abb_irb140_support/meshes/irb140/visual/{name}.stl"),
                kind: AssetKind::Mesh,
            }
        })
        .collect();

    let resolution = resolver.resolve(&refs);

    // All 7 visual meshes should resolve
    assert_eq!(resolution.resolved.len(), 7, "all 7 visual meshes should resolve");
    assert_eq!(resolution.missing.len(), 0, "no meshes should be missing");

    // Verify each resolved path exists
    for (uri, path) in resolution.resolved.iter() {
        assert!(path.exists(), "resolved path should exist: {uri} -> {}", path.display());
    }
}
