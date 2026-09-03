use thalos_importer::assets::AssetKind;
use thalos_importer::assets::discovery::{UrdfAssetDiscovery, collect_asset_references};
use thalos_importer::assets::AssetDiscovery;
use thalos_importer::urdf;

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
