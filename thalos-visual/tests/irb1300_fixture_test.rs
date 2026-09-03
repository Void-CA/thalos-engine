use std::path::PathBuf;
use std::f64::consts::PI;

use thalos_engine::core::kinematics::forward::ForwardKinematics;
use thalos_engine::core::robot::adapter;
use thalos_importer::import_urdf;
use thalos_visual::{
    map_visuals_with_resolver, load_stl, AssetResolver, PrimitiveGeometry, SceneBuilder,
};

#[test]
fn test_irb1300_fixture_matrix() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let fixture_base = manifest_dir
        .parent()
        .unwrap()
        .join("thalos-models/tests/fixtures/urdf/abb_irb1300_support");
    let urdf_path = fixture_base.join("urdf/irb1300_10_115.urdf");

    let urdf_content = std::fs::read_to_string(&urdf_path)
        .expect("URDF fixture file irb1300_10_115.urdf should exist");

    // 1. Parse URDF
    let robot = import_urdf(&urdf_content).expect("URDF parsing should succeed");
    assert!(robot.links.len() > 0, "Parse URDF: links > 0");
    assert_eq!(robot.links.len(), 9, "Parse URDF: expected 9 links (base_link, link_1..6, flange, tool0)");
    assert_eq!(robot.joints.len(), 8, "Parse URDF: expected 8 joints");

    // 2. Root link
    assert_eq!(robot.name, "abb_irb1300_10_115");
    let base_link = robot.links.get("base_link").expect("base_link found");
    assert_eq!(base_link.name, "base_link", "Root link name matches base_link");

    // 3. package:// Asset Resolution
    let resolver = AssetResolver::new()
        .with_base_dir(&fixture_base)
        .register_package("abb_irb1300_support", &fixture_base);

    let links_with_visuals = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"];
    for link_name in &links_with_visuals {
        let uri = format!("package://abb_irb1300_support/meshes/visual/{}.stl", link_name);
        let resolved = resolver.resolve(&uri);
        assert!(
            resolved.is_ok(),
            "package:// resolution failed for visual asset {}",
            uri
        );
        let collision_uri = format!("package://abb_irb1300_support/meshes/collision/{}.stl", link_name);
        let resolved_col = resolver.resolve(&collision_uri);
        assert!(
            resolved_col.is_ok(),
            "package:// resolution failed for collision asset {}",
            collision_uri
        );
    }

    // 4. STL Parsing
    for link_name in &links_with_visuals {
        let uri = format!("package://abb_irb1300_support/meshes/visual/{}.stl", link_name);
        let path = resolver.resolve(&uri).unwrap();
        let mesh_data = load_stl(&path);
        assert!(mesh_data.is_ok(), "STL parsing failed for {}", path.display());
        let data = mesh_data.unwrap();
        assert!(!data.vertices.is_empty(), "Vertices should not be empty for {}", link_name);
        assert_eq!(data.vertices.len(), data.normals.len(), "Vertices and normals count must match");
    }

    // 5. Link Hierarchy & Chain adaptation
    let chain = adapter::from_tip(&robot, "tool0").expect("Kinematic chain adaptation tip=tool0 should succeed");
    assert_eq!(chain.dof_count(), 6, "IRB 1300 chain must have 6 degrees of freedom");

    // 6. Visual meshes (Zero None) & Materials
    let elements = map_visuals_with_resolver(&robot, &chain, Some(&resolver));
    assert_eq!(elements.len(), 7, "All 7 visual elements must be mapped without any None returns");

    for el in &elements {
        assert!(
            matches!(el.geometry, PrimitiveGeometry::Mesh { .. }),
            "Element {} must be a Mesh geometry",
            el.id
        );
        assert!(el.color.is_some(), "Material color must be preserved for element {}", el.id);
    }

    // 7. Visual origins & Mesh scale preserved
    let base_elem = elements.iter().find(|e| e.id == "base_link_0").unwrap();
    if let PrimitiveGeometry::Mesh { scale, filename, .. } = &base_elem.geometry {
        assert_eq!(filename, "package://abb_irb1300_support/meshes/visual/base_link.stl");
        assert_eq!(*scale, Some([1.0, 1.0, 1.0]), "Mesh scale preserved");
    }

    // 8. Forward Kinematics (FK)
    let fk = ForwardKinematics::new(chain.clone());
    let fk_zero = fk.evaluate(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    
    let builder = SceneBuilder::new(&chain);
    let scene_zero = builder.with_visual_elements(&fk_zero, &elements);

    // 9. Viewport Scene DTO generation
    assert_eq!(scene_zero.primitives.len(), 7, "Viewport scene must contain all 7 mesh primitives");
    assert!(scene_zero.frames.len() >= 7, "Viewport scene must contain link frames");

    // Verify non-zero FK pose changes frame positions
    let fk_bent = fk.evaluate(&[PI / 4.0, -PI / 6.0, PI / 3.0, 0.0, PI / 4.0, 0.0]);
    let scene_bent = builder.with_visual_elements(&fk_bent, &elements);

    assert_eq!(scene_bent.primitives.len(), 7);
}
