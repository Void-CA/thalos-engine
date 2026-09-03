use std::path::PathBuf;

use thalos_engine::core::kinematics::forward::ForwardKinematics;
use thalos_engine::core::robot::adapter;
use thalos_importer::import_urdf;
use thalos_visual::{
    map_visuals_with_resolver, load_stl, load_dae, AssetResolver, PrimitiveGeometry, SceneBuilder,
};

#[test]
fn test_ur10_fixture_matrix() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let fixture_base = manifest_dir
        .parent()
        .unwrap()
        .join("thalos-models/tests/fixtures/urdf/ur_description");
    let urdf_path = fixture_base.join("urdf/ur10.urdf");

    let urdf_content = std::fs::read_to_string(&urdf_path)
        .expect("URDF fixture file ur10.urdf should exist");

    // 1. Parse URDF completely
    let robot = import_urdf(&urdf_content).expect("URDF parsing should succeed");
    assert_eq!(robot.name, "ur10");
    assert_eq!(robot.links.len(), 9, "UR10 should have 9 links");
    assert_eq!(robot.joints.len(), 8, "UR10 should have 8 joints");

    // 2 & 3. package:// Asset Resolution for visual (DAE) and collision (STL) independently
    let resolver = AssetResolver::new()
        .with_base_dir(&fixture_base)
        .register_package("ur_description", &fixture_base);

    let links_with_assets = ["base", "shoulder", "upperarm", "forearm", "wrist1", "wrist2", "wrist3"];
    for link_name in &links_with_assets {
        let visual_uri = format!("package://ur_description/meshes/ur10/visual/{}.dae", link_name);
        let resolved_vis = resolver.resolve(&visual_uri);
        assert!(
            resolved_vis.is_ok(),
            "package:// resolution failed for visual DAE asset {}",
            visual_uri
        );

        let collision_uri = format!("package://ur_description/meshes/ur10/collision/{}.stl", link_name);
        let resolved_col = resolver.resolve(&collision_uri);
        assert!(
            resolved_col.is_ok(),
            "package:// resolution failed for collision STL asset {}",
            collision_uri
        );
    }

    // 4 & 5. Verify DAE (visual) and STL (collision) load independently
    for link_name in &links_with_assets {
        // Visual DAE
        let vis_uri = format!("package://ur_description/meshes/ur10/visual/{}.dae", link_name);
        let vis_path = resolver.resolve(&vis_uri).unwrap();
        let vis_mesh = load_dae(&vis_path).expect("Visual DAE parsing should succeed");
        assert_eq!(vis_mesh.vertices.len(), 9, "Visual DAE vertices must be 9 floats (1 triangle)");
        assert_eq!(vis_mesh.normals.len(), 9, "Visual DAE normals must be 9 floats");

        // Collision STL
        let col_uri = format!("package://ur_description/meshes/ur10/collision/{}.stl", link_name);
        let col_path = resolver.resolve(&col_uri).unwrap();
        let col_mesh = load_stl(&col_path).expect("Collision STL parsing should succeed");
        assert_eq!(col_mesh.vertices.len(), 9, "Collision STL vertices must be 9 floats (1 triangle)");
    }

    // 6. Kinematic Chain & FK
    let chain = adapter::from_tip(&robot, "tool0").expect("Chain tip=tool0 should adapt");
    assert_eq!(chain.dof_count(), 6, "UR10 chain must have 6 degrees of freedom");

    let fk = ForwardKinematics::new(chain.clone());
    let joint_angles = [0.1, -0.5, 1.2, -0.3, 0.8, 0.0];
    let fk_pose = fk.evaluate(&joint_angles);

    // 7 & 8. Visual mapping & Viewport DTO generation
    let elements = map_visuals_with_resolver(&robot, &chain, Some(&resolver));
    assert_eq!(elements.len(), 7, "7 visual elements expected");

    // Verify visual elements retain DAE filenames, non-empty vertices, and colors
    for el in &elements {
        if let PrimitiveGeometry::Mesh { filename, vertices, normals, .. } = &el.geometry {
            assert!(filename.ends_with(".dae"), "Visual element {} should reference a .dae mesh", el.id);
            assert_eq!(vertices.len(), 9, "Visual element {} mesh vertices must be loaded from DAE", el.id);
            assert_eq!(normals.len(), 9, "Visual element {} mesh normals must be loaded from DAE", el.id);
        } else {
            panic!("Visual element {} is not a Mesh", el.id);
        }
        assert!(el.color.is_some(), "Visual element {} must retain material color", el.id);
    }

    let builder = SceneBuilder::new(&chain);
    let scene = builder.with_visual_elements(&fk_pose, &elements);
    assert_eq!(scene.primitives.len(), 7, "Viewport scene DTO must contain 7 mesh primitives");
}
