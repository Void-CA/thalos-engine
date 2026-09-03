use std::f64::consts::PI;
use std::fs::File;
use std::io::Write;
use tempfile::tempdir;

use thalos_engine::core::kinematics::forward::ForwardKinematics;
use thalos_engine::core::robot::adapter;
use thalos_importer::import_urdf;
use thalos_visual::{
    map_visuals_with_resolver, AssetResolver, PrimitiveGeometry, SceneBuilder,
};

fn create_binary_stl(triangles: &[[ [f32; 3]; 4 ]]) -> Vec<u8> {
    let mut buf = Vec::new();
    // 80 bytes header
    buf.extend_from_slice(&[0u8; 80]);
    // Number of triangles (u32 LE)
    buf.extend_from_slice(&(triangles.len() as u32).to_le_bytes());

    for tri in triangles {
        // normal (3 x f32)
        for n in tri[0] {
            buf.extend_from_slice(&n.to_le_bytes());
        }
        // v1, v2, v3 (3 x 3 x f32)
        for i in 1..=3 {
            for v in tri[i] {
                buf.extend_from_slice(&v.to_le_bytes());
            }
        }
        // attribute byte count (u16 LE)
        buf.extend_from_slice(&0u16.to_le_bytes());
    }

    buf
}

#[test]
fn test_multilink_mesh_visual_integration() {
    let dir = tempdir().expect("tempdir creation failed");
    let meshes_dir = dir.path().join("meshes");
    std::fs::create_dir_all(&meshes_dir).unwrap();

    // Mesh A (1 triangle): normal [0, 0, 1]
    let mesh_a_bytes = create_binary_stl(&[
        [
            [0.0, 0.0, 1.0], // normal
            [0.0, 0.0, 0.0], // v1
            [1.0, 0.0, 0.0], // v2
            [0.0, 1.0, 0.0], // v3
        ],
    ]);
    let mesh_a_path = meshes_dir.join("mesh_a.stl");
    let mut f_a = File::create(&mesh_a_path).unwrap();
    f_a.write_all(&mesh_a_bytes).unwrap();

    // Mesh B (2 triangles): normals [1, 0, 0]
    let mesh_b_bytes = create_binary_stl(&[
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
    ]);
    let mesh_b_path = meshes_dir.join("mesh_b.stl");
    let mut f_b = File::create(&mesh_b_path).unwrap();
    f_b.write_all(&mesh_b_bytes).unwrap();

    let urdf_str = r#"<?xml version="1.0"?>
<robot name="multilink_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

  <link name="link_1">
    <visual>
      <origin xyz="0.05 0 0.2" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot/meshes/mesh_a.stl" scale="2.0 1.0 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

  <link name="link_2">
    <visual>
      <origin xyz="0 0.1 0.3" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot/meshes/mesh_b.stl" scale="1.0 1.0 1.0"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
"#;

    let robot = import_urdf(urdf_str).expect("URDF parsing failed");
    let chain = adapter::auto(&robot).expect("Chain adaptation failed");

    let resolver = AssetResolver::new()
        .with_base_dir(dir.path())
        .register_package("robot", dir.path());

    let elements = map_visuals_with_resolver(&robot, &chain, Some(&resolver));

    let fk = ForwardKinematics::new(chain.clone());
    let fk_result_zero = fk.evaluate(&[0.0, 0.0]);

    let builder = SceneBuilder::new(&chain);
    let scene = builder.with_visual_elements(&fk_result_zero, &elements);

    // 1. Appear all primitives (1 Box + 2 Meshes)
    assert_eq!(scene.primitives.len(), 3, "Expected 3 primitives in total");

    let base_prim = scene
        .primitives
        .iter()
        .find(|p| p.id == "base_link_0")
        .expect("base_link_0 primitive missing");
    let link1_prim = scene
        .primitives
        .iter()
        .find(|p| p.id == "link_1_0")
        .expect("link_1_0 primitive missing");
    let link2_prim = scene
        .primitives
        .iter()
        .find(|p| p.id == "link_2_0")
        .expect("link_2_0 primitive missing");

    // 2. Existing box primitive still works
    assert!(
        matches!(base_prim.geometry, PrimitiveGeometry::Box { width, height, depth } if (width - 0.2).abs() < 1e-6 && (height - 0.2).abs() < 1e-6 && (depth - 0.1).abs() < 1e-6),
        "base_link primitive should be box of size 0.2x0.2x0.1"
    );

    // 3. Link/joint frame hierarchy & non-world binding (no displacement to world origin)
    assert_eq!(base_prim.frame_id, "base_link");
    assert_eq!(link1_prim.frame_id, "link_1");
    assert_eq!(link2_prim.frame_id, "link_2");

    // 4. Local visual.origin preserved
    assert_eq!(link1_prim.translation, [0.05, 0.0, 0.2]);
    assert_eq!(link2_prim.translation, [0.0, 0.1, 0.3]);

    // 5. Mesh scale & vertices/normals respected
    if let PrimitiveGeometry::Mesh {
        filename,
        scale,
        vertices,
        normals,
    } = &link1_prim.geometry
    {
        assert_eq!(filename, "package://robot/meshes/mesh_a.stl");
        assert_eq!(*scale, Some([2.0, 1.0, 0.5]));
        assert_eq!(vertices.len(), 9, "Mesh A should have 3 vertices (9 floats)");
        assert_eq!(normals.len(), 9, "Mesh A should have 3 normals (9 floats)");
    } else {
        panic!("link_1_0 primitive is not a Mesh");
    }

    if let PrimitiveGeometry::Mesh {
        filename,
        scale,
        vertices,
        normals,
    } = &link2_prim.geometry
    {
        assert_eq!(filename, "package://robot/meshes/mesh_b.stl");
        assert_eq!(*scale, Some([1.0, 1.0, 1.0]));
        assert_eq!(vertices.len(), 18, "Mesh B should have 6 vertices (18 floats)");
        assert_eq!(normals.len(), 18, "Mesh B should have 6 normals (18 floats)");
    } else {
        panic!("link_2_0 primitive is not a Mesh");
    }

    // 6. Joint transformation check: when joint angles change, frame positions update correctly
    let fk_result_bent = fk.evaluate(&[PI / 2.0, 0.0]);
    let scene_bent = builder.with_visual_elements(&fk_result_bent, &elements);

    let _frame1_zero = scene
        .frames
        .iter()
        .find(|f| f.id == "link_1")
        .expect("link_1 frame expected");
    let frame1_bent = scene_bent
        .frames
        .iter()
        .find(|f| f.id == "link_1")
        .expect("link_1 frame expected");

    // joint_1 rotates by +90deg around Z
    assert!((frame1_bent.rotation[3] - (PI / 4.0).sin()).abs() < 1e-4, "link_1 frame should rotate around Z");

    // The primitives themselves retain local translation relative to frame
    let link1_prim_bent = scene_bent
        .primitives
        .iter()
        .find(|p| p.id == "link_1_0")
        .unwrap();
    assert_eq!(link1_prim_bent.translation, [0.05, 0.0, 0.2]);
}
