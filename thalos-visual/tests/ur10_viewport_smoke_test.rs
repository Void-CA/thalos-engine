use std::f64::consts::PI;
use std::path::PathBuf;

use thalos_engine::core::kinematics::forward::ForwardKinematics;
use thalos_engine::core::robot::adapter;
use thalos_importer::import_urdf;
use thalos_visual::{
    map_visuals_with_resolver, AssetResolver, PrimitiveGeometry, SceneBuilder,
};

#[test]
fn test_ur10_viewport_smoke_test_matrix() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let fixture_base = manifest_dir
        .parent()
        .unwrap()
        .join("thalos-models/tests/fixtures/urdf/ur_description");
    let urdf_path = fixture_base.join("urdf/ur10.urdf");

    let urdf_content = std::fs::read_to_string(&urdf_path)
        .expect("UR10 URDF fixture must exist");

    // 1. URDF Import
    let robot = import_urdf(&urdf_content).expect("UR10 URDF should parse");
    let resolver = AssetResolver::new()
        .with_base_dir(&fixture_base)
        .register_package("ur_description", &fixture_base);

    let chain = adapter::from_tip(&robot, "tool0").expect("UR10 chain tool0 should adapt");
    let fk = ForwardKinematics::new(chain.clone());

    // 2. Extract visual elements
    let elements = map_visuals_with_resolver(&robot, &chain, Some(&resolver));

    // Assertion 1 & 2: 7 visual DAE meshes with non-empty vertices & normals
    assert_eq!(elements.len(), 7, "UR10 viewport must receive exactly 7 visual elements");
    for el in &elements {
        if let PrimitiveGeometry::Mesh { filename, vertices, normals, scale } = &el.geometry {
            assert!(filename.ends_with(".dae"), "Visual element {} must be a DAE mesh", el.id);
            assert!(!vertices.is_empty(), "Visual element {} vertices must not be empty", el.id);
            assert_eq!(vertices.len(), normals.len(), "Normals count must match vertices count for {}", el.id);
            assert_eq!(scale.unwrap_or([1.0, 1.0, 1.0]), [1.0, 1.0, 1.0], "Effective scale must be [1.0, 1.0, 1.0] for {}", el.id);
        } else {
            panic!("Element {} is not a Mesh", el.id);
        }
    }

    // Assertion 3 & 4: Check frame hierarchy and visual local origins
    let builder = SceneBuilder::new(&chain);
    
    // Evaluate at zero pose q = [0, 0, 0, 0, 0, 0]
    let q_zero = vec![0.0; 6];
    let fk_zero = fk.evaluate(&q_zero);
    let scene_zero = builder.with_visual_elements(&fk_zero, &elements);

    assert_eq!(scene_zero.primitives.len(), 7, "Scene DTO must have 7 primitives");
    assert_eq!(scene_zero.frames.len(), 9, "Scene DTO must have 9 frames (including world)");

    // Assertion 5: Dynamic Joint Movement Test (modifying joint 2: shoulder_lift_joint)
    // q_bent = [0.0, PI / 4.0, 0.0, 0.0, 0.0, 0.0]
    let mut q_bent = vec![0.0; 6];
    q_bent[1] = PI / 4.0;
    let fk_bent = fk.evaluate(&q_bent);
    let scene_bent = builder.with_visual_elements(&fk_bent, &elements);

    let base_frame_zero = scene_zero.frames.iter().find(|f| f.id == "base_link").unwrap();
    let upper_frame_zero = scene_zero.frames.iter().find(|f| f.id == "upper_arm_link").unwrap();
    let upper_frame_bent = scene_bent.frames.iter().find(|f| f.id == "upper_arm_link").unwrap();
    let forearm_frame_zero = scene_zero.frames.iter().find(|f| f.id == "forearm_link").unwrap();
    let forearm_frame_bent = scene_bent.frames.iter().find(|f| f.id == "forearm_link").unwrap();

    // Base frame does not move
    assert_eq!(base_frame_zero.translation, [0.0, 0.0, 0.0]);

    // Upper arm frame orientation changes due to joint 2 rotation
    assert_ne!(
        upper_frame_zero.rotation, upper_frame_bent.rotation,
        "Upper arm frame orientation must change when joint 2 is rotated"
    );

    // Forearm frame translation moves in 3D world space as joint 2 rotates
    let forearm_moved = ((forearm_frame_bent.translation[0] - forearm_frame_zero.translation[0]).powi(2)
        + (forearm_frame_bent.translation[1] - forearm_frame_zero.translation[1]).powi(2)
        + (forearm_frame_bent.translation[2] - forearm_frame_zero.translation[2]).powi(2))
    .sqrt();
    assert!(forearm_moved > 0.05, "Forearm frame must translate in world space (moved = {}m)", forearm_moved);

    // Primitive local origins relative to parent frame MUST remain invariant
    let prim_upper_zero = scene_zero.primitives.iter().find(|p| p.frame_id == "upper_arm_link").unwrap();
    let prim_upper_bent = scene_bent.primitives.iter().find(|p| p.frame_id == "upper_arm_link").unwrap();
    assert_eq!(
        prim_upper_zero.translation, prim_upper_bent.translation,
        "Local primitive translation must be invariant under joint movement"
    );
    assert_eq!(
        prim_upper_zero.rotation, prim_upper_bent.rotation,
        "Local primitive rotation must be invariant under joint movement"
    );

    // Assertion 6: Collision mesh isolation (0 collision meshes in visual scene DTO)
    for p in &scene_zero.primitives {
        if let PrimitiveGeometry::Mesh { filename, .. } = &p.geometry {
            assert!(!filename.contains("collision"), "Collision mesh {} must NOT be in visual scene DTO", filename);
        }
    }

    // Assertion 7 & 8: Check physical reach & unit bounds (~1.3m reach for UR10)
    let tool0_frame = scene_zero.frames.iter().find(|f| f.id == "tool0").unwrap();
    let reach = (tool0_frame.translation[0].powi(2)
        + tool0_frame.translation[1].powi(2)
        + tool0_frame.translation[2].powi(2))
    .sqrt();
    assert!(reach > 0.5 && reach < 2.5, "UR10 physical reach must be ~1.3m (got {})", reach);
}
