use std::f64::consts::PI;

use thalos_engine::core::{kinematics::forward::ForwardKinematics, models::planar_2r::Planar2RSpec};
use thalos_visual::{SceneBuilder, SceneDiff, VisualPrecision};

#[test]
fn planar_2r_zero_config() {
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let result = fk.evaluate(&[0.0, 0.0]);

    let builder = SceneBuilder::new(&robot);
    let scene = builder.from_fk(&result);

    insta::assert_json_snapshot!(scene);
}

#[test]
fn planar_2r_bent_config() {
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let result = fk.evaluate(&[PI / 2.0, 0.0]);

    let builder = SceneBuilder::new(&robot);
    let scene = builder.from_fk(&result);

    insta::assert_json_snapshot!(scene);
}

#[test]
fn precision_canonicalizes_noise() {
    let precision = VisualPrecision {
        epsilon_zero: 1e-10,
        decimal_places: 6,
    };

    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let result = fk.evaluate(&[PI / 2.0, 0.0]);

    let builder = SceneBuilder::new(&robot).with_precision(precision);
    let scene = builder.from_fk(&result);

    let link_1 = scene
        .frames
        .iter()
        .find(|f| f.id == "link_1")
        .expect("link_1 frame expected");

    assert_eq!(
        link_1.rotation[1], 0.0,
        "x should be exactly 0 after normalization"
    );
    assert_eq!(
        link_1.rotation[2], 0.0,
        "y should be exactly 0 after normalization"
    );
    assert!(
        (link_1.rotation[0] - 0.707107).abs() < 1e-12,
        "w should be ~0.707107, got {}",
        link_1.rotation[0]
    );
    assert!(
        (link_1.rotation[3] - 0.707107).abs() < 1e-12,
        "z should be ~0.707107, got {}",
        link_1.rotation[3]
    );

    assert_eq!(link_1.translation[0], 0.0, "tx should be exactly 0");
    assert_eq!(link_1.translation[1], 1.0, "ty should be 1");
}

#[test]
fn diff_detects_translation_and_rotation() {
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let builder = SceneBuilder::new(&robot);

    let old = builder.from_fk(&fk.evaluate(&[0.0, 0.0]));
    let new = builder.from_fk(&fk.evaluate(&[PI / 2.0, 0.0]));

    let diff = SceneDiff::between(&old, &new, 1e-6);

    assert!(diff.frames_added.is_empty(), "no frames should be added");
    assert!(
        diff.frames_removed.is_empty(),
        "no frames should be removed"
    );
    assert!(
        !diff.changed_frames.is_empty(),
        "frames should have changed"
    );

    let link_1 = diff
        .changed_frames
        .iter()
        .find(|c| c.id == "link_1")
        .expect("link_1 should have changed");

    assert!(link_1.translation_delta > 0.0, "link_1 should have moved");
    assert!(
        link_1.rotation_angle_deg > 0.0,
        "link_1 should have rotated"
    );

    let link_2 = diff
        .changed_frames
        .iter()
        .find(|c| c.id == "link_2")
        .expect("link_2 should have changed");

    assert!(link_2.translation_delta > 0.0, "link_2 should have moved");
}

#[test]
fn diff_identical_scenes() {
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let builder = SceneBuilder::new(&robot);

    let old = builder.from_fk(&fk.evaluate(&[0.3, 0.5]));
    let new = builder.from_fk(&fk.evaluate(&[0.3, 0.5]));

    let diff = SceneDiff::between(&old, &new, 1e-6);

    assert!(diff.frames_added.is_empty());
    assert!(diff.frames_removed.is_empty());
    assert!(
        diff.changed_frames.is_empty(),
        "identical scenes should have no diff"
    );
}

#[test]
fn dump_json() {
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let result = fk.evaluate(&[0.0, 0.0]);

    let builder = SceneBuilder::new(&robot);
    let scene = builder.from_fk(&result);

    let json = serde_json::to_string_pretty(&scene).unwrap();
    println!("{}", json);
}

/// Regression guard for the URDF reference-dimension bug: the old inline
/// ref_dim summed ONLY link translations, but the URDF adapter stores
/// lengths on joint origins (links are identity), so URDF robots collapsed
/// to the 0.01 floor. The chain-side `reference_dimension` (link + origin
/// norms) recovers the true scale: SCARA canonical = 0.5 + 1.0 + 0.8 = 2.3.
#[test]
fn urdf_scara_reference_dimension_is_not_the_broken_floor() {
    use std::fs;
    use std::path::PathBuf;

    use thalos_engine::core::kinematics::forward::ForwardKinematics;
    use thalos_engine::core::robot::adapter;
    use thalos_importer::import_urdf;

    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let fixture = manifest
        .parent()
        .unwrap()
        .join("thalos-models/tests/fixtures/scara.urdf");
    let source = fs::read_to_string(fixture).expect("SCARA fixture file not found");
    let robot = import_urdf(&source).expect("SCARA should parse");
    let chain = adapter::from_tip(&robot, "tool0").expect("from_tip with tool0");

    let fk = ForwardKinematics::new(chain.clone());
    let scene = SceneBuilder::new(&chain).from_fk(&fk.evaluate(&[0.0, 0.0, 0.0, 0.0]));

    assert!(
        (scene.reference_dimension - 2.3).abs() < 1e-9,
        "URDF SCARA reference_dimension must be 2.3, got {}",
        scene.reference_dimension
    );
}
