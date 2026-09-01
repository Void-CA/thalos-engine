use std::f64::consts::PI;

use thalos_engine::core::{kinematics::forward::ForwardKinematics, models::planar_2r::Planar2RSpec};
use thalos_visual::{SceneBuilder, SceneError, SceneValidator, VisualScene};

#[test]
fn valid_scene_passes() {
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let builder = SceneBuilder::new(&robot);
    let scene = builder.from_fk(&fk.evaluate(&[0.5, 0.3]));

    let validator = SceneValidator::default();
    assert!(validator.validate(&scene).is_ok());
}

#[test]
fn missing_world_fails() {
    let scene = VisualScene::default();

    let validator = SceneValidator::default();
    assert_eq!(validator.validate(&scene), Err(SceneError::MissingWorld));
}

#[test]
fn duplicate_ids_fail() {
    let scene = VisualScene {
        frames: vec![
            frame("world", None, [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            frame(
                "link_1",
                Some("world"),
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ),
            frame(
                "link_1",
                Some("world"),
                [2.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ),
        ],
        ..Default::default()
    };

    let validator = SceneValidator::default();
    assert_eq!(
        validator.validate(&scene),
        Err(SceneError::DuplicateId {
            id: "link_1".into()
        })
    );
}

#[test]
fn missing_parent_fails() {
    let scene = VisualScene {
        frames: vec![
            frame("world", None, [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            frame(
                "link_1",
                Some("phantom"),
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ),
        ],
        ..Default::default()
    };

    let validator = SceneValidator::default();
    assert_eq!(
        validator.validate(&scene),
        Err(SceneError::MissingFrame("phantom".into()))
    );
}

#[test]
fn cycle_detected() {
    let scene = VisualScene {
        frames: vec![
            frame("world", None, [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            frame("a", Some("b"), [1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            frame("b", Some("a"), [2.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        ],
        ..Default::default()
    };

    let validator = SceneValidator::default();
    let result = validator.validate(&scene);
    assert!(result.is_err(), "cycle should be detected");
    match result.unwrap_err() {
        SceneError::BrokenTopology { frame } => {
            assert!(frame == "a" || frame == "b");
        }
        other => panic!("expected BrokenTopology, got {:?}", other),
    }
}

#[test]
fn orphan_frame_fails() {
    let scene = VisualScene {
        frames: vec![
            frame("world", None, [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            frame("orphan", None, [5.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        ],
        ..Default::default()
    };

    let validator = SceneValidator::default();
    let result = validator.validate(&scene);
    assert!(result.is_err(), "orphan frame should fail");
}

#[test]
fn nan_value_detected() {
    let scene = VisualScene {
        frames: vec![
            frame("world", None, [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            frame(
                "link_1",
                Some("world"),
                [f64::NAN, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ),
        ],
        ..Default::default()
    };

    let validator = SceneValidator::default();
    assert_eq!(
        validator.validate(&scene),
        Err(SceneError::NonFiniteValue {
            frame: "link_1".into()
        })
    );
}

#[test]
fn invalid_quaternion_detected() {
    let scene = VisualScene {
        frames: vec![
            frame("world", None, [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            frame(
                "link_1",
                Some("world"),
                [1.0, 0.0, 0.0],
                [5.0, 0.0, 0.0, 0.0],
            ),
        ],
        ..Default::default()
    };

    let validator = SceneValidator::default();
    let result = validator.validate(&scene);
    assert!(result.is_err(), "non-unit quaternion should fail");
    match result.unwrap_err() {
        SceneError::InvalidQuaternion { frame, norm } => {
            assert_eq!(frame, "link_1");
            assert!(
                (norm - 5.0).abs() < 1e-10,
                "norm should be ~5.0, got {}",
                norm
            );
        }
        other => panic!("expected InvalidQuaternion, got {:?}", other),
    }
}

#[test]
fn orphan_link_detected() {
    let scene = VisualScene {
        frames: vec![
            frame("world", None, [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            frame(
                "link_1",
                Some("world"),
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ),
        ],
        links: vec![
            link(0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            link(1, [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]),
        ],
        ..Default::default()
    };

    let validator = SceneValidator::default();
    let result = validator.validate(&scene);
    assert!(result.is_err(), "orphan link should fail");
    match result.unwrap_err() {
        SceneError::OrphanLink { index } => assert_eq!(index, 1),
        other => panic!("expected OrphanLink, got {:?}", other),
    }
}

#[test]
fn twists_mismatch_detected() {
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    let builder = SceneBuilder::new(&robot);
    let mut scene = builder.from_fk(&fk.evaluate(&[0.3, 0.5]));

    scene.twists.push(thalos_visual::VisualTwist {
        origin: [0.0; 3],
        linear: [0.0; 3],
        angular: [0.0; 3],
    });
    scene.twists.push(thalos_visual::VisualTwist {
        origin: [0.0; 3],
        linear: [0.0; 3],
        angular: [0.0; 3],
    });
    scene.twists.push(thalos_visual::VisualTwist {
        origin: [0.0; 3],
        linear: [0.0; 3],
        angular: [0.0; 3],
    });

    let validator = SceneValidator::default();
    let result = validator.validate(&scene);
    assert!(result.is_err(), "twists mismatch should fail");
    match result.unwrap_err() {
        SceneError::TwistsMismatch { expected, found } => {
            assert_eq!(expected, 2);
            assert_eq!(found, 3);
        }
        other => panic!("expected TwistsMismatch, got {:?}", other),
    }
}

fn frame(
    id: &str,
    parent: Option<&str>,
    translation: [f64; 3],
    rotation: [f64; 4],
) -> thalos_visual::VisualFrame {
    thalos_visual::VisualFrame {
        id: id.into(),
        parent: parent.map(|p| p.into()),
        translation,
        rotation,
        style: None,
    }
}

fn link(id: u32, start: [f64; 3], end: [f64; 3]) -> thalos_visual::VisualLink {
    thalos_visual::VisualLink { id, start, end }
}
