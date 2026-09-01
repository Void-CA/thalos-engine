//! SceneFile validation — tiers (a) schema + (b) semantic + (c) robot compat.
//! Tier (a) is enforced by serde on parse; tier (b) is pure semantic checks
//! (duplicate IDs, negative dimensions, non-finite poses, unknown references);
//! tier (c) compares the SceneFile robot against the loaded runtime robot.
//!
//! Tier (d) — planning validity — is deliberately NOT a SceneFile concern; it is
//! non-blocking and handled by `POST /plan/analyze`.

use std::collections::HashSet;

use crate::scene_file::{SceneFile, SCENE_FILE_SCHEMA_VERSION};

/// Supported geometry types in v1 (box / cylinder / sphere).
const SUPPORTED_GEOMETRY_TYPES: [&str; 3] = ["box", "cylinder", "sphere"];
/// Supported location kind in v1.
const SUPPORTED_LOCATION_KIND: &str = "placement_target";

/// A semantic validation error found in a `SceneFile` (tier (b)).
#[derive(Debug, Clone, PartialEq)]
pub enum SceneFileError {
    /// `schema_version` is not the supported `"1"` (tier (a) value check —
    /// serde already rejects structurally invalid documents on parse).
    UnsupportedSchemaVersion(String),
    /// An id appears more than once across objects, fixtures, or locations.
    DuplicateId(String),
    /// A geometry dimension is negative.
    NegativeDimension(String),
    /// A pose contains a non-finite value.
    InvalidPose(String),
    /// Geometry `type` is not box/cylinder/sphere.
    UnsupportedGeometryType { id: String, r#type: String },
    /// A location `kind` other than `"placement_target"`.
    UnsupportedLocationKind { id: String, kind: String },
    /// An object's `location_ref` points to a location id that does not exist.
    UnknownReference { object: String, reference: String },
}

/// Robot compatibility mismatch (tier (c)) — the SceneFile robot `name` (D11:
/// stable identity) differs from the loaded runtime robot.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotMismatch {
    pub expected: String,
    pub loaded: String,
}

fn pose_is_finite(pose: &crate::pose::Pose) -> bool {
    pose.position.iter().chain(pose.orientation.iter()).all(|v| v.is_finite())
}

/// Validate a `SceneFile` at tiers (a)+(b). Returns `Ok(())` when no semantic
/// errors are found, or every detected `SceneFileError`.
pub fn validate_scene_file(file: &SceneFile) -> Result<(), Vec<SceneFileError>> {
    let mut errors = Vec::new();

    // Tier (a) value check: schema_version must be "1".
    if file.schema_version != SCENE_FILE_SCHEMA_VERSION {
        errors.push(SceneFileError::UnsupportedSchemaVersion(
            file.schema_version.clone(),
        ));
    }

    // Tier (b): duplicate ids across objects, fixtures, and locations.
    let mut seen: HashSet<&str> = HashSet::new();
    let all_ids = file
        .objects
        .iter()
        .map(|o| o.id.as_str())
        .chain(file.fixtures.iter().map(|f| f.id.as_str()))
        .chain(file.locations.iter().map(|l| l.id.as_str()));
    for id in all_ids {
        if !seen.insert(id) {
            errors.push(SceneFileError::DuplicateId(id.to_string()));
        }
    }

    // Tier (b): object checks — pose finiteness, location reference, geometry.
    for obj in &file.objects {
        if !pose_is_finite(&obj.pose) {
            errors.push(SceneFileError::InvalidPose(obj.id.clone()));
        }
        if let Some(reference) = &obj.location_ref {
            if !file.locations.iter().any(|l| &l.id == reference) {
                errors.push(SceneFileError::UnknownReference {
                    object: obj.id.clone(),
                    reference: reference.clone(),
                });
            }
        }
        if let Some(geom) = &obj.geometry {
            validate_geometry(&obj.id, geom, &mut errors);
        }
    }

    // Tier (b): fixture checks — pose finiteness and geometry.
    for fixture in &file.fixtures {
        if !pose_is_finite(&fixture.pose) {
            errors.push(SceneFileError::InvalidPose(fixture.id.clone()));
        }
        if let Some(geom) = &fixture.geometry {
            validate_geometry(&fixture.id, geom, &mut errors);
        }
    }

    // Tier (b): location checks — kind and pose finiteness.
    for loc in &file.locations {
        if loc.kind != SUPPORTED_LOCATION_KIND {
            errors.push(SceneFileError::UnsupportedLocationKind {
                id: loc.id.clone(),
                kind: loc.kind.clone(),
            });
        }
        if !pose_is_finite(&loc.pose) {
            errors.push(SceneFileError::InvalidPose(loc.id.clone()));
        }
    }

    // Tier (b): home pose must be finite.
    if !pose_is_finite(&file.home_pose) {
        errors.push(SceneFileError::InvalidPose("home_pose".into()));
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

fn validate_geometry(id: &str, geom: &crate::scene_file::GeometryDef, errors: &mut Vec<SceneFileError>) {
    if !SUPPORTED_GEOMETRY_TYPES.contains(&geom.r#type.as_str()) {
        errors.push(SceneFileError::UnsupportedGeometryType {
            id: id.to_string(),
            r#type: geom.r#type.clone(),
        });
    }
    if geom.size.iter().any(|d| *d < 0.0) {
        errors.push(SceneFileError::NegativeDimension(id.to_string()));
    }
}

/// Tier (c): verify the SceneFile robot's stable `name` matches the loaded
/// runtime robot. The API layer calls this with the runtime robot's name.
pub fn validate_robot_compat(
    file: &SceneFile,
    loaded_robot_name: &str,
) -> Result<(), RobotMismatch> {
    if file.robot.name == loaded_robot_name {
        Ok(())
    } else {
        Err(RobotMismatch {
            expected: file.robot.name.clone(),
            loaded: loaded_robot_name.to_string(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::Pose;
    use crate::scene_file::{GeometryDef, RobotRef, SceneFile, SceneFixtureDef, SceneLocationDef, SceneObjectDef};

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose {
            position: [x, y, z],
            orientation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    fn object(id: &str, kind: &str) -> SceneObjectDef {
        SceneObjectDef {
            id: id.into(),
            kind: kind.into(),
            name: None,
            location_ref: None,
            geometry: None,
            pose: pose(0.2, 0.1, 0.0),
        }
    }

    fn fixture(id: &str) -> SceneFixtureDef {
        SceneFixtureDef {
            id: id.into(),
            geometry: None,
            pose: pose(0.4, 0.0, 0.0),
        }
    }

    fn location(id: &str) -> SceneLocationDef {
        SceneLocationDef {
            id: id.into(),
            kind: "placement_target".into(),
            pose: pose(0.3, -0.2, 0.0),
        }
    }

    fn valid_file() -> SceneFile {
        SceneFile {
            schema_version: "1".into(),
            robot: RobotRef {
                name: "icebot".into(),
                urdf: "docs/execution/robot/icebot.urdf".into(),
            },
            objects: vec![object("box-1", "box")],
            fixtures: vec![fixture("fence-1")],
            locations: vec![location("tray-1")],
            home_pose: pose(0.0, 0.0, 0.5),
            approach_height: 0.05,
        }
    }

    // ── 1.3: tier (b) semantic validation ───────────────────────────────

    #[test]
    fn valid_scene_file_passes_validation() {
        assert_eq!(validate_scene_file(&valid_file()), Ok(()));
    }

    #[test]
    fn duplicate_object_ids_are_rejected() {
        let mut file = valid_file();
        file.objects.push(object("box-1", "bolt"));
        match validate_scene_file(&file) {
            Err(errors) => assert!(
                errors.contains(&SceneFileError::DuplicateId("box-1".into())),
                "expected DuplicateId for box-1, got {errors:?}"
            ),
            Ok(()) => panic!("duplicate ids must fail validation"),
        }
    }

    #[test]
    fn negative_geometry_dimensions_are_rejected() {
        let mut file = valid_file();
        file.objects[0].geometry = Some(GeometryDef {
            r#type: "box".into(),
            size: vec![0.1, -0.1, 0.1],
        });
        match validate_scene_file(&file) {
            Err(errors) => assert!(
                errors.contains(&SceneFileError::NegativeDimension("box-1".into())),
                "expected NegativeDimension, got {errors:?}"
            ),
            Ok(()) => panic!("negative geometry dimension must fail validation"),
        }
    }

    #[test]
    fn non_finite_pose_is_rejected() {
        let mut file = valid_file();
        file.objects[0].pose.position[2] = f64::NAN;
        match validate_scene_file(&file) {
            Err(errors) => assert!(
                errors.contains(&SceneFileError::InvalidPose("box-1".into())),
                "expected InvalidPose, got {errors:?}"
            ),
            Ok(()) => panic!("non-finite pose must fail validation"),
        }
    }

    #[test]
    fn missing_location_reference_is_rejected() {
        let mut file = valid_file();
        file.objects[0].location_ref = Some("ghost-tray".into());
        match validate_scene_file(&file) {
            Err(errors) => assert!(
                errors.contains(&SceneFileError::UnknownReference {
                    object: "box-1".into(),
                    reference: "ghost-tray".into(),
                }),
                "expected UnknownReference, got {errors:?}"
            ),
            Ok(()) => panic!("missing location reference must fail validation"),
        }
    }

    #[test]
    fn unsupported_schema_version_is_rejected() {
        let mut file = valid_file();
        file.schema_version = "99".into();
        match validate_scene_file(&file) {
            Err(errors) => assert!(
                errors.contains(&SceneFileError::UnsupportedSchemaVersion("99".into())),
                "expected UnsupportedSchemaVersion, got {errors:?}"
            ),
            Ok(()) => panic!("unknown schema_version must fail validation"),
        }
    }

    #[test]
    fn unsupported_geometry_type_is_rejected() {
        let mut file = valid_file();
        file.objects[0].geometry = Some(GeometryDef {
            r#type: "cone".into(),
            size: vec![0.1, 0.1],
        });
        match validate_scene_file(&file) {
            Err(errors) => assert!(
                errors.contains(&SceneFileError::UnsupportedGeometryType {
                    id: "box-1".into(),
                    r#type: "cone".into(),
                }),
                "expected UnsupportedGeometryType, got {errors:?}"
            ),
            Ok(()) => panic!("unsupported geometry type must fail validation"),
        }
    }

    // ── 1.3: tier (c) robot compat ──────────────────────────────────────

    #[test]
    fn robot_compat_passes_when_name_matches() {
        assert_eq!(
            validate_robot_compat(&valid_file(), "icebot"),
            Ok(())
        );
    }

    #[test]
    fn robot_compat_fails_when_name_mismatches() {
        match validate_robot_compat(&valid_file(), "scara") {
            Err(RobotMismatch { expected, loaded }) => {
                assert_eq!(expected, "icebot");
                assert_eq!(loaded, "scara");
            }
            Ok(()) => panic!("robot name mismatch must fail tier (c)"),
        }
    }
}
