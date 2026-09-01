use crate::id::{LocationId, ObjectId};
use crate::pose::Pose;
use crate::resource::{Location, Object, Tool};
use thalos_engine::core::motion::MotionPose;
use thalos_engine::semantic::knowledge::{GraspPlan, KnowledgeProvider, LoweringError, PlacementPlan};

// ---------------------------------------------------------------------------
// Scene content — the logical world model for a task document
// ---------------------------------------------------------------------------

/// Default SCARA approach/retreat transit height (metres) when the field is
/// omitted from serialized scene data — matches the documented frontend
/// default and the historical 5 cm behaviour.
fn default_approach_height() -> f64 {
    0.05
}

/// The logical scene model: objects, locations, tools, and the home pose.
///
/// SceneContent owns the data. Call `scene.knowledge()` to obtain a
/// `SceneKnowledge` adapter that implements `KnowledgeProvider` by reference.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SceneContent {
    /// Physical objects in the scene.
    pub objects: Vec<Object>,
    /// Logical locations (assembly stations, bins, trays, etc.).
    pub locations: Vec<Location>,
    /// Available tools / end-effectors.
    pub tools: Vec<Tool>,
    /// The robot's home pose (return target for Home operations).
    pub home_pose: Pose,
    /// SCARA approach/retreat transit height (metres) above each object or
    /// location. Serde default for backward compat: scenes from versions that
    /// never emitted this field receive `default_approach_height()` (0.05).
    #[serde(default = "default_approach_height")]
    pub approach_height: f64,
}

impl SceneContent {
    /// Create a `SceneKnowledge` adapter that resolves resources by reference.
    pub fn knowledge(&self) -> SceneKnowledge<'_> {
        SceneKnowledge { scene: self }
    }
}

// ---------------------------------------------------------------------------
// SceneKnowledge — lightweight KnowledgeProvider adapter
// ---------------------------------------------------------------------------

/// A `KnowledgeProvider` that resolves object/location IDs from a `SceneContent`
/// reference, deriving GraspPlan / PlacementPlan from the pose field.
///
/// Approach/retreat frames use `scene.approach_height` metres of Z offset above
/// the object or location pose (serde-default 0.05 m when omitted).
/// This is a lightweight adapter — no HashMap construction, no data copying.
pub struct SceneKnowledge<'a> {
    scene: &'a SceneContent,
}

/// Convert a document `Pose` into a `MotionPose` (adds a "world" frame).
fn pose_to_motion(pose: &Pose) -> MotionPose {
    MotionPose {
        position: pose.position,
        orientation: pose.orientation,
        frame: "world".into(),
    }
}

/// Offset a `MotionPose` by `dz` metres along the Z axis.
fn offset_pose(mp: &MotionPose, dz: f64) -> MotionPose {
    MotionPose {
        position: [mp.position[0], mp.position[1], mp.position[2] + dz],
        orientation: mp.orientation,
        frame: mp.frame.clone(),
    }
}

impl KnowledgeProvider for SceneKnowledge<'_> {
    fn grasp_plan(&self, object: &ObjectId) -> Result<GraspPlan, LoweringError> {
        let obj = self
            .scene
            .objects
            .iter()
            .find(|o| o.id == *object)
            .ok_or_else(|| {
                LoweringError::KnowledgeProvider(format!("unknown object '{}'", object.0))
            })?;
        let grasp_frame = pose_to_motion(&obj.pose);
        let approach_frame = offset_pose(&grasp_frame, self.scene.approach_height);
        let retreat_frame = offset_pose(&grasp_frame, self.scene.approach_height);
        Ok(GraspPlan {
            grasp_frame,
            approach_frame,
            retreat_frame,
            preferred_tool: None,
        })
    }

    fn place_plan(
        &self,
        _object: &ObjectId,
        location: &LocationId,
    ) -> Result<PlacementPlan, LoweringError> {
        let loc = self
            .scene
            .locations
            .iter()
            .find(|l| l.id == *location)
            .ok_or_else(|| {
                LoweringError::KnowledgeProvider(format!("unknown location '{}'", location.0))
            })?;
        let drop_frame = pose_to_motion(&loc.pose);
        let approach_frame = offset_pose(&drop_frame, self.scene.approach_height);
        let retreat_frame = offset_pose(&drop_frame, self.scene.approach_height);
        Ok(PlacementPlan {
            drop_frame,
            approach_frame,
            retreat_frame,
        })
    }

    fn location_pose(&self, location: &LocationId) -> Result<MotionPose, LoweringError> {
        let loc = self
            .scene
            .locations
            .iter()
            .find(|l| l.id == *location)
            .ok_or_else(|| {
                LoweringError::KnowledgeProvider(format!("unknown location '{}'", location.0))
            })?;
        Ok(pose_to_motion(&loc.pose))
    }

    fn home_pose(&self) -> Result<MotionPose, LoweringError> {
        Ok(pose_to_motion(&self.scene.home_pose))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::id::ObjectId;
    use thalos_engine::core::ids::OperationId;
    use thalos_engine::core::motion::MotionProfile;
    use thalos_engine::semantic::lowering::SemanticLowering;
    use thalos_engine::semantic::lowering::context::LoweringContext;
    use thalos_engine::semantic::operation::{HomeOp, SemanticOperation};
    use thalos_engine::semantic::program::SemanticProgram;

    fn default_pose() -> Pose {
        Pose {
            position: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    fn bolt_object() -> Object {
        Object {
            id: ObjectId("bolt".into()),
            name: "Bolt".into(),
            category: None,
            pose: Pose {
                position: [0.5, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
        }
    }

    fn tray_location() -> Location {
        Location {
            id: LocationId("tray".into()),
            name: "Tray".into(),
            description: None,
            pose: Pose {
                position: [0.8, -0.3, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
        }
    }

    fn sample_scene() -> SceneContent {
        SceneContent {
            objects: vec![bolt_object()],
            locations: vec![tray_location()],
            tools: vec![],
            home_pose: Pose {
                position: [0.0, 0.0, 0.5],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
            approach_height: 0.05,
        }
    }

    // ── 3.1: SceneContent::knowledge() returns a SceneKnowledge that can
    //         resolve objects into GraspPlans ─────────────────────────────

    #[test]
    fn scene_knowledge_resolves_objects() {
        let scene = SceneContent {
            objects: vec![Object {
                id: ObjectId("bolt".into()),
                name: "Bolt".into(),
                category: None,
                pose: Pose {
                    position: [0.5, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                },
            }],
            locations: vec![],
            tools: vec![],
            home_pose: default_pose(),
            approach_height: 0.05,
        };
        let knowledge = scene.knowledge();
        let plan = knowledge
            .grasp_plan(&ObjectId("bolt".into()))
            .expect("should resolve bolt");
        assert_eq!(plan.grasp_frame.position[0], 0.5);
    }

    // ── 3.2: All four KnowledgeProvider methods ─────────────────────────

    #[test]
    fn scene_knowledge_grasp_plan_returns_frames() {
        let scene = sample_scene();
        let knowledge = scene.knowledge();

        let plan = knowledge
            .grasp_plan(&ObjectId("bolt".into()))
            .expect("should resolve");
        assert_eq!(plan.grasp_frame.position, [0.5, 0.0, 0.0]);
        assert!(
            plan.approach_frame.position[2] > 0.0,
            "approach should have positive Z offset"
        );
        assert!(
            plan.retreat_frame.position[2] > 0.0,
            "Pick retreat should be above grasp (positive Z offset for SCARA)"
        );
    }

    #[test]
    fn scene_knowledge_place_plan_returns_frames() {
        let scene = sample_scene();
        let knowledge = scene.knowledge();

        let plan = knowledge
            .place_plan(&ObjectId("bolt".into()), &LocationId("tray".into()))
            .expect("should resolve");
        assert_eq!(plan.drop_frame.position[0], 0.8);
        assert_eq!(plan.drop_frame.position[1], -0.3);
    }

    #[test]
    fn scene_knowledge_location_pose_returns_pose() {
        let scene = sample_scene();
        let knowledge = scene.knowledge();

        let pose = knowledge
            .location_pose(&LocationId("tray".into()))
            .expect("should resolve");
        assert_eq!(pose.position[0], 0.8);
        assert_eq!(pose.position[1], -0.3);
    }

    #[test]
    fn scene_knowledge_home_pose_returns_configured_pose() {
        let scene = sample_scene();
        let knowledge = scene.knowledge();

        let pose = knowledge.home_pose().expect("should have home pose");
        assert_eq!(pose.position[2], 0.5);
    }

    // ── 3.3: Error cases ────────────────────────────────────────────────

    #[test]
    fn scene_knowledge_unknown_object_returns_error() {
        let scene = SceneContent {
            objects: vec![],
            locations: vec![],
            tools: vec![],
            home_pose: default_pose(),
            approach_height: 0.05,
        };
        let knowledge = scene.knowledge();

        let result = knowledge.grasp_plan(&ObjectId("ghost".into()));
        assert!(result.is_err());
        match result {
            Err(LoweringError::KnowledgeProvider(msg)) => {
                assert!(msg.contains("ghost"), "error should mention object id");
            }
            _ => panic!("Expected KnowledgeProvider error"),
        }
    }

    #[test]
    fn scene_knowledge_unknown_location_in_place_plan_returns_error() {
        let scene = SceneContent {
            objects: vec![bolt_object()],
            locations: vec![],
            tools: vec![],
            home_pose: default_pose(),
            approach_height: 0.05,
        };
        let knowledge = scene.knowledge();

        let result = knowledge.place_plan(&ObjectId("bolt".into()), &LocationId("unknown".into()));
        assert!(result.is_err());
    }

    #[test]
    fn scene_knowledge_unknown_location_in_location_pose_returns_error() {
        let scene = SceneContent {
            objects: vec![],
            locations: vec![],
            tools: vec![],
            home_pose: default_pose(),
            approach_height: 0.05,
        };
        let knowledge = scene.knowledge();

        let result = knowledge.location_pose(&LocationId("ghost".into()));
        assert!(result.is_err());
    }

    // ── 3.4: ProgramDocument serde round-trip ──────────────────────────────
    //
    // ProgramDocument is defined in program_document.rs, but we test the
    // complete serde round-trip here alongside the scene tests.

    #[test]
    fn program_document_serde_round_trip() {
        use crate::id::{ProgramDocumentId, TaskDocumentId};
        use crate::program_document::{Metadata as DocumentMetadata, ProgramDocument};

        let doc = ProgramDocument {
            id: TaskDocumentId("test-1".into()),
            metadata: DocumentMetadata {
                name: "test".into(),
                version: 1,
                created_at: "2026-07-29T00:00:00Z".into(),
                modified_at: "2026-07-29T00:00:00Z".into(),
            },
            scene: SceneContent {
                objects: vec![bolt_object()],
                locations: vec![tray_location()],
                tools: vec![],
                home_pose: default_pose(),
                approach_height: 0.05,
            },
            program: SemanticProgram::new(vec![SemanticOperation::Home(HomeOp {
                origin: OperationId("op-1".into()),
            })]),
        };
        let json = serde_json::to_string(&doc).unwrap();
        let back: ProgramDocument = serde_json::from_str(&json).unwrap();
        assert_eq!(doc, back);
    }

    // ── 3.5: SceneKnowledge feeds lowering directly ─────────────────────

    #[test]
    fn scene_knowledge_feeds_lowering_directly() {
        let scene = sample_scene();
        let provider = scene.knowledge();

        let ctx = LoweringContext::new(&provider)
            .with_default_profile(MotionProfile {
                max_velocity: 1.0,
                max_acceleration: 0.5,
                max_jerk: None,
            });

        let program = SemanticProgram::new(vec![SemanticOperation::Home(HomeOp {
            origin: OperationId("op-1".into()),
        })]);
        let ir = thalos_engine::semantic::ir::SemanticIr::from(&program);

        let result = SemanticLowering::lower(&ir, &ctx);
        assert!(
            result.is_ok(),
            "lowering should succeed with SceneKnowledge"
        );
    }

    // ── 3.6: SceneContent.approach_height parameterizes approach/retreat
    //       frames and serde-round-trips with a default ────────────────

    fn scene_with_approach_height(height: f64) -> SceneContent {
        SceneContent {
            approach_height: height,
            ..sample_scene()
        }
    }

    #[test]
    fn approach_height_sets_approach_and_retreat_z() {
        let low = scene_with_approach_height(0.02);
        let high = scene_with_approach_height(0.05);
        let low_plan = low
            .knowledge()
            .grasp_plan(&ObjectId("bolt".into()))
            .expect("should resolve");
        let high_plan = high
            .knowledge()
            .grasp_plan(&ObjectId("bolt".into()))
            .expect("should resolve");

        let grasp_z = low_plan.grasp_frame.position[2];
        assert!(
            (low_plan.approach_frame.position[2] - (grasp_z + 0.02)).abs() < 1e-9,
            "approach Z should be grasp Z + 0.02"
        );
        assert!(
            (low_plan.retreat_frame.position[2] - (grasp_z + 0.02)).abs() < 1e-9,
            "retreat Z should be grasp Z + 0.02"
        );
        assert!(
            (high_plan.approach_frame.position[2] - (grasp_z + 0.05)).abs() < 1e-9,
            "approach Z should be grasp Z + 0.05"
        );
        assert!(
            (high_plan.approach_frame.position[2] - low_plan.approach_frame.position[2] - 0.03)
                .abs()
                < 1e-9,
            "the two approach frames should differ by exactly 0.03"
        );
    }

    #[test]
    fn grasp_and_place_consume_same_approach_height() {
        let scene = scene_with_approach_height(0.05);
        let knowledge = scene.knowledge();
        let grasp = knowledge
            .grasp_plan(&ObjectId("bolt".into()))
            .expect("should resolve");
        let place = knowledge
            .place_plan(&ObjectId("bolt".into()), &LocationId("tray".into()))
            .expect("should resolve");

        let grasp_offset = grasp.approach_frame.position[2] - grasp.grasp_frame.position[2];
        let place_offset = place.approach_frame.position[2] - place.drop_frame.position[2];
        assert!((grasp_offset - 0.05).abs() < 1e-9);
        assert!((place_offset - 0.05).abs() < 1e-9);
        assert!(
            (grasp_offset - place_offset).abs() < 1e-9,
            "grasp and place must consume the same approach_height"
        );
    }

    #[test]
    fn scene_content_approach_height_serde_default() {
        let with_default: SceneContent = serde_json::from_str(
            r#"{"objects":[],"locations":[],"tools":[],"home_pose":{"position":[0.0,0.0,0.5],"orientation":[0.0,0.0,0.0,1.0]}}"#,
        )
        .expect("scene without approach_height should deserialize");
        assert_eq!(with_default.approach_height, 0.05);

        let with_value: SceneContent = serde_json::from_str(
            r#"{"objects":[],"locations":[],"tools":[],"home_pose":{"position":[0.0,0.0,0.5],"orientation":[0.0,0.0,0.0,1.0]},"approach_height":0.1}"#,
        )
        .expect("scene with explicit approach_height should deserialize");
        assert_eq!(with_value.approach_height, 0.1);
    }
}
