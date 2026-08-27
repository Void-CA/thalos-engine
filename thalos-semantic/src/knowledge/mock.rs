use std::collections::HashMap;

use thalos_core::motion::MotionPose;

use crate::resource::{LocationId, ObjectId};

use super::grasp::GraspPlan;
use super::place::PlacementPlan;
use super::{KnowledgeProvider, LoweringError};

/// A configurable `KnowledgeProvider` for testing.
///
/// Configured at construction time with per-key return values for each of the
/// four provider methods. Supports both `Ok` and `Err` returns per key.
///
/// # Example
///
/// ```rust
/// use thalos_semantic::knowledge::mock::MockKnowledgeProvider;
/// use thalos_semantic::knowledge::LoweringError;
/// use thalos_semantic::resource::ObjectId;
///
/// let provider = MockKnowledgeProvider::new()
///     .with_grasp_error(ObjectId("unknown".into()), LoweringError::KnowledgeProvider("not found".into()));
/// ```
#[derive(Debug, Clone)]
pub struct MockKnowledgeProvider {
    grasp_plans: HashMap<ObjectId, Result<GraspPlan, LoweringError>>,
    place_plans: HashMap<(ObjectId, LocationId), Result<PlacementPlan, LoweringError>>,
    location_poses: HashMap<LocationId, Result<MotionPose, LoweringError>>,
    home_pose_result: Result<MotionPose, LoweringError>,
}

impl MockKnowledgeProvider {
    /// Create a new `MockKnowledgeProvider` with no configured values.
    ///
    /// All methods return `Err(LoweringError::KnowledgeProvider("not configured"))`
    /// until specific keys are configured via builder methods.
    pub fn new() -> Self {
        Self {
            grasp_plans: HashMap::new(),
            place_plans: HashMap::new(),
            location_poses: HashMap::new(),
            home_pose_result: Err(LoweringError::KnowledgeProvider(
                "not configured".to_string(),
            )),
        }
    }

    /// Configure the return value for a specific object's `grasp_plan`.
    pub fn with_grasp_plan(
        mut self,
        object: ObjectId,
        result: Result<GraspPlan, LoweringError>,
    ) -> Self {
        self.grasp_plans.insert(object, result);
        self
    }

    /// Configure an `Ok` return for a specific object's `grasp_plan` (convenience).
    pub fn with_grasp_ok(self, object: ObjectId, plan: GraspPlan) -> Self {
        self.with_grasp_plan(object, Ok(plan))
    }

    /// Configure an `Err` return for a specific object's `grasp_plan` (convenience).
    pub fn with_grasp_error(self, object: ObjectId, error: LoweringError) -> Self {
        self.with_grasp_plan(object, Err(error))
    }

    /// Configure the return value for a specific (object, location) pair's `place_plan`.
    pub fn with_place_plan(
        mut self,
        object: ObjectId,
        location: LocationId,
        result: Result<PlacementPlan, LoweringError>,
    ) -> Self {
        self.place_plans.insert((object, location), result);
        self
    }

    /// Configure an `Ok` return for a specific (object, location) pair's `place_plan`.
    pub fn with_place_ok(self, object: ObjectId, location: LocationId, plan: PlacementPlan) -> Self {
        self.with_place_plan(object, location, Ok(plan))
    }

    /// Configure an `Err` return for a specific (object, location) pair's `place_plan`.
    pub fn with_place_error(
        self,
        object: ObjectId,
        location: LocationId,
        error: LoweringError,
    ) -> Self {
        self.with_place_plan(object, location, Err(error))
    }

    /// Configure the return value for a specific location's `location_pose`.
    pub fn with_location_pose(
        mut self,
        location: LocationId,
        result: Result<MotionPose, LoweringError>,
    ) -> Self {
        self.location_poses.insert(location, result);
        self
    }

    /// Configure an `Ok` return for a specific location's `location_pose`.
    pub fn with_location_ok(self, location: LocationId, pose: MotionPose) -> Self {
        self.with_location_pose(location, Ok(pose))
    }

    /// Configure an `Err` return for a specific location's `location_pose`.
    pub fn with_location_error(self, location: LocationId, error: LoweringError) -> Self {
        self.with_location_pose(location, Err(error))
    }

    /// Configure the return value for `home_pose`.
    pub fn with_home_pose(mut self, result: Result<MotionPose, LoweringError>) -> Self {
        self.home_pose_result = result;
        self
    }
}

impl Default for MockKnowledgeProvider {
    fn default() -> Self {
        Self::new()
    }
}

impl KnowledgeProvider for MockKnowledgeProvider {
    fn grasp_plan(&self, object: &ObjectId) -> Result<GraspPlan, LoweringError> {
        self.grasp_plans
            .get(object)
            .cloned()
            .unwrap_or_else(|| Err(LoweringError::KnowledgeProvider("not configured".to_string())))
    }

    fn place_plan(
        &self,
        object: &ObjectId,
        location: &LocationId,
    ) -> Result<PlacementPlan, LoweringError> {
        let key = (object.clone(), location.clone());
        self.place_plans
            .get(&key)
            .cloned()
            .unwrap_or_else(|| Err(LoweringError::KnowledgeProvider("not configured".to_string())))
    }

    fn location_pose(&self, location: &LocationId) -> Result<MotionPose, LoweringError> {
        self.location_poses
            .get(location)
            .cloned()
            .unwrap_or_else(|| Err(LoweringError::KnowledgeProvider("not configured".to_string())))
    }

    fn home_pose(&self) -> Result<MotionPose, LoweringError> {
        self.home_pose_result.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_pose(x: f64, y: f64, z: f64) -> MotionPose {
        MotionPose {
            position: [x, y, z],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        }
    }

    fn sample_grasp_plan() -> GraspPlan {
        GraspPlan {
            grasp_frame: sample_pose(1.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 1.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 1.0),
            preferred_tool: None,
        }
    }

    fn sample_placement_plan() -> PlacementPlan {
        PlacementPlan {
            drop_frame: sample_pose(2.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 2.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 2.0),
        }
    }

    // ── Configured Ok returns per key ────────────────────────────────────

    #[test]
    fn mock_returns_configured_grasp_plan() {
        let object = ObjectId("bolt-1".to_string());
        let plan = sample_grasp_plan();
        let provider = MockKnowledgeProvider::new().with_grasp_ok(object.clone(), plan.clone());

        let result = provider.grasp_plan(&object);
        assert_eq!(result, Ok(plan));
    }

    #[test]
    fn mock_returns_configured_place_plan() {
        let object = ObjectId("bolt-1".to_string());
        let location = LocationId("tray-1".to_string());
        let plan = sample_placement_plan();
        let provider =
            MockKnowledgeProvider::new().with_place_ok(object.clone(), location.clone(), plan.clone());

        let result = provider.place_plan(&object, &location);
        assert_eq!(result, Ok(plan));
    }

    #[test]
    fn mock_returns_configured_location_pose() {
        let location = LocationId("shelf-a".to_string());
        let pose = sample_pose(5.0, 0.0, 0.0);
        let provider =
            MockKnowledgeProvider::new().with_location_ok(location.clone(), pose.clone());

        let result = provider.location_pose(&location);
        assert_eq!(result, Ok(pose));
    }

    #[test]
    fn mock_returns_configured_home_pose() {
        let home = sample_pose(0.0, 0.0, 0.5);
        let provider = MockKnowledgeProvider::new().with_home_pose(Ok(home.clone()));

        let result = provider.home_pose();
        assert_eq!(result, Ok(home));
    }

    // ── Configured Err returns ───────────────────────────────────────────

    #[test]
    fn mock_returns_configured_grasp_error() {
        let object = ObjectId("unknown".to_string());
        let error = LoweringError::KnowledgeProvider("object not found".to_string());
        let provider = MockKnowledgeProvider::new().with_grasp_error(object.clone(), error.clone());

        let result = provider.grasp_plan(&object);
        assert_eq!(result, Err(error));
    }

    #[test]
    fn mock_returns_configured_place_error() {
        let object = ObjectId("unknown".to_string());
        let location = LocationId("unknown-loc".to_string());
        let error = LoweringError::KnowledgeProvider("placement not found".to_string());
        let provider = MockKnowledgeProvider::new()
            .with_place_error(object.clone(), location.clone(), error.clone());

        let result = provider.place_plan(&object, &location);
        assert_eq!(result, Err(error));
    }

    #[test]
    fn mock_returns_configured_location_error() {
        let location = LocationId("unknown".to_string());
        let error = LoweringError::KnowledgeProvider("location unknown".to_string());
        let provider = MockKnowledgeProvider::new()
            .with_location_error(location.clone(), error.clone());

        let result = provider.location_pose(&location);
        assert_eq!(result, Err(error));
    }

    #[test]
    fn mock_returns_configured_home_error() {
        let provider = MockKnowledgeProvider::new()
            .with_home_pose(Err(LoweringError::MissingHomePose));

        let result = provider.home_pose();
        assert_eq!(result, Err(LoweringError::MissingHomePose));
    }

    // ── Per-key returns (different keys return different values) ─────────

    #[test]
    fn mock_returns_different_values_per_object_key() {
        let known = ObjectId("known".to_string());
        let unknown = ObjectId("unknown".to_string());
        let plan = sample_grasp_plan();

        let provider = MockKnowledgeProvider::new()
            .with_grasp_ok(known.clone(), plan.clone());

        assert_eq!(provider.grasp_plan(&known), Ok(plan));
        assert!(
            provider.grasp_plan(&unknown).is_err(),
            "Unconfigured key should return Err"
        );
    }

    #[test]
    fn mock_returns_different_values_per_location_key() {
        let known = LocationId("known".to_string());
        let unknown = LocationId("unknown".to_string());
        let pose = sample_pose(1.0, 0.0, 0.0);

        let provider = MockKnowledgeProvider::new()
            .with_location_ok(known.clone(), pose.clone());

        assert_eq!(provider.location_pose(&known), Ok(pose));
        assert!(
            provider.location_pose(&unknown).is_err(),
            "Unconfigured key should return Err"
        );
    }

    // ── Determinism ──────────────────────────────────────────────────────

    #[test]
    fn mock_returns_same_value_on_repeated_calls() {
        let object = ObjectId("bolt-1".to_string());
        let plan = sample_grasp_plan();
        let provider = MockKnowledgeProvider::new().with_grasp_ok(object.clone(), plan.clone());

        let first = provider.grasp_plan(&object);
        let second = provider.grasp_plan(&object);
        assert_eq!(first, second);
        assert_eq!(first, Ok(plan));
    }

    #[test]
    fn mock_location_pose_deterministic() {
        let location = LocationId("base".to_string());
        let pose = sample_pose(0.0, 0.0, 0.0);
        let provider =
            MockKnowledgeProvider::new().with_location_ok(location.clone(), pose.clone());

        assert_eq!(provider.location_pose(&location), Ok(pose.clone()));
        assert_eq!(provider.location_pose(&location), Ok(pose));
    }

    // ── Default / unconfigured ──────────────────────────────────────────

    #[test]
    fn unconfigured_mock_returns_error() {
        let provider = MockKnowledgeProvider::new();
        let object = ObjectId("anything".to_string());

        let result = provider.grasp_plan(&object);
        assert!(result.is_err());
        match result {
            Err(LoweringError::KnowledgeProvider(msg)) => {
                assert_eq!(msg, "not configured");
            }
            _ => panic!("Expected KnowledgeProvider error"),
        }
    }
}
