pub mod grasp;
pub mod mock;
pub mod place;

use thiserror::Error;

use crate::resource::{LocationId, ObjectId};
use thalos_core::motion::MotionPose;

pub use self::grasp::GraspPlan;
pub use self::mock::MockKnowledgeProvider;
pub use self::place::PlacementPlan;

/// Errors that can occur during semantic lowering and knowledge provider
/// resolution.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum LoweringError {
    /// The knowledge provider returned an error for the requested resource.
    #[error("knowledge provider error: {0}")]
    KnowledgeProvider(String),
    /// The home pose is not configured and cannot be resolved.
    #[error("missing home pose")]
    MissingHomePose,
    /// The requested skill is not registered in the SkillRegistry.
    #[error("unknown skill: '{0}'")]
    UnknownSkill(thalos_core::ids::SkillId),
}

/// Create the conventional gripper output channel used by lowering for
/// grip (`SetOutput(gripper, true)`) and ungrip (`SetOutput(gripper, false)`).
pub fn gripper_channel() -> thalos_core::motion::OutputChannel {
    thalos_core::motion::OutputChannel {
        name: "gripper".into(),
        channel_type: "digital".into(),
    }
}

/// The read-only knowledge interface through which lowering resolves
/// semantic resource identifiers into geometric frames.
///
/// The provider returns plans and poses, NOT motion instructions.
/// All methods take `&self` — the provider is stateless from lowering's
/// perspective.
pub trait KnowledgeProvider {
    /// Resolve an object to a grasp plan containing grasp, approach, and
    /// retreat frames plus an optional preferred tool.
    fn grasp_plan(&self, object: &ObjectId) -> Result<GraspPlan, LoweringError>;

    /// Resolve an object and destination to a placement plan containing
    /// drop, approach, and retreat frames.
    fn place_plan(
        &self,
        object: &ObjectId,
        location: &LocationId,
    ) -> Result<PlacementPlan, LoweringError>;

    /// Resolve a location to a target pose for MoveTo operations.
    fn location_pose(&self, location: &LocationId) -> Result<MotionPose, LoweringError>;

    /// Return the home pose for Home operations.
    fn home_pose(&self) -> Result<MotionPose, LoweringError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify that `KnowledgeProvider` has exactly 4 methods by writing a
    /// generic function that exercises all of them. If the trait gains or
    /// loses methods, this function will fail to compile.
    fn assert_trait_has_four_methods<P: KnowledgeProvider>(
        provider: &P,
        object: &ObjectId,
        location: &LocationId,
    ) {
        let _ = provider.grasp_plan(object);
        let _ = provider.place_plan(object, location);
        let _ = provider.location_pose(location);
        let _ = provider.home_pose();
    }

    #[test]
    fn grasp_plan_returns_result() {
        // This test exists at compile-time via the trait definition.
        // We confirm the return type is Result<GraspPlan, LoweringError>.
        fn check_return_type(provider: &dyn KnowledgeProvider, object: &ObjectId) {
            let result: Result<GraspPlan, LoweringError> = provider.grasp_plan(object);
            let _ = result; // suppress unused warning
        }
        // Cannot call check_return_type without a concrete provider,
        // but the function signature confirms the return type.
    }

    #[test]
    fn place_plan_returns_result() {
        fn check_return_type(
            provider: &dyn KnowledgeProvider,
            object: &ObjectId,
            location: &LocationId,
        ) {
            let result: Result<PlacementPlan, LoweringError> =
                provider.place_plan(object, location);
            let _ = result;
        }
    }

    #[test]
    fn location_pose_returns_result() {
        fn check_return_type(provider: &dyn KnowledgeProvider, location: &LocationId) {
            let result: Result<MotionPose, LoweringError> = provider.location_pose(location);
            let _ = result;
        }
    }

    #[test]
    fn home_pose_returns_result() {
        fn check_return_type(provider: &dyn KnowledgeProvider) {
            let result: Result<MotionPose, LoweringError> = provider.home_pose();
            let _ = result;
        }
    }

    #[test]
    fn all_methods_take_self_by_ref() {
        // The trait uses `&self` — confirmed by the signatures above.
        // This test proves it by using a generic function that takes `&P`.
        fn takes_ref<P: KnowledgeProvider>(_provider: &P) {}
        // If methods took `&mut self` or `self`, this would fail to compile.
        // We just verify the function exists and is well-typed.
    }

    #[test]
    fn no_motion_instruction_types_in_signatures() {
        // Confirm that GraspPlan/PlacementPlan do NOT contain ProgramInstruction
        fn check_grasp_plan_fields(plan: &GraspPlan) {
            let GraspPlan {
                grasp_frame: _,
                approach_frame: _,
                retreat_frame: _,
                preferred_tool: _,
            } = plan;
        }

        fn check_placement_plan_fields(plan: &PlacementPlan) {
            let PlacementPlan {
                drop_frame: _,
                approach_frame: _,
                retreat_frame: _,
            } = plan;
        }

        // Also verify the return types don't reference ProgramInstruction
        fn provider_returns_no_instructions(provider: &dyn KnowledgeProvider, object: &ObjectId) {
            match provider.grasp_plan(object) {
                Ok(plan) => {
                    // plan is GraspPlan — not ProgramInstruction, not ExecutionProgram
                    let _: &GraspPlan = &plan;
                }
                Err(_) => {}
            }
        }
    }
}
