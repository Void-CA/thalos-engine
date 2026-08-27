use std::fmt;

use thalos_core::motion::MotionProfile;

use crate::knowledge::KnowledgeProvider;
use crate::resource::ToolId;

/// The context for lowering a `SemanticProgram` into a `ExecutionProgram`.
///
/// Wraps the read-only dependencies: a `KnowledgeProvider` for resource
/// resolution, a default tool for operations that omit tool selection, and
/// a default `MotionProfile` for emitted motion instructions.
///
/// Context is immutable during lowering — all fields are public for
/// construction but never mutated once built.
pub struct LoweringContext<'a> {
    /// The knowledge provider for resolving semantic resource IDs into
    /// geometric frames and plans.
    pub provider: &'a dyn KnowledgeProvider,
    /// The default tool to use when an operation specifies `tool: None`.
    pub default_tool: Option<ToolId>,
    /// The default motion profile for emitted JOINT-space instructions
    /// (MoveJ approach/Home/MoveTo). Units: rad/s, rad/s² — MoveJ plans in
    /// radians, so this is NOT the cartesian default.
    pub default_profile: MotionProfile,
    /// The default motion profile for emitted CARTESIAN instructions (MoveL
    /// grasp/drop/retract). Units: m/s, m/s². `None` → falls back to
    /// `default_profile` (existing callers that only set `default_profile`
    /// keep working).
    pub default_cartesian_profile: Option<MotionProfile>,
}

impl LoweringContext<'_> {
    /// The profile for cartesian (MoveL) instructions: the explicit cartesian
    /// profile when configured, otherwise the joint `default_profile`
    /// (backward-compatible fallback for callers that only set
    /// `default_profile`).
    pub fn cartesian_profile(&self) -> MotionProfile {
        self.default_cartesian_profile
            .clone()
            .unwrap_or_else(|| self.default_profile.clone())
    }
}

impl Clone for LoweringContext<'_> {
    fn clone(&self) -> Self {
        LoweringContext {
            provider: self.provider,
            default_tool: self.default_tool.clone(),
            default_profile: self.default_profile.clone(),
            default_cartesian_profile: self.default_cartesian_profile.clone(),
        }
    }
}

impl fmt::Debug for LoweringContext<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LoweringContext")
            .field("default_tool", &self.default_tool)
            .field("default_profile", &self.default_profile)
            .field("default_cartesian_profile", &self.default_cartesian_profile)
            .field("provider", &"<KnowledgeProvider>")
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::motion::MotionPose;

    use crate::knowledge::MockKnowledgeProvider;

    fn sample_profile() -> MotionProfile {
        MotionProfile {
            max_velocity: 500.0,
            max_acceleration: 1000.0,
            max_jerk: None,
        }
    }

    fn sample_provider() -> MockKnowledgeProvider {
        MockKnowledgeProvider::new().with_home_pose(Ok(MotionPose {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        }))
    }

    // ── Construction ────────────────────────────────────────────────────

    #[test]
    fn lowering_context_wraps_provider_ref() {
        let provider = sample_provider();
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };
        // Confirm we can call provider methods through the context
        let home = ctx.provider.home_pose();
        assert!(home.is_ok());
    }

    #[test]
    fn lowering_context_with_default_tool() {
        let provider = sample_provider();
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: Some(ToolId("gripper-1".to_string())),
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };
        assert_eq!(ctx.default_tool, Some(ToolId("gripper-1".to_string())));
    }

    #[test]
    fn lowering_context_without_default_tool() {
        let provider = sample_provider();
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };
        assert!(ctx.default_tool.is_none());
    }

    #[test]
    fn lowering_context_has_default_profile() {
        let provider = sample_provider();
        let profile = sample_profile();
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: None,
            default_profile: profile.clone(),
            default_cartesian_profile: None,
        };
        assert_eq!(ctx.default_profile, profile);
    }

    // ── Cartesian profile resolution (follow-up fix) ──────────────────────

    /// Existing callers that only set `default_profile` (the JOINT profile)
    /// must keep working: cartesian instructions fall back to the joint
    /// profile when `default_cartesian_profile` is `None`.
    #[test]
    fn cartesian_profile_falls_back_to_joint_profile_when_unspecified() {
        let provider = sample_provider();
        let profile = sample_profile();
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: None,
            default_profile: profile.clone(),
            default_cartesian_profile: None,
        };
        assert_eq!(ctx.cartesian_profile(), profile);
    }

    /// When a cartesian profile IS configured, cartesian instructions get it
    /// — distinct from the joint profile.
    #[test]
    fn cartesian_profile_prefers_the_explicit_profile() {
        let provider = sample_provider();
        let joint = sample_profile();
        let cartesian = MotionProfile {
            max_velocity: 0.1,
            max_acceleration: 0.5,
            max_jerk: None,
        };
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: None,
            default_profile: joint.clone(),
            default_cartesian_profile: Some(cartesian.clone()),
        };
        assert_eq!(ctx.default_profile, joint);
        assert_eq!(ctx.cartesian_profile(), cartesian);
    }

    // ── Immutability ────────────────────────────────────────────────────

    #[test]
    fn lowering_context_cannot_mutate_provider() {
        let provider = sample_provider();
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };
        // All methods take &self — confirmed by trait signature.
        // The provider reference is immutable (&dyn, not &mut dyn).
        let _provider_ref: &dyn KnowledgeProvider = ctx.provider;
    }

    #[test]
    fn lowering_context_is_read_only() {
        let provider = sample_provider();
        let ctx = LoweringContext {
            provider: &provider,
            default_tool: None,
            default_profile: sample_profile(),
            default_cartesian_profile: None,
        };
        // Prove we can read from the context multiple times
        let _ = ctx.provider;
        let _ = ctx.default_tool;
        let _ = ctx.default_profile;
    }
}
