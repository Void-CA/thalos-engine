use serde::{Deserialize, Serialize};
use crate::ids::{RobotId, SkillId};
use crate::robot::definition::RobotDefinition;

/// Source location or mechanism for resolving a skill implementation within a `RobotProfile`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SkillBindingSource {
    /// Built-in driver or native hardware skill identifier (e.g. "driver.digital_gripper").
    Native { native_id: String },
    /// External planning policy reference.
    Planner { policy: String },
}

/// Binding connecting a declared `SkillId` capability to its persistence/implementation source.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SkillBinding {
    pub skill: SkillId,
    pub source: SkillBindingSource,
}

impl SkillBinding {
    pub fn new(skill: SkillId, source: SkillBindingSource) -> Self {
        Self { skill, source }
    }
}

/// Persisted configuration profile representing a specific physical or simulated robot instance.
///
/// Serves as the configuration aggregate that materializes runtime compiler inputs:
/// `RobotProfile` → (`RobotDefinition`, `SkillRegistry`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotProfile {
    pub id: RobotId,
    pub definition: RobotDefinition,
    pub skill_bindings: Vec<SkillBinding>,
}

impl RobotProfile {
    pub fn new(
        id: RobotId,
        definition: RobotDefinition,
        skill_bindings: Vec<SkillBinding>,
    ) -> Self {
        Self {
            id,
            definition,
            skill_bindings,
        }
    }

    /// Returns `true` if the profile contains a skill binding for the given `SkillId`.
    pub fn has_skill_binding(&self, skill: &SkillId) -> bool {
        self.skill_bindings.iter().any(|b| &b.skill == skill)
    }
}
