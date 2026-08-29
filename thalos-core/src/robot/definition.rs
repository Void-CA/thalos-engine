use serde::{Deserialize, Serialize};
use crate::ids::{RobotId, SkillId, ToolId};
use crate::robot::capability::RobotCapability;
use crate::robot::serial_chain::SerialChain;
use crate::skill::SkillCapability;

/// Complete structural, kinematic, and functional definition of a Robot in Thalos.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotDefinition {
    pub id: RobotId,
    pub name: String,
    #[serde(skip)]
    pub kinematic_chain: Option<SerialChain>,
    pub tools: Vec<ToolId>,
    pub observation_capability: RobotCapability,
    pub skill_capabilities: Vec<SkillCapability>,
}

impl PartialEq for RobotDefinition {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
            && self.name == other.name
            && self.tools == other.tools
            && self.observation_capability == other.observation_capability
            && self.skill_capabilities == other.skill_capabilities
    }
}

impl RobotDefinition {
    pub fn new(
        id: RobotId,
        name: impl Into<String>,
        kinematic_chain: Option<SerialChain>,
        tools: Vec<ToolId>,
        observation_capability: RobotCapability,
        skill_capabilities: Vec<SkillCapability>,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            kinematic_chain,
            tools,
            observation_capability,
            skill_capabilities,
        }
    }

    /// Check if this robot explicitly declares capability for a given skill.
    pub fn supports_skill(&self, skill_id: &SkillId) -> bool {
        self.skill_capabilities.iter().any(|c| &c.skill == skill_id)
    }
}
