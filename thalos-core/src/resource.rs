use serde::{Deserialize, Serialize};
use crate::ids::ResourceId;
use crate::capability::CapabilityRequirement;

/// ResourceKind (ADR-014)
/// Categorizes resources in the inventory registry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResourceKind {
    Robot,
    Device,
    Channel,
    Simulator,
}

/// ResourceRef (ADR-014)
/// Light reference linking a Station to a Resource in the global registry.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ResourceRef {
    pub id: ResourceId,
    pub kind: ResourceKind,
}

impl ResourceRef {
    pub fn new(id: impl Into<String>, kind: ResourceKind) -> Self {
        Self {
            id: ResourceId(id.into()),
            kind,
        }
    }
}

/// Resource (ADR-014)
/// Canonical inventory resource entity holding identity and provided capabilities.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Resource {
    pub id: ResourceId,
    pub kind: ResourceKind,
    pub name: String,
    pub capabilities: Vec<CapabilityRequirement>,
}

impl Resource {
    pub fn new(id: impl Into<String>, kind: ResourceKind, name: impl Into<String>, capabilities: Vec<CapabilityRequirement>) -> Self {
        Self {
            id: ResourceId(id.into()),
            kind,
            name: name.into(),
            capabilities,
        }
    }

    pub fn to_ref(&self) -> ResourceRef {
        ResourceRef {
            id: self.id.clone(),
            kind: self.kind,
        }
    }
}
