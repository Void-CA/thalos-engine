use std::collections::HashMap;
use thalos_engine::prelude::*;

/// ResourceRegistry (ADR-014)
/// In-memory inventory repository holding all registered system resources.
#[derive(Debug, Default, Clone)]
pub struct ResourceRegistry {
    resources: HashMap<ResourceId, Resource>,
}

impl ResourceRegistry {
    pub fn new() -> Self {
        Self {
            resources: HashMap::new(),
        }
    }

    /// Register or update a resource in the global registry.
    pub fn register(&mut self, resource: Resource) {
        self.resources.insert(resource.id.clone(), resource);
    }

    /// Lookup a resource by ID.
    pub fn get(&self, id: &ResourceId) -> Option<&Resource> {
        self.resources.get(id)
    }

    /// List all registered resources.
    pub fn list(&self) -> Vec<&Resource> {
        let mut list: Vec<&Resource> = self.resources.values().collect();
        list.sort_by(|a, b| a.id.as_str().cmp(b.id.as_str()));
        list
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_and_get() {
        let mut registry = ResourceRegistry::new();
        let robot = Resource::new(
            "scara-01",
            ResourceKind::Robot,
            "SCARA Robot 01",
            vec![CapabilityRequirement::CartesianMotion],
        );

        registry.register(robot.clone());
        let fetched = registry.get(&ResourceId("scara-01".into()));

        assert!(fetched.is_some());
        assert_eq!(fetched.unwrap().name, "SCARA Robot 01");
    }
}
