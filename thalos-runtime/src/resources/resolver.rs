use serde::{Deserialize, Serialize};
use thiserror::Error;
use thalos_engine::prelude::*;
use super::registry::ResourceRegistry;
use super::reservation::ResourceReservationManager;

/// ResourceResolutionError (ADR-014)
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ResourceResolutionError {
    #[error("Unresolved mandatory capability requirement: {0:?}")]
    UnresolvedMandatoryRequirement(CapabilityRequirement),

    #[error("Resource referenced by station not found in registry: {0}")]
    ResourceNotFound(String),
}

/// ResourceMatch (ADR-014)
/// Maps a requirement to the deterministically selected resource reference.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResourceMatch {
    pub requirement: ResourceRequirement,
    pub matched_resource: ResourceRef,
}

/// ResolvedResources (ADR-014)
/// Outcome of capability resolution containing matches and unfulfilled optional requirements.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResolvedResources {
    pub matches: Vec<ResourceMatch>,
    pub unresolved_optional: Vec<ResourceRequirement>,
}

/// ResourceResolver (ADR-014)
/// Capability-based, deterministic resource resolution engine.
pub struct ResourceResolver;

impl ResourceResolver {
    /// Resolve program resource requirements considering active reservations.
    pub fn resolve_available(
        station: &Station,
        registry: &ResourceRegistry,
        reservation_manager: &ResourceReservationManager,
        requirements: &[ResourceRequirement],
    ) -> Result<ResolvedResources, ResourceResolutionError> {
        let mut available_registry = ResourceRegistry::new();
        for res in registry.list() {
            if !reservation_manager.is_reserved(&res.id) {
                available_registry.register(res.clone());
            }
        }
        Self::resolve(station, &available_registry, requirements)
    }

    /// Resolve program resource requirements against station context and global registry.
    pub fn resolve(
        station: &Station,
        registry: &ResourceRegistry,
        requirements: &[ResourceRequirement],
    ) -> Result<ResolvedResources, ResourceResolutionError> {
        let mut matches = Vec::new();
        let mut unresolved_optional = Vec::new();

        // Convert station resources into a set for quick station-binding priority lookup
        let station_ref_set: Vec<&ResourceRef> = station.resources.iter().collect();

        for req in requirements {
            let mut candidates: Vec<(&Resource, bool)> = Vec::new();

            for resource in registry.list() {
                if Self::satisfies_capability(resource, &req.capability) {
                    let is_station_bound = station_ref_set.contains(&&resource.to_ref());
                    candidates.push((resource, is_station_bound));
                }
            }

            if candidates.is_empty() {
                if req.is_mandatory {
                    return Err(ResourceResolutionError::UnresolvedMandatoryRequirement(
                        req.capability.clone(),
                    ));
                } else {
                    unresolved_optional.push(req.clone());
                    continue;
                }
            }

            // Deterministic candidate selection:
            // 1. Station-bound candidates first (Priority 1)
            // 2. Lexicographical ResourceId ordering (Priority 2)
            candidates.sort_by(|(res_a, is_bound_a), (res_b, is_bound_b)| {
                match is_bound_b.cmp(is_bound_a) {
                    std::cmp::Ordering::Equal => res_a.id.as_str().cmp(res_b.id.as_str()),
                    other => other,
                }
            });

            let (best_match, _) = candidates[0];
            matches.push(ResourceMatch {
                requirement: req.clone(),
                matched_resource: best_match.to_ref(),
            });
        }

        Ok(ResolvedResources {
            matches,
            unresolved_optional,
        })
    }

    /// Helper to evaluate if a Resource satisfies a required CapabilityRequirement.
    fn satisfies_capability(resource: &Resource, req_cap: &CapabilityRequirement) -> bool {
        resource.capabilities.iter().any(|provided_cap| match (provided_cap, req_cap) {
            (CapabilityRequirement::CartesianMotion, CapabilityRequirement::CartesianMotion) => true,
            (CapabilityRequirement::JointMotion, CapabilityRequirement::JointMotion) => true,
            (CapabilityRequirement::GripperControl, CapabilityRequirement::GripperControl) => true,
            (CapabilityRequirement::TemperatureSensor, CapabilityRequirement::TemperatureSensor) => true,
            (CapabilityRequirement::VibrationSensor, CapabilityRequirement::VibrationSensor) => true,
            (
                CapabilityRequirement::PayloadCapacity { min_grams: provided },
                CapabilityRequirement::PayloadCapacity { min_grams: required },
            ) => provided >= required,
            (
                CapabilityRequirement::Custom { name: provided_name },
                CapabilityRequirement::Custom { name: required_name },
            ) => provided_name == required_name,
            _ => false,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mandatory_resolution_success() {
        let mut registry = ResourceRegistry::new();

        let scara_01 = Resource::new(
            "scara-01",
            ResourceKind::Robot,
            "SCARA Robot 01",
            vec![
                CapabilityRequirement::CartesianMotion,
                CapabilityRequirement::PayloadCapacity { min_grams: 1000 },
            ],
        );
        registry.register(scara_01.clone());

        let station = Station::new("cell-01", "Cell 01", vec![scara_01.to_ref()]);

        let reqs = vec![
            ResourceRequirement::mandatory(CapabilityRequirement::CartesianMotion),
            ResourceRequirement::mandatory(CapabilityRequirement::PayloadCapacity { min_grams: 500 }),
        ];

        let resolved = ResourceResolver::resolve(&station, &registry, &reqs).unwrap();

        assert_eq!(resolved.matches.len(), 2);
        assert_eq!(resolved.matches[0].matched_resource.id.as_str(), "scara-01");
        assert_eq!(resolved.matches[1].matched_resource.id.as_str(), "scara-01");
        assert!(resolved.unresolved_optional.is_empty());
    }

    #[test]
    fn test_unresolved_mandatory_fails() {
        let registry = ResourceRegistry::new();
        let station = Station::new("cell-01", "Cell 01", vec![]);

        let reqs = vec![ResourceRequirement::mandatory(
            CapabilityRequirement::VibrationSensor,
        )];

        let res = ResourceResolver::resolve(&station, &registry, &reqs);
        assert_eq!(
            res,
            Err(ResourceResolutionError::UnresolvedMandatoryRequirement(
                CapabilityRequirement::VibrationSensor
            ))
        );
    }

    #[test]
    fn test_unresolved_optional_passes() {
        let registry = ResourceRegistry::new();
        let station = Station::new("cell-01", "Cell 01", vec![]);

        let reqs = vec![ResourceRequirement::optional(
            CapabilityRequirement::VibrationSensor,
        )];

        let resolved = ResourceResolver::resolve(&station, &registry, &reqs).unwrap();
        assert!(resolved.matches.is_empty());
        assert_eq!(resolved.unresolved_optional.len(), 1);
        assert_eq!(
            resolved.unresolved_optional[0].capability,
            CapabilityRequirement::VibrationSensor
        );
    }

    #[test]
    fn test_station_bound_priority_over_other_candidates() {
        let mut registry = ResourceRegistry::new();

        let scara_01 = Resource::new(
            "scara-01",
            ResourceKind::Robot,
            "SCARA 01",
            vec![CapabilityRequirement::CartesianMotion],
        );
        let scara_02 = Resource::new(
            "scara-02",
            ResourceKind::Robot,
            "SCARA 02",
            vec![CapabilityRequirement::CartesianMotion],
        );

        registry.register(scara_01.clone());
        registry.register(scara_02.clone());

        // Station explicitly binds SCARA-02
        let station = Station::new("cell-02", "Cell 02", vec![scara_02.to_ref()]);

        let reqs = vec![ResourceRequirement::mandatory(
            CapabilityRequirement::CartesianMotion,
        )];

        let resolved = ResourceResolver::resolve(&station, &registry, &reqs).unwrap();
        assert_eq!(resolved.matches[0].matched_resource.id.as_str(), "scara-02");
    }
}
