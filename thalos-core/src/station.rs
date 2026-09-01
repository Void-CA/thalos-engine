use serde::{Deserialize, Serialize};
use crate::ids::StationId;
use crate::resource::ResourceRef;

/// Station (ADR-014)
/// Persistent operational environment context referencing participating resources.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Station {
    pub id: StationId,
    pub name: String,
    pub resources: Vec<ResourceRef>,
}

impl Station {
    pub fn new(id: impl Into<String>, name: impl Into<String>, resources: Vec<ResourceRef>) -> Self {
        Self {
            id: StationId(id.into()),
            name: name.into(),
            resources,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resource::ResourceKind;

    #[test]
    fn test_station_creation_with_resource_refs() {
        let robot_ref = ResourceRef::new("scara-01", ResourceKind::Robot);
        let temp_ref = ResourceRef::new("temp-sensor-01", ResourceKind::Channel);

        let station = Station::new("station-scara-cell", "SCARA Cell 01", vec![robot_ref.clone(), temp_ref.clone()]);

        assert_eq!(station.id.as_str(), "station-scara-cell");
        assert_eq!(station.name, "SCARA Cell 01");
        assert_eq!(station.resources.len(), 2);
        assert_eq!(station.resources[0], robot_ref);
    }
}
