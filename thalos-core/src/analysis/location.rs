//! Typed locations where an observation phenomenon is anchored (spec I2).
//!
//! `Location` is part of the machine-readable identification of a phenomenon:
//! `kind` + `artifact` + `location` must suffice to identify an observation without
//! parsing text.

use serde::{Deserialize, Serialize};

use crate::analysis::region::RegionId;
use crate::ids::{ObjectId, OperationId};

/// Where an observation phenomenon is anchored in the analyzed artifact.
///
/// The enum is `#[non_exhaustive]`: new location kinds can be added without breaking
/// downstream exhaustive matches.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Location {
    /// A joint index (0-based) of the robot model.
    Joint(usize),
    /// A waypoint index (0-based) of a trajectory.
    Waypoint(usize),
    /// A program operation, identified by its [`OperationId`].
    Operation(OperationId),
    /// A problem region, identified by its [`RegionId`].
    Region(RegionId),
    /// A scene object, identified by its [`ObjectId`].
    Object(ObjectId),
    /// A kinematic frame name (e.g. `"tool_flange"`).
    Frame(String),
    /// An absolute timestamp in **milliseconds** (integer), the wire-clean equivalent of
    /// the seconds-based timestamps used in execution plans. Keeps the enum `Eq`/`Hash`
    /// capable and JSON unambiguous.
    Timestamp(u64),
}

#[cfg(test)]
mod tests {
    use super::Location;
    use crate::analysis::region::RegionId;
    use crate::ids::{ObjectId, OperationId};
    use serde_json::json;

    #[test]
    fn waypoint_holds_index() {
        let loc = Location::Waypoint(5);
        assert_eq!(loc, Location::Waypoint(5));
        assert_ne!(loc, Location::Waypoint(6));
    }

    #[test]
    fn joint_holds_index() {
        let loc = Location::Joint(3);
        assert_eq!(loc, Location::Joint(3));
        assert_ne!(loc, Location::Waypoint(3));
    }

    #[test]
    fn operation_holds_operation_id() {
        let loc = Location::Operation(OperationId("pick".to_string()));
        assert_eq!(loc, Location::Operation(OperationId("pick".to_string())));
        assert_ne!(loc, Location::Operation(OperationId("place".to_string())));
    }

    #[test]
    fn object_holds_object_id() {
        let loc = Location::Object(ObjectId("box_1".to_string()));
        assert_eq!(loc, Location::Object(ObjectId("box_1".to_string())));
        assert_ne!(loc, Location::Object(ObjectId("box_2".to_string())));
    }

    #[test]
    fn region_holds_region_id() {
        let loc = Location::Region(RegionId(7));
        assert_eq!(loc, Location::Region(RegionId(7)));
        assert_ne!(loc, Location::Region(RegionId(8)));
    }

    #[test]
    fn frame_holds_name() {
        let loc = Location::Frame("tool_flange".to_string());
        assert_eq!(loc, Location::Frame("tool_flange".to_string()));
    }

    #[test]
    fn timestamp_holds_milliseconds() {
        let loc = Location::Timestamp(1234);
        assert_eq!(loc, Location::Timestamp(1234));
        assert_ne!(loc, Location::Timestamp(1235));
    }

    #[test]
    fn all_seven_variants_are_distinct() {
        let locations = [
            Location::Joint(0),
            Location::Waypoint(0),
            Location::Operation(OperationId("o".to_string())),
            Location::Region(RegionId(0)),
            Location::Object(ObjectId("obj".to_string())),
            Location::Frame("f".to_string()),
            Location::Timestamp(0),
        ];
        for (i, a) in locations.iter().enumerate() {
            for (j, b) in locations.iter().enumerate() {
                if i != j {
                    assert_ne!(a, b, "locations {i} and {j} must be distinct");
                }
            }
        }
    }

    #[test]
    fn serializes_as_machine_readable_enum() {
        let loc = Location::Waypoint(5);
        let json = serde_json::to_value(&loc).expect("to_value");
        assert_eq!(json, json!({"Waypoint": 5}));
    }

    #[test]
    fn round_trip_via_json() {
        for loc in [
            Location::Joint(2),
            Location::Waypoint(9),
            Location::Operation(OperationId("pick".to_string())),
            Location::Region(RegionId(3)),
            Location::Object(ObjectId("box_1".to_string())),
            Location::Frame("base".to_string()),
            Location::Timestamp(1500),
        ] {
            let json = serde_json::to_string(&loc).expect("serialize");
            let back: Location = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(back, loc);
        }
    }
}
