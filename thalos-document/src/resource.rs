use crate::id::*;
use crate::pose::Pose;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Semantic resource types — logical entities referenced by SemanticProgram
// ---------------------------------------------------------------------------

/// A physical object that can be manipulated (picked, placed, inspected).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Object {
    pub id: ObjectId,
    pub name: String,
    /// Optional semantic category (e.g. "screw", "housing", "tool").
    pub category: Option<String>,
    /// The object's pose in the scene.
    pub pose: Pose,
}

/// A logical location in the workspace (assembly station, bin, tray, etc.).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Location {
    pub id: LocationId,
    pub name: String,
    /// Optional description of this location's purpose.
    pub description: Option<String>,
    /// The location's pose in the scene.
    pub pose: Pose,
}

/// A tool or end-effector that can be attached to the robot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    pub id: ToolId,
    pub name: String,
    /// Optional tool type descriptor (e.g. "gripper", "vacuum", "welder").
    pub tool_type: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::Pose;

    // --- Semantic resource construction ---

    #[test]
    fn object_construction() {
        let obj = Object {
            id: ObjectId("bolt-01".to_string()),
            name: "M8 Bolt".to_string(),
            category: Some("fastener".to_string()),
            pose: Pose {
                position: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
        };
        assert_eq!(obj.id.as_str(), "bolt-01");
        assert_eq!(obj.name, "M8 Bolt");
    }

    #[test]
    fn location_construction() {
        let loc = Location {
            id: LocationId("tray-a".to_string()),
            name: "Tray A".to_string(),
            description: Some("Finished parts tray".to_string()),
            pose: Pose {
                position: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
        };
        assert_eq!(loc.name, "Tray A");
        assert!(loc.description.is_some());
    }

    #[test]
    fn tool_construction() {
        let tool = Tool {
            id: ToolId("gripper-1".to_string()),
            name: "Parallel Gripper".to_string(),
            tool_type: Some("gripper".to_string()),
        };
        assert_eq!(tool.id.as_str(), "gripper-1");
    }

    #[test]
    fn semantic_resources_serde_round_trip() {
        let obj = Object {
            id: ObjectId("bolt".to_string()),
            name: "Bolt".to_string(),
            category: None,
            pose: Pose {
                position: [0.5, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
        };
        let json = serde_json::to_string(&obj).unwrap();
        let back: Object = serde_json::from_str(&json).unwrap();
        assert_eq!(obj, back);
    }
}
