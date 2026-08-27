use serde::{Deserialize, Serialize};
use thalos_math::Transform3D;
use crate::ids::{ObjectId, TargetId, TargetName};
use crate::spatial::pose::Pose;

/// Joint position representation for joint-space targets.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointPosition {
    pub positions: Vec<f64>,
}

impl JointPosition {
    pub fn new(positions: Vec<f64>) -> Self {
        Self { positions }
    }
}

/// Target spatial reference specification (ADR-001).
/// Spatial resolution is deferred to the compilation context.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TargetReference {
    Cartesian {
        pose: Pose,
    },
    Joint {
        position: JointPosition,
    },
    Relative {
        reference: TargetId,
        transform: Transform3D,
    },
    Object {
        object: ObjectId,
        offset: Transform3D,
    },
}

/// Target definition in a RobotProgram.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Target {
    pub id: TargetId,
    pub name: TargetName,
    pub reference: TargetReference,
}

impl Target {
    pub fn new(id: TargetId, name: TargetName, reference: TargetReference) -> Self {
        Self { id, name, reference }
    }
}
