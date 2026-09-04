use crate::station::{RoboticsModuleId, StationId};

/// Identifies where a robot is referenced within the workspace.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RobotReference {
    pub station_id: StationId,
    pub module_id: RoboticsModuleId,
}

/// Checks whether a robot is referenced by any station module.
///
/// This trait exists so that `RobotService` can validate delete operations
/// without depending on `StationService` directly.
pub trait RobotReferenceChecker: Send + Sync {
    /// Returns the first reference to the given robot, or None if unreferenced.
    fn find_robot_reference(&self, robot_id: &str) -> Option<RobotReference>;
}
