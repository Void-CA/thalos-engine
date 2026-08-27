use crate::robot::joint::JointInfo;

pub struct RobotMetadata {
    pub id: &'static str,
    pub display_name: &'static str,
    pub dof: usize,
    pub joints: &'static [JointInfo],
}
