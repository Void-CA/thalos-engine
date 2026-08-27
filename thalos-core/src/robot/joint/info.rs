use crate::robot::joint::{JointKind, JointLimits};

#[derive(Debug, Clone, Copy)]
pub struct JointInfo {
    pub name: &'static str,
    pub kind: JointKind,
    pub limits: Option<JointLimits>,
}
