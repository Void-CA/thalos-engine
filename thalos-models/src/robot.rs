use crate::{Joint, Link, Material};
use std::collections::HashMap;

/// A complete robot description.
///
/// A `Robot` is a tree (or graph) of links connected by joints, with
/// an explicit `root_link`. This representation is URDF-native and
/// supports arbitrary kinematic topologies — serial chains, humanoids,
/// quadrupeds, multi-arm systems, etc.
///
/// # Tree structure
///
/// The kinematic hierarchy is defined by the `parent`/`child` fields
/// on each [`Joint`](crate::Joint). To walk the tree starting from
/// `root_link`:
///
/// ```ignore
/// for joint in robot.joints.values() {
///     if joint.parent == current_link_name { … }
/// }
/// ```
///
/// Corresponds to `<robot>` in URDF.
#[derive(Debug, Clone, PartialEq)]
pub struct Robot {
    pub name: String,

    /// All links, keyed by name.
    pub links: HashMap<String, Link>,

    /// All joints, keyed by name.
    pub joints: HashMap<String, Joint>,

    /// Name of the root (base) link.
    pub root_link: String,

    /// Shared visual materials.
    pub materials: HashMap<String, Material>,
}

impl Robot {
    pub fn new(name: impl Into<String>, root_link: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            links: HashMap::new(),
            joints: HashMap::new(),
            root_link: root_link.into(),
            materials: HashMap::new(),
        }
    }

    /// Add a link to the robot.
    pub fn add_link(&mut self, link: Link) {
        self.links.insert(link.name.clone(), link);
    }

    /// Add a joint to the robot.
    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.insert(joint.name.clone(), joint);
    }

    /// Add a shared material.
    pub fn add_material(&mut self, material: Material) {
        self.materials.insert(material.name.clone(), material);
    }

    /// Iterate over all joints, following the kinematic chain from
    /// `root_link` as a breadth-first traversal.
    ///
    /// Returns `None` if the robot graph contains cycles or references
    /// to non-existent links.
    pub fn bfs_joints(&self) -> Option<Vec<&Joint>> {
        use std::collections::VecDeque;

        let mut visited_links: std::collections::HashSet<&str> =
            [self.root_link.as_str()].into_iter().collect();
        let mut queue: VecDeque<&str> = [self.root_link.as_str()].into_iter().collect();
        let mut ordered = Vec::new();

        while let Some(current) = queue.pop_front() {
            for joint in self.joints.values() {
                if joint.parent != current {
                    continue;
                }
                if !self.links.contains_key(&joint.child) {
                    return None;
                }
                if !visited_links.insert(joint.child.as_str()) {
                    return None; // cycle detected
                }
                queue.push_back(&joint.child);
                ordered.push(joint);
            }
        }

        Some(ordered)
    }
}
