//! Topological graph of a robot's kinematic tree.
//!
//! `RobotGraph` decouples connectivity from data. The graph describes
//! **which links connect to which, via which joints** — but the actual
//! joint properties live in [`Robot`](crate::Robot) and are referenced
//! by [`JointId`].
//!
//! Unlike [`Robot`] (which uses `HashMap` for storage), `RobotGraph`
//! assigns sequential IDs in BFS order and stores children in
//! alphabetically-sorted vectors. This gives **deterministic traversal**
//! across runs, machines, and hash seeds.
//!
//! ```rust
//! # use thalos_math::Transform3D;
//! # use thalos_models::{Robot, Link, Joint, JointKind};
//! # use thalos_models::graph::RobotGraph;
//! let mut robot = Robot::new("test", "base");
//! robot.add_link(Link::new("base"));
//! robot.add_link(Link::new("tool"));
//! robot.add_joint(Joint::new("j1", JointKind::Revolute, "base", "tool", Transform3D::identity()));
//!
//! let graph = RobotGraph::from_robot(&robot);
//! let path = graph.path_by_name("base", "tool").unwrap();
//! assert_eq!(path.links.len(), 2); // base → tool
//! assert_eq!(path.joints.len(), 1); // j1
//! ```

use std::collections::{HashMap, HashSet, VecDeque};

use crate::Robot;

/// Index of a link within a [`RobotGraph`].
pub type LinkId = u32;

/// Index of a joint within a [`RobotGraph`].
pub type JointId = u32;

/// A kinematic path: alternating links and joints from root to target.
///
/// For the path `root → j₁ → l₁ → j₂ → l₂ → ... → target`:
///
/// | Field    | Contents                              |
/// |----------|---------------------------------------|
/// | `links`  | `[root, l₁, l₂, ..., target]`         |
/// | `joints` | `[j₁, j₂, ...]`                       |
///
/// Both vectors are in root→target order.
#[derive(Debug, Clone)]
pub struct Path {
    pub links: Vec<LinkId>,
    pub joints: Vec<JointId>,
}

/// Topological structure of a robot's kinematic tree.
///
/// # Ordering
///
/// Links and joints are assigned sequential [`LinkId`]/[`JointId`] values
/// during construction via breadth-first traversal starting from the root.
/// Within each BFS level, joints are sorted alphabetically by name. This
/// guarantees deterministic IDs across runs.
///
/// # Relationship to `Robot`
///
/// `RobotGraph` stores **only connectivity**. Joint properties
/// (limits, axis, origin, …) live in [`Robot`]. To look up a joint:
///
/// ```ignore
/// let j_name = &graph.joint_name[joint_id];
/// let joint = &robot.joints[j_name];
/// ```
///
/// This avoids duplicating data while keeping the graph lightweight.
#[derive(Debug, Clone)]
pub struct RobotGraph {
    /// Root link of the entire tree.
    pub root: LinkId,
    /// Children of each link, indexed by [`LinkId`].
    /// `children[id]` is a `Vec<LinkId>` sorted in BFS+alphabetical order.
    children: Vec<Vec<LinkId>>,
    /// Parent of each link (`None` for root).
    parent: Vec<Option<LinkId>>,
    /// Joint connecting each non-root link to its parent (`None` for root).
    parent_joint: Vec<Option<JointId>>,
    /// Human-readable link names, indexed by [`LinkId`].
    pub link_name: Vec<String>,
    /// Human-readable joint names, indexed by [`JointId`].
    pub joint_name: Vec<String>,
    /// Name → ID lookup for links.
    link_index: HashMap<String, LinkId>,
    /// Name → ID lookup for joints.
    joint_index: HashMap<String, JointId>,
}

impl RobotGraph {
    /// Build a graph from a [`Robot`].
    ///
    /// Links and joints not reachable from `robot.root_link` are silently
    /// omitted from the graph (dangling subtrees).
    pub fn from_robot(robot: &Robot) -> Self {
        let n_links = robot.links.len();
        let n_joints = robot.joints.len();

        let mut link_name: Vec<String> = Vec::with_capacity(n_links);
        let mut joint_name: Vec<String> = Vec::with_capacity(n_joints);
        let mut link_index: HashMap<String, LinkId> = HashMap::with_capacity(n_links);
        let mut joint_index: HashMap<String, JointId> = HashMap::with_capacity(n_joints);
        let mut children: Vec<Vec<LinkId>> = Vec::with_capacity(n_links);
        let mut parent: Vec<Option<LinkId>> = Vec::with_capacity(n_links);
        let mut parent_joint: Vec<Option<JointId>> = Vec::with_capacity(n_links);

        // Build lookup: parent_name → [(joint_name, child_name)],
        // sorted alphabetically by joint name within each parent.
        let mut child_map: HashMap<&str, Vec<(&str, &str)>> = HashMap::new();
        for joint in robot.joints.values() {
            child_map
                .entry(joint.parent.as_str())
                .or_default()
                .push((joint.name.as_str(), joint.child.as_str()));
        }
        for children_list in child_map.values_mut() {
            children_list.sort_by(|a, b| a.0.cmp(b.0));
        }

        // Root link gets ID 0.
        let root_id: LinkId = 0;
        link_index.insert(robot.root_link.clone(), root_id);
        link_name.push(robot.root_link.clone());
        children.push(Vec::new());
        parent.push(None);
        parent_joint.push(None);

        let mut visited: HashSet<String> = HashSet::new();
        visited.insert(robot.root_link.clone());

        let mut queue: VecDeque<String> = VecDeque::new();
        queue.push_back(robot.root_link.clone());

        while let Some(current_name) = queue.pop_front() {
            let current_id = link_index[&current_name];

            if let Some(entries) = child_map.get(current_name.as_str()) {
                for &(j_name, c_name) in entries {
                    // Skip dangling references and already-visited links.
                    if !robot.links.contains_key(c_name) || visited.contains(c_name) {
                        continue;
                    }
                    visited.insert(c_name.to_string());

                    // Assign link ID.
                    let child_id = link_name.len() as LinkId;
                    link_index.insert(c_name.to_string(), child_id);
                    link_name.push(c_name.to_string());
                    children.push(Vec::new());
                    parent.push(Some(current_id));
                    parent_joint.push(None); // filled below

                    // Assign joint ID.
                    let j_id = joint_name.len() as JointId;
                    joint_index.insert(j_name.to_string(), j_id);
                    joint_name.push(j_name.to_string());
                    parent_joint[child_id as usize] = Some(j_id);

                    // Record edge.
                    children[current_id as usize].push(child_id);

                    queue.push_back(c_name.to_string());
                }
            }
        }

        Self {
            root: root_id,
            children,
            parent,
            parent_joint,
            link_name,
            joint_name,
            link_index,
            joint_index,
        }
    }

    /// Number of links in the connected tree (may be less than `robot.links.len()`
    /// if there are unreachable subtrees).
    pub fn link_count(&self) -> usize {
        self.link_name.len()
    }

    /// Number of joints in the connected tree.
    pub fn joint_count(&self) -> usize {
        self.joint_name.len()
    }

    /// Resolve a link name to its [`LinkId`].
    pub fn link_id(&self, name: &str) -> Option<LinkId> {
        self.link_index.get(name).copied()
    }

    /// Resolve a joint name to its [`JointId`].
    pub fn joint_id(&self, name: &str) -> Option<JointId> {
        self.joint_index.get(name).copied()
    }

    /// Iterate over all link IDs in the connected tree.
    pub fn link_ids(&self) -> impl Iterator<Item = LinkId> + '_ {
        0..self.link_count() as LinkId
    }

    /// Get the children of a link.
    pub fn children(&self, link: LinkId) -> &[LinkId] {
        if (link as usize) < self.children.len() {
            &self.children[link as usize]
        } else {
            &[]
        }
    }

    /// Get the parent of a link (`None` for root).
    pub fn parent(&self, link: LinkId) -> Option<LinkId> {
        self.parent.get(link as usize).copied().flatten()
    }

    /// Get the joint connecting a link to its parent (`None` for root).
    pub fn parent_joint(&self, link: LinkId) -> Option<JointId> {
        self.parent_joint.get(link as usize).copied().flatten()
    }

    /// Name of a link.
    pub fn link_name(&self, id: LinkId) -> Option<&str> {
        self.link_name.get(id as usize).map(String::as_str)
    }

    /// Name of a joint.
    pub fn joint_name(&self, id: JointId) -> Option<&str> {
        self.joint_name.get(id as usize).map(String::as_str)
    }

    /// Find the path from `root` to `target` in the kinematic tree.
    ///
    /// Returns `None` if either link is unknown or no path exists.
    ///
    /// # Determinism
    ///
    /// Children are visited in the stable order defined at construction
    /// (BFS + alphabetical joint name). The returned path is therefore
    /// deterministic for the same input.
    pub fn path(&self, root: LinkId, target: LinkId) -> Option<Path> {
        if root >= self.link_count() as LinkId || target >= self.link_count() as LinkId {
            return None;
        }

        let mut links = Vec::new();
        let mut joints = Vec::new();
        let mut visited = HashSet::new();

        if !self.dfs_path(root, target, &mut links, &mut joints, &mut visited) {
            return None;
        }

        Some(Path { links, joints })
    }

    /// String-based convenience wrapper around [`path()`](Self::path).
    pub fn path_by_name(&self, root: &str, target: &str) -> Option<Path> {
        let r = self.link_id(root)?;
        let t = self.link_id(target)?;
        self.path(r, t)
    }

    // ── Private helpers ───────────────────────────────────────────

    /// DFS building `path` forward (root-first order).
    fn dfs_path(
        &self,
        current: LinkId,
        target: LinkId,
        links: &mut Vec<LinkId>,
        joints: &mut Vec<JointId>,
        visited: &mut HashSet<LinkId>,
    ) -> bool {
        links.push(current);

        if current == target {
            return true;
        }

        if !visited.insert(current) {
            links.pop();
            return false;
        }

        for &child in &self.children[current as usize] {
            // Edge joint: the joint connecting child to its parent (current).
            if let Some(j_id) = self.parent_joint(child) {
                joints.push(j_id);

                if self.dfs_path(child, target, links, joints, visited) {
                    return true;
                }

                joints.pop();
            }
        }

        visited.remove(&current);
        links.pop();
        false
    }

    /// Collect all leaf link IDs (links with no children).
    pub fn leaves(&self) -> Vec<LinkId> {
        self.link_ids()
            .filter(|&id| self.children[id as usize].is_empty())
            .collect()
    }

    /// Count actuated (non-fixed) joints along a path.
    ///
    /// Requires the [`Robot`] to resolve joint kinds.
    pub fn actuated_count(&self, path: &Path, robot: &Robot) -> usize {
        path.joints
            .iter()
            .filter(|&&j_id| {
                let j_name = &self.joint_name[j_id as usize];
                let joint = &robot.joints[j_name];
                !joint.kind.is_fixed()
            })
            .count()
    }
}

// ─── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Robot,
        joint::{Joint, JointKind, JointLimits},
        link::Link,
    };
    use thalos_math::Transform3D;

    fn identity_origin() -> Transform3D {
        Transform3D::identity()
    }

    fn simple_chain() -> Robot {
        let mut robot = Robot::new("chain", "base");
        robot.add_link(Link::new("base"));
        robot.add_link(Link::new("link1"));
        robot.add_link(Link::new("link2"));
        robot.add_joint(Joint {
            name: "j1".into(),
            kind: JointKind::Revolute,
            parent: "base".into(),
            child: "link1".into(),
            origin: identity_origin(),
            axis: None,
            limits: Some(JointLimits::new(-1.0, 1.0)),
        });
        robot.add_joint(Joint {
            name: "j2".into(),
            kind: JointKind::Revolute,
            parent: "link1".into(),
            child: "link2".into(),
            origin: identity_origin(),
            axis: None,
            limits: Some(JointLimits::new(-1.0, 1.0)),
        });
        robot
    }

    #[test]
    fn graph_from_robot() {
        let robot = simple_chain();
        let graph = RobotGraph::from_robot(&robot);

        assert_eq!(graph.link_count(), 3);
        assert_eq!(graph.joint_count(), 2);
        assert_eq!(graph.link_name[0], "base");
        assert_eq!(graph.link_name[1], "link1");
        assert_eq!(graph.link_name[2], "link2");
    }

    #[test]
    fn path_root_to_target() {
        let robot = simple_chain();
        let graph = RobotGraph::from_robot(&robot);

        let path = graph.path_by_name("base", "link2").unwrap();
        assert_eq!(path.links.len(), 3); // base → link1 → link2
        assert_eq!(path.joints.len(), 2); // j1, j2
    }

    #[test]
    fn path_single_link() {
        let robot = simple_chain();
        let graph = RobotGraph::from_robot(&robot);

        let path = graph.path_by_name("base", "base").unwrap();
        assert_eq!(path.links, vec![0]);
        assert!(path.joints.is_empty());
    }

    #[test]
    fn path_nonexistent_target() {
        let robot = simple_chain();
        let graph = RobotGraph::from_robot(&robot);

        assert!(graph.path_by_name("base", "ghost").is_none());
    }

    #[test]
    fn leaves_detection() {
        let robot = simple_chain();
        let graph = RobotGraph::from_robot(&robot);

        let leaves = graph.leaves();
        assert_eq!(leaves, vec![2]); // only link2 is a leaf
    }

    #[test]
    fn alphabetical_ordering_at_same_level() {
        // Two joints from same parent: "b" should come before "a"
        // in the graph because joints are sorted alphabetically.
        let mut robot = Robot::new("test", "base");
        robot.add_link(Link::new("base"));
        robot.add_link(Link::new("left"));
        robot.add_link(Link::new("right"));

        // Note: "b_joint" starts with 'b', "a_joint" starts with 'a'.
        // They should appear in alphabetical order: a_joint, b_joint.
        // But we add them in reverse order to test sorting.
        robot.add_joint(Joint {
            name: "b_joint".into(),
            kind: JointKind::Fixed,
            parent: "base".into(),
            child: "right".into(),
            origin: identity_origin(),
            axis: None,
            limits: None,
        });
        robot.add_joint(Joint {
            name: "a_joint".into(),
            kind: JointKind::Fixed,
            parent: "base".into(),
            child: "left".into(),
            origin: identity_origin(),
            axis: None,
            limits: None,
        });

        let graph = RobotGraph::from_robot(&robot);

        // Children should be in alphabetical joint order: a_joint, b_joint
        let children = graph.children(graph.root);
        assert_eq!(children.len(), 2);

        let child_a = &graph.link_name[children[0] as usize];
        let child_b = &graph.link_name[children[1] as usize];

        // "a_joint" has child "left", "b_joint" has child "right"
        assert_eq!(child_a, "left", "a_joint should come before b_joint");
        assert_eq!(child_b, "right");
    }

    #[test]
    fn actuated_count_on_mixed_chain() {
        let mut robot = Robot::new("test", "base");
        robot.add_link(Link::new("base"));
        robot.add_link(Link::new("mid"));
        robot.add_link(Link::new("tip"));
        robot.add_joint(Joint {
            name: "revolute_j".into(),
            kind: JointKind::Revolute,
            parent: "base".into(),
            child: "mid".into(),
            origin: identity_origin(),
            axis: None,
            limits: Some(JointLimits::new(-1.0, 1.0)),
        });
        robot.add_joint(Joint {
            name: "fixed_j".into(),
            kind: JointKind::Fixed,
            parent: "mid".into(),
            child: "tip".into(),
            origin: identity_origin(),
            axis: None,
            limits: None,
        });

        let graph = RobotGraph::from_robot(&robot);
        let path = graph.path_by_name("base", "tip").unwrap();
        assert_eq!(graph.actuated_count(&path, &robot), 1);
    }

    #[test]
    fn dangling_link_omitted() {
        let mut robot = Robot::new("test", "root");
        robot.add_link(Link::new("root"));
        robot.add_link(Link::new("connected"));
        robot.add_link(Link::new("dangling")); // no joint connects it
        robot.add_joint(Joint {
            name: "j".into(),
            kind: JointKind::Fixed,
            parent: "root".into(),
            child: "connected".into(),
            origin: identity_origin(),
            axis: None,
            limits: None,
        });

        let graph = RobotGraph::from_robot(&robot);
        assert_eq!(graph.link_count(), 2); // dangling omitted
        assert!(graph.link_id("dangling").is_none());
    }
}
