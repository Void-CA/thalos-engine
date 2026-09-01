use std::collections::{HashMap, HashSet};

use crate::scene::{VisualFrame, VisualId, VisualScene};

#[derive(Debug, Clone, PartialEq)]
pub enum SceneError {
    MissingWorld,
    MissingFrame(VisualId),
    DuplicateId { id: VisualId },
    BrokenTopology { frame: VisualId },
    NonFiniteValue { frame: VisualId },
    InvalidQuaternion { frame: VisualId, norm: f64 },
    OrphanLink { index: usize },
    TwistsMismatch { expected: usize, found: usize },
}

impl std::fmt::Display for SceneError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SceneError::MissingWorld => write!(f, "scene must contain a 'world' frame"),
            SceneError::MissingFrame(id) => write!(f, "parent frame '{}' not found in scene", id),
            SceneError::DuplicateId { id } => write!(f, "duplicate frame id '{}'", id),
            SceneError::BrokenTopology { frame } => {
                write!(f, "topology error involving frame '{}'", frame)
            }
            SceneError::NonFiniteValue { frame } => {
                write!(f, "non-finite value in frame '{}'", frame)
            }
            SceneError::InvalidQuaternion { frame, norm } => {
                write!(
                    f,
                    "quaternion norm in frame '{}' is {} (expected 1.0)",
                    frame, norm
                )
            }
            SceneError::OrphanLink { index } => write!(f, "orphan link at index {}", index),
            SceneError::TwistsMismatch { expected, found } => {
                write!(f, "twists count {} != joint axes count {}", found, expected)
            }
        }
    }
}

pub struct SceneValidator {
    pub epsilon_rot: f64,
}

impl Default for SceneValidator {
    fn default() -> Self {
        Self { epsilon_rot: 1e-6 }
    }
}

impl SceneValidator {
    pub fn new(epsilon_rot: f64) -> Self {
        Self { epsilon_rot }
    }

    pub fn validate(&self, scene: &VisualScene) -> Result<(), SceneError> {
        self.check_world_exists(scene)?;
        self.check_ids_unique(scene)?;
        self.check_parents_exist(scene)?;
        self.check_no_cycles(scene)?;
        self.check_connectivity(scene)?;
        self.check_finite(scene)?;
        self.check_quaternion_norm(scene)?;
        self.check_links_consistency(scene)?;
        self.check_twists_consistency(scene)?;
        Ok(())
    }

    fn check_world_exists(&self, scene: &VisualScene) -> Result<(), SceneError> {
        if !scene.frames.iter().any(|f| f.id == "world") {
            return Err(SceneError::MissingWorld);
        }
        Ok(())
    }

    fn check_ids_unique(&self, scene: &VisualScene) -> Result<(), SceneError> {
        let mut seen = HashSet::new();
        for frame in &scene.frames {
            if !seen.insert(&frame.id) {
                return Err(SceneError::DuplicateId {
                    id: frame.id.clone(),
                });
            }
        }
        Ok(())
    }

    fn check_parents_exist(&self, scene: &VisualScene) -> Result<(), SceneError> {
        let ids: HashSet<&str> = scene.frames.iter().map(|f| f.id.as_str()).collect();
        for frame in &scene.frames {
            if let Some(ref parent) = frame.parent {
                if !ids.contains(parent.as_str()) {
                    return Err(SceneError::MissingFrame(parent.clone()));
                }
            }
        }
        Ok(())
    }

    fn check_no_cycles(&self, scene: &VisualScene) -> Result<(), SceneError> {
        let by_id: HashMap<&str, &VisualFrame> =
            scene.frames.iter().map(|f| (f.id.as_str(), f)).collect();

        for frame in &scene.frames {
            let mut visited = HashSet::new();
            let mut current: Option<&str> = Some(&frame.id);
            while let Some(id) = current {
                if !visited.insert(id) {
                    return Err(SceneError::BrokenTopology {
                        frame: frame.id.clone(),
                    });
                }
                current = by_id.get(id).and_then(|f| f.parent.as_deref());
            }
        }
        Ok(())
    }

    fn check_connectivity(&self, scene: &VisualScene) -> Result<(), SceneError> {
        let mut children: HashMap<&str, Vec<&str>> = HashMap::new();
        for frame in &scene.frames {
            if let Some(ref parent) = frame.parent {
                children.entry(parent.as_str()).or_default().push(&frame.id);
            }
        }

        let mut reachable = HashSet::new();
        let mut queue = vec!["world"];
        reachable.insert("world");

        while let Some(id) = queue.pop() {
            if let Some(kids) = children.get(id) {
                for child in kids {
                    if reachable.insert(child) {
                        queue.push(child);
                    }
                }
            }
        }

        for frame in &scene.frames {
            if !reachable.contains(frame.id.as_str()) {
                return Err(SceneError::BrokenTopology {
                    frame: frame.id.clone(),
                });
            }
        }
        Ok(())
    }

    fn check_finite(&self, scene: &VisualScene) -> Result<(), SceneError> {
        for frame in &scene.frames {
            for &v in &frame.translation {
                if !v.is_finite() {
                    return Err(SceneError::NonFiniteValue {
                        frame: frame.id.clone(),
                    });
                }
            }
            for &v in &frame.rotation {
                if !v.is_finite() {
                    return Err(SceneError::NonFiniteValue {
                        frame: frame.id.clone(),
                    });
                }
            }
        }
        for (i, axis) in scene.joint_axes.iter().enumerate() {
            for &v in &axis.origin {
                if !v.is_finite() {
                    return Err(SceneError::NonFiniteValue {
                        frame: format!("joint_axis[{}]", i),
                    });
                }
            }
            for &v in &axis.axis {
                if !v.is_finite() {
                    return Err(SceneError::NonFiniteValue {
                        frame: format!("joint_axis[{}]", i),
                    });
                }
            }
        }
        Ok(())
    }

    fn check_quaternion_norm(&self, scene: &VisualScene) -> Result<(), SceneError> {
        for frame in &scene.frames {
            let r = &frame.rotation;
            let norm_sq = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3];
            if (norm_sq.sqrt() - 1.0).abs() > self.epsilon_rot {
                return Err(SceneError::InvalidQuaternion {
                    frame: frame.id.clone(),
                    norm: norm_sq.sqrt(),
                });
            }
        }
        Ok(())
    }

    fn check_links_consistency(&self, scene: &VisualScene) -> Result<(), SceneError> {
        let by_id: HashMap<&str, &VisualFrame> =
            scene.frames.iter().map(|f| (f.id.as_str(), f)).collect();

        let mut expected: Vec<([f64; 3], [f64; 3])> = Vec::new();
        for frame in &scene.frames {
            if let Some(ref parent) = frame.parent {
                if let Some(pf) = by_id.get(parent.as_str()) {
                    expected.push((pf.translation, frame.translation));
                }
            }
        }

        for (i, link) in scene.links.iter().enumerate() {
            let ok = expected.iter().any(|(s, e)| {
                (link.start[0] - s[0]).abs() < 1e-10
                    && (link.start[1] - s[1]).abs() < 1e-10
                    && (link.start[2] - s[2]).abs() < 1e-10
                    && (link.end[0] - e[0]).abs() < 1e-10
                    && (link.end[1] - e[1]).abs() < 1e-10
                    && (link.end[2] - e[2]).abs() < 1e-10
            });
            if !ok {
                return Err(SceneError::OrphanLink { index: i });
            }
        }
        Ok(())
    }

    fn check_twists_consistency(&self, scene: &VisualScene) -> Result<(), SceneError> {
        let n_axes = scene.joint_axes.len();
        let n_twists = scene.twists.len();
        if n_twists > 0 && n_twists != n_axes {
            return Err(SceneError::TwistsMismatch {
                expected: n_axes,
                found: n_twists,
            });
        }
        Ok(())
    }
}
