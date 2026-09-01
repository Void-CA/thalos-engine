use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::scene::{VisualFrame, VisualId, VisualScene};

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct SceneDiff {
    pub frames_removed: Vec<VisualId>,
    pub frames_added: Vec<VisualId>,
    pub changed_frames: Vec<ChangedFrame>,
}

impl SceneDiff {
    pub fn between(old: &VisualScene, new: &VisualScene, epsilon: f64) -> Self {
        let new_by_id: HashMap<&str, &VisualFrame> =
            new.frames.iter().map(|f| (f.id.as_str(), f)).collect();
        let old_by_id: HashMap<&str, &VisualFrame> =
            old.frames.iter().map(|f| (f.id.as_str(), f)).collect();

        let mut diff = SceneDiff::default();

        for frame in &old.frames {
            if !new_by_id.contains_key(frame.id.as_str()) {
                diff.frames_removed.push(frame.id.clone());
            }
        }

        for frame in &new.frames {
            if !old_by_id.contains_key(frame.id.as_str()) {
                diff.frames_added.push(frame.id.clone());
            }
        }

        for frame in &old.frames {
            if let Some(new_frame) = new_by_id.get(frame.id.as_str()) {
                let tx_dist = translation_distance(&frame.translation, &new_frame.translation);
                let rot_angle = geodesic_rotation_deg(&frame.rotation, &new_frame.rotation);

                if tx_dist > epsilon || rot_angle > epsilon {
                    diff.changed_frames.push(ChangedFrame {
                        id: frame.id.clone(),
                        translation_delta: round_to(tx_dist, 6),
                        rotation_angle_deg: round_to(rot_angle, 4),
                    });
                }
            }
        }

        diff
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ChangedFrame {
    pub id: VisualId,
    pub translation_delta: f64,
    pub rotation_angle_deg: f64,
}

fn translation_distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn geodesic_rotation_deg(a: &[f64; 4], b: &[f64; 4]) -> f64 {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];

    let norm_a = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]).sqrt();
    let norm_b = (b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]).sqrt();
    let denominator = norm_a * norm_b;

    if denominator < 1e-15 {
        return 0.0;
    }

    let cos_half = (dot / denominator).abs().clamp(-1.0, 1.0);
    (2.0 * cos_half.acos()).to_degrees()
}

fn round_to(val: f64, places: usize) -> f64 {
    let factor = 10_f64.powi(places as i32);
    (val * factor).round() / factor
}
