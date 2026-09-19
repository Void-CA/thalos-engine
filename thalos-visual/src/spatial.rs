use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use thalos_core::kinematics::{FramePose, SpatialState};
use thalos_core::robot::serial_chain::SerialChain;
use thalos_core::spatial::frame::FrameId;

use crate::align_y_to;
use crate::builder::frame_visual_id;

/// Render-ready transform projected from the neutral [`SpatialState`].
///
/// This is the VIEWPORT representation: frames keyed by visual id, links keyed
/// by joint id, with the cylinder midpoint/orientation/scale already resolved.
/// It is NOT the spatial state of the kinematic model — see
/// `docs/system/architecture/spatial-state-contract.md`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpatialTransform {
    pub id: String,
    pub translation: [f64; 3],
    /// Unit quaternion `[w, x, y, z]`.
    pub rotation: [f64; 4],
    pub scale: [f64; 3],
}

/// Project the neutral spatial state of the kinematic model into render-ready
/// transforms.
///
/// `chain` is the STATIC kinematic topology (visual frame names + segment
/// parent/child); it is projection CONTEXT, not part of the observed state.
/// Frame poses provide the per-tick positions/orientations; link transforms
/// are derived from the parent/child frame poses (midpoint + `align_y_to` +
/// length scale).
pub fn project_spatial(chain: &SerialChain, spatial: &SpatialState) -> Vec<SpatialTransform> {
    let by_frame: HashMap<FrameId, &FramePose> =
        spatial.frames.iter().map(|f| (f.frame, f)).collect();

    let mut transforms = Vec::with_capacity(spatial.frames.len() + chain.segments.len());

    // 1. Frames: one transform per frame pose, keyed by visual id.
    for pose in &spatial.frames {
        transforms.push(SpatialTransform {
            id: frame_visual_id(chain, &pose.frame),
            translation: pose.position,
            rotation: pose.orientation,
            scale: [1.0, 1.0, 1.0],
        });
    }

    // 2. Links: a scaled cylinder between the parent and child frame poses.
    for segment in &chain.segments {
        if segment.joint.dof() == 0 {
            continue;
        }

        let (Some(parent), Some(child)) =
            (by_frame.get(&segment.parent), by_frame.get(&segment.child))
        else {
            continue;
        };

        let start = parent.position;
        let end = child.position;
        let dx = end[0] - start[0];
        let dy = end[1] - start[1];
        let dz = end[2] - start[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();

        if len < 1e-10 {
            continue;
        }

        let midpoint = [
            (start[0] + end[0]) / 2.0,
            (start[1] + end[1]) / 2.0,
            (start[2] + end[2]) / 2.0,
        ];

        transforms.push(SpatialTransform {
            id: segment.joint.id().to_string(),
            translation: midpoint,
            rotation: align_y_to([dx / len, dy / len, dz / len]),
            scale: [1.0, len, 1.0],
        });
    }

    transforms
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::kinematics::forward::ForwardKinematics;
    use thalos_core::models::planar_2r::Planar2RSpec;

    #[test]
    fn projection_has_a_transform_per_frame_plus_link() {
        let chain = Planar2RSpec::ideal().build();
        let fk = ForwardKinematics::new(chain.clone()).evaluate(&[0.4, -0.3]);
        let spatial = SpatialState::from_fk(&fk);

        let transforms = project_spatial(&chain, &spatial);

        // All frames + one link per actuated segment (2R → 2 links).
        assert_eq!(transforms.len(), spatial.frames.len() + 2);
        assert_eq!(transforms[0].id, "world");
        assert!(
            transforms
                .iter()
                .any(|t| t.scale[1] > 0.0 && t.id != "world")
        );
    }

    #[test]
    fn projection_is_deterministic() {
        let chain = Planar2RSpec::ideal().build();
        let fk = ForwardKinematics::new(chain.clone()).evaluate(&[0.1, 0.2]);
        let spatial = SpatialState::from_fk(&fk);

        assert_eq!(
            project_spatial(&chain, &spatial),
            project_spatial(&chain, &spatial)
        );
    }

    #[test]
    fn link_transform_matches_frame_positions() {
        let chain = Planar2RSpec::ideal().build();
        let fk = ForwardKinematics::new(chain.clone()).evaluate(&[0.3, 0.5]);
        let spatial = SpatialState::from_fk(&fk);
        let transforms = project_spatial(&chain, &spatial);

        let first_segment = chain
            .segments
            .iter()
            .find(|s| s.joint.dof() > 0)
            .expect("2R has actuated segments");
        let link = transforms
            .iter()
            .find(|t| t.id == first_segment.joint.id().to_string())
            .expect("link transform exists");

        let parent = fk.pose(&first_segment.parent).unwrap().translation();
        let child = fk.pose(&first_segment.child).unwrap().translation();
        assert!((link.translation[0] - (parent.x + child.x) / 2.0).abs() < 1e-12);
        assert!((link.translation[1] - (parent.y + child.y) / 2.0).abs() < 1e-12);
        assert!((link.translation[2] - (parent.z + child.z) / 2.0).abs() < 1e-12);
    }
}
