use serde::{Deserialize, Serialize};

use crate::kinematics::forward::result::FKResult;
use crate::spatial::frame::FrameId;

/// World pose of a single kinematic frame, produced by the same FK evaluation
/// that yields the tick's TCP.
///
/// This is a NEUTRAL spatial state: it carries no visual id, no link geometry,
/// no scale and no rendering concept. The projection toward a visual
/// representation is the responsibility of the visual layer.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FramePose {
    pub frame: FrameId,
    pub position: [f64; 3],
    /// Unit quaternion `[w, x, y, z]`.
    pub orientation: [f64; 4],
}

/// Neutral spatial state of the kinematic model for one observation point
/// (a control tick).
///
/// Derived from the SAME [`FKResult`] that produces the tick's TCP: one
/// kinematic evaluation produces both. See
/// `docs/system/architecture/spatial-state-contract.md`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SpatialState {
    pub frames: Vec<FramePose>,
}

impl SpatialState {
    /// Project an [`FKResult`] into the neutral spatial state.
    ///
    /// Frames are sorted deterministically (`world` first, then ascending
    /// numeric id) so the wire payload is stable across evaluations — the FK
    /// result stores poses in a hash map with unspecified iteration order.
    pub fn from_fk(fk: &FKResult) -> Self {
        let mut frames: Vec<FramePose> = fk
            .frames()
            .filter_map(|id| fk.pose(id).map(|pose| (id, pose)))
            .map(|(id, pose)| {
                let transform = pose.transform();
                let q = transform.rotation.inner();
                FramePose {
                    frame: *id,
                    position: [
                        transform.translation.x,
                        transform.translation.y,
                        transform.translation.z,
                    ],
                    orientation: [q.w, q.x, q.y, q.z],
                }
            })
            .collect();

        frames.sort_by_key(|f| match f.frame {
            FrameId::World => (0u8, 0u64),
            FrameId::Id(id) => (1u8, id),
        });

        Self { frames }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::forward::ForwardKinematics;
    use crate::models::planar_2r::Planar2RSpec;

    #[test]
    fn from_fk_carries_every_frame_pose() {
        let chain = Planar2RSpec::ideal().build();
        let fk = ForwardKinematics::new(chain.clone()).evaluate(&[0.4, -0.3]);

        let spatial = SpatialState::from_fk(&fk);

        assert_eq!(spatial.frames.len(), fk.frames().count());
        for pose in &spatial.frames {
            let expected = fk.pose(&pose.frame).expect("frame must exist in FK");
            let t = expected.transform();
            assert!((pose.position[0] - t.translation.x).abs() < 1e-12);
            assert!((pose.position[1] - t.translation.y).abs() < 1e-12);
            assert!((pose.position[2] - t.translation.z).abs() < 1e-12);
        }
    }

    #[test]
    fn from_fk_is_deterministic_and_world_first() {
        let chain = Planar2RSpec::ideal().build();
        let fk = ForwardKinematics::new(chain).evaluate(&[0.1, 0.2]);

        let a = SpatialState::from_fk(&fk);
        let b = SpatialState::from_fk(&fk);

        assert_eq!(a, b, "projection must be deterministic");
        assert_eq!(a.frames[0].frame, FrameId::World);
        let ids: Vec<(u8, u64)> = a
            .frames
            .iter()
            .map(|f| match f.frame {
                FrameId::World => (0, 0),
                FrameId::Id(id) => (1, id),
            })
            .collect();
        let mut sorted = ids.clone();
        sorted.sort();
        assert_eq!(ids, sorted, "frames must be ordered world-first then by id");
    }
}
