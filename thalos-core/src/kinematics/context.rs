use std::sync::Arc;

use serde::{Deserialize, Serialize};

use crate::kinematics::forward::ForwardKinematics;
use crate::robot::serial_chain::SerialChain;
use crate::robot::tool_frame::ToolFrame;

/// World-frame TCP pose produced by the shared forward-kinematics authority.
///
/// `orientation` is a unit quaternion in `[w, x, y, z]` order.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TcpPose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// The shared kinematic authority: `joints → TCP`.
///
/// It CONSUMES the domain FK (`crate::kinematics::forward::ForwardKinematics`)
/// — it never reimplements kinematics — and guarantees the invariant:
///
/// > same robot + same joints + same ToolFrame ⇒ same TCP,
///
/// for every runner (simulation, telemetry, physical). Runners share one
/// `Arc<KinematicContext>` instead of each resolving the model on its own.
#[derive(Clone)]
pub struct KinematicContext {
    fk: ForwardKinematics,
    tool: ToolFrame,
}

impl KinematicContext {
    pub fn new(chain: SerialChain, tool: ToolFrame) -> Self {
        Self {
            fk: ForwardKinematics::new(chain),
            tool,
        }
    }

    /// Build the context for a chain with the TCP at its end-effector frame
    /// (identity offset) — the same fallback the UI uses when no calibrated
    /// tool frame is selected.
    pub fn at_end_effector(chain: SerialChain) -> Self {
        let tool = ToolFrame::identity(*chain.end_effector());
        Self::new(chain, tool)
    }

    /// Resolve the world TCP pose for a joint configuration.
    ///
    /// Returns `None` when the configuration does not match the chain DOF or
    /// the TCP base frame is absent from the FK result — the caller renders `—`
    /// rather than inventing a pose.
    pub fn tcp_pose(&self, joints: &[f64]) -> Option<TcpPose> {
        if joints.len() != self.fk.robot().dof_count() {
            return None;
        }
        let result = self.fk.evaluate(joints);
        let pose = result.tcp_pose(&self.tool)?;
        let transform = pose.transform();
        let q = transform.rotation.inner();
        Some(TcpPose {
            position: [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            orientation: [q.w, q.x, q.y, q.z],
        })
    }
}

impl std::fmt::Debug for KinematicContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KinematicContext")
            .field("dof", &self.fk.robot().dof_count())
            .field("tool", &self.tool.base_frame)
            .finish()
    }
}

/// Shared handle to the runtime's kinematic authority.
pub type SharedKinematics = Arc<KinematicContext>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::planar_2r::Planar2RSpec;

    #[test]
    fn tcp_pose_matches_fk_end_effector_for_identity_tool() {
        let chain = Planar2RSpec::ideal().build();
        let expected_fk = ForwardKinematics::new(chain.clone());
        let q = [0.4, -0.3];

        let expected = expected_fk
            .evaluate(&q)
            .ee_position()
            .expect("planar 2R exposes an end-effector pose");

        let ctx = KinematicContext::at_end_effector(chain);
        let tcp = ctx.tcp_pose(&q).expect("TCP must resolve");

        assert!((tcp.position[0] - expected.x).abs() < 1e-9);
        assert!((tcp.position[1] - expected.y).abs() < 1e-9);
        assert!((tcp.position[2] - expected.z).abs() < 1e-9);
        // Orientation is a unit quaternion ([w, x, y, z]).
        let norm_sq: f64 = tcp.orientation.iter().map(|c| c * c).sum();
        assert!((norm_sq - 1.0).abs() < 1e-9);
    }

    #[test]
    fn tcp_pose_is_none_when_dof_does_not_match() {
        let chain = Planar2RSpec::ideal().build();
        let ctx = KinematicContext::at_end_effector(chain);
        assert!(ctx.tcp_pose(&[0.0]).is_none());
        assert!(ctx.tcp_pose(&[]).is_none());
    }

    #[test]
    fn same_chain_joints_and_tool_produce_the_same_tcp() {
        let chain = Planar2RSpec::ideal().build();
        let ctx = KinematicContext::at_end_effector(chain);
        let q = [0.1, 0.2];
        assert_eq!(ctx.tcp_pose(&q), ctx.tcp_pose(&q));
    }
}
