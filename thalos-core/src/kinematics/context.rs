use std::sync::Arc;

use serde::{Deserialize, Serialize};

use crate::kinematics::forward::ForwardKinematics;
use crate::kinematics::forward::result::FKResult;
use crate::robot::serial_chain::SerialChain;
use crate::robot::tool_frame::ToolFrame;
use crate::spatial::pose::Pose;

/// World-frame TCP pose produced by the shared forward-kinematics authority.
///
/// `orientation` is a unit quaternion in `[w, x, y, z]` order.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TcpPose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

impl TcpPose {
    fn from_pose(pose: &Pose) -> Self {
        let transform = pose.transform();
        let q = transform.rotation.inner();
        Self {
            position: [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            orientation: [q.w, q.x, q.y, q.z],
        }
    }
}

/// Result of a SINGLE kinematic evaluation: the full FK result plus the TCP
/// pose derived from it.
///
/// Callers that need both the TCP and the neutral spatial state (the full set
/// of frame poses) resolve once instead of evaluating FK twice. `tcp` is
/// `None` when the tool frame cannot be resolved against the FK result, even
/// though the frame poses are still valid.
#[derive(Debug, Clone)]
pub struct KinematicState {
    pub fk: FKResult,
    pub tcp: Option<TcpPose>,
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
        self.resolve(joints)?.tcp
    }

    /// Evaluate kinematics ONCE and return both the full FK result and the TCP
    /// pose derived from it.
    ///
    /// Returns `None` when the configuration does not match the chain DOF; a
    /// mismatched configuration has no meaningful spatial state either.
    pub fn resolve(&self, joints: &[f64]) -> Option<KinematicState> {
        if joints.len() != self.fk.robot().dof_count() {
            return None;
        }
        let fk = self.fk.evaluate(joints);
        let tcp = fk
            .tcp_pose(&self.tool)
            .map(|pose| TcpPose::from_pose(&pose));
        Some(KinematicState { fk, tcp })
    }

    /// The kinematic chain this authority evaluates.
    pub fn chain(&self) -> &SerialChain {
        self.fk.robot()
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

    #[test]
    fn resolve_returns_one_evaluation_with_fk_and_tcp() {
        let chain = Planar2RSpec::ideal().build();
        let q = [0.4, -0.3];
        let ctx = KinematicContext::at_end_effector(chain.clone());

        let state = ctx.resolve(&q).expect("DOF matches");

        // The TCP comes from the SAME evaluation that produced `fk`: rebuild
        // the expected FK from the same joints and compare the end-effector.
        let expected_fk = ForwardKinematics::new(chain).evaluate(&q);
        assert_eq!(state.fk.ee_position(), expected_fk.ee_position());

        let tcp = state.tcp.expect("identity tool resolves");
        assert_eq!(ctx.tcp_pose(&q), Some(tcp));
    }

    #[test]
    fn resolve_is_none_on_dof_mismatch() {
        let chain = Planar2RSpec::ideal().build();
        let ctx = KinematicContext::at_end_effector(chain);
        assert!(ctx.resolve(&[0.0]).is_none());
        assert!(ctx.resolve(&[]).is_none());
    }

    #[test]
    fn chain_is_exposed() {
        let chain = Planar2RSpec::ideal().build();
        let dof = chain.dof_count();
        let ctx = KinematicContext::at_end_effector(chain);
        assert_eq!(ctx.chain().dof_count(), dof);
    }
}
