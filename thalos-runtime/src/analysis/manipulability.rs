use rand::SeedableRng;
use rand::rngs::StdRng;

use thalos_engine::core::analysis::manipulability::{ManipulabilityAnalysis, ManipulabilityAnalyzer};
use thalos_engine::core::analysis::workspace::{
    WorkspaceConfig, WorkspaceError, sampler::WorkspaceSampler,
};
use thalos_engine::core::kinematics::forward::ForwardKinematics;
use thalos_engine::core::kinematics::jacobian::GeometricJacobian;
use thalos_engine::core::models::{RobotModel, RobotRegistry};
use thalos_engine::core::robot::serial_chain::SerialChain;
use thalos_engine::core::robot::tool_frame::ToolFrame;

use crate::error::RuntimeError;

pub struct ManipulabilityService;

impl ManipulabilityService {
    pub fn analyze(
        model: RobotModel,
        config: WorkspaceConfig,
    ) -> Result<ManipulabilityAnalysis, RuntimeError> {
        if config.samples == 0 {
            return Err(RuntimeError::Workspace(WorkspaceError::InvalidSampleCount(
                0,
            )));
        }

        let chain = RobotRegistry::create_default(model);
        Self::analyze_from_chain(&chain, config)
    }

    pub fn analyze_from_chain(
        chain: &SerialChain,
        config: WorkspaceConfig,
    ) -> Result<ManipulabilityAnalysis, RuntimeError> {
        Self::analyze_from_chain_with_tcp(chain, config, None)
    }

    /// Analyze manipulability with an optional TCP frame.
    ///
    /// If `tcp` is `Some`, the Jacobian references the TCP position.
    /// If `None`, references the flange (end effector).
    pub fn analyze_from_chain_with_tcp(
        chain: &SerialChain,
        config: WorkspaceConfig,
        tcp: Option<&ToolFrame>,
    ) -> Result<ManipulabilityAnalysis, RuntimeError> {
        if config.samples == 0 {
            return Err(RuntimeError::Workspace(WorkspaceError::InvalidSampleCount(
                0,
            )));
        }

        let mut rng = StdRng::seed_from_u64(config.seed);

        let ws = WorkspaceSampler
            .sample_with_tcp(chain, config, tcp, &mut rng)
            .map_err(RuntimeError::Workspace)?;

        let fk = ForwardKinematics::new(chain.clone());
        let jac = if let Some(tcp) = tcp {
            GeometricJacobian::with_tcp(fk, tcp.clone())
        } else {
            GeometricJacobian::new(fk, chain.end_effector.clone())
        };

        let analysis = ManipulabilityAnalyzer::analyze(&ws, &jac, chain);

        Ok(analysis)
    }
}
