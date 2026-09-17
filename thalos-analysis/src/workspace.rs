use std::sync::Arc;

use rand::SeedableRng;
use rand::rngs::StdRng;

use thalos_core::analysis::workspace::{
    Reachability, Workspace, WorkspaceConfig, WorkspaceError, sampler::WorkspaceSampler,
};
use thalos_core::models::{RobotModel, RobotRegistry};
use thalos_core::robot::serial_chain::SerialChain;
use thalos_core::robot::tool_frame::ToolFrame;
use thalos_math::Vector3;

/// Stateless service for workspace sampling and reachability queries.
pub struct WorkspaceService;

impl WorkspaceService {
    pub fn sample(
        model: RobotModel,
        config: WorkspaceConfig,
    ) -> Result<Arc<Workspace>, WorkspaceError> {
        if config.samples == 0 {
            return Err(WorkspaceError::InvalidSampleCount(0));
        }

        let chain = RobotRegistry::create_default(model);
        Self::sample_from_chain(&chain, config)
    }

    /// Sample workspace using an already-built chain (e.g. from the active robot).
    pub fn sample_from_chain(
        chain: &SerialChain,
        config: WorkspaceConfig,
    ) -> Result<Arc<Workspace>, WorkspaceError> {
        Self::sample_from_chain_with_tcp(chain, config, None)
    }

    /// Sample workspace with an optional TCP frame.
    ///
    /// If `tcp` is `Some`, samples the TCP position. If `None`, samples
    /// the flange (end effector) position.
    pub fn sample_from_chain_with_tcp(
        chain: &SerialChain,
        config: WorkspaceConfig,
        tcp: Option<&ToolFrame>,
    ) -> Result<Arc<Workspace>, WorkspaceError> {
        if config.samples == 0 {
            return Err(WorkspaceError::InvalidSampleCount(0));
        }

        let mut rng = StdRng::seed_from_u64(config.seed);

        let ws = WorkspaceSampler.sample_with_tcp(chain, config, tcp, &mut rng)?;

        Ok(Arc::new(ws))
    }

    /// Check whether a point is reachable within `tolerance`.
    ///
    /// Pure delegation to `Workspace::is_reachable`.
    pub fn query(
        workspace: &Workspace,
        point: &Vector3,
        tolerance: f64,
    ) -> Result<Reachability, WorkspaceError> {
        workspace.is_reachable(point, tolerance)
    }
}
