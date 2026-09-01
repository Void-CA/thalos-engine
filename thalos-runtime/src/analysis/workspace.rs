use std::sync::Arc;

use rand::SeedableRng;
use rand::rngs::StdRng;

use thalos_engine::core::analysis::workspace::{
    Workspace, WorkspaceConfig, WorkspaceError, sampler::WorkspaceSampler,
};
use thalos_engine::core::models::{RobotModel, RobotRegistry};
use thalos_engine::core::robot::serial_chain::SerialChain;
use thalos_engine::core::robot::tool_frame::ToolFrame;
use thalos_engine::math::Vector3;

use crate::error::RuntimeError;

/// Stateless service for workspace sampling and reachability queries.
pub struct WorkspaceService;

impl WorkspaceService {
    pub fn sample(
        model: RobotModel,
        config: WorkspaceConfig,
    ) -> Result<Arc<Workspace>, RuntimeError> {
        if config.samples == 0 {
            return Err(RuntimeError::Workspace(WorkspaceError::InvalidSampleCount(
                0,
            )));
        }

        let chain = RobotRegistry::create_default(model);
        Self::sample_from_chain(&chain, config)
    }

    /// Sample workspace using an already-built chain (e.g. from the active robot).
    pub fn sample_from_chain(
        chain: &SerialChain,
        config: WorkspaceConfig,
    ) -> Result<Arc<Workspace>, RuntimeError> {
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
    ) -> Result<Arc<Workspace>, RuntimeError> {
        if config.samples == 0 {
            return Err(RuntimeError::Workspace(WorkspaceError::InvalidSampleCount(
                0,
            )));
        }

        let mut rng = StdRng::seed_from_u64(config.seed);

        let ws = WorkspaceSampler
            .sample_with_tcp(chain, config, tcp, &mut rng)
            .map_err(RuntimeError::Workspace)?;

        Ok(Arc::new(ws))
    }

    /// Check whether a point is reachable within `tolerance`.
    ///
    /// Pure delegation to `Workspace::is_reachable`.
    pub fn query(
        workspace: &Workspace,
        point: &Vector3,
        tolerance: f64,
    ) -> Result<thalos_engine::core::analysis::workspace::Reachability, WorkspaceError> {
        workspace.is_reachable(point, tolerance)
    }
}
