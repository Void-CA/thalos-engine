//! Monte Carlo sampler for joint configurations.
//!
//! Given a `SerialChain` and a `WorkspaceConfig`, the sampler produces a
//! `Workspace` of `samples` configurations uniformly distributed over the
//! robot's joint limits (D6: sequential, no parallelism). The RNG is
//! injected so determinism (R1) is testable.

use std::f64::consts::PI;

use rand::{Rng, SeedableRng};

use crate::robot::joint::JointLimits;
use crate::robot::serial_chain::SerialChain;
use crate::robot::tool_frame::ToolFrame;

use super::WorkspaceConfig;
use super::error::WorkspaceError;
use super::types::WorkspaceSample;
use super::workspace::Workspace;

pub struct WorkspaceSampler;

impl WorkspaceSampler {
    /// Sample `config.samples` joint configurations uniformly within the
    /// joint limits, evaluate FK on each, and build a `Workspace`.
    ///
    /// If `tcp` is `Some`, samples the TCP position. If `None`, samples
    /// the flange (end effector) position for backward compatibility.
    ///
    /// The RNG is injected by the caller so tests can fix the seed
    /// and assert determinism (R1).
    pub fn sample<R: Rng + SeedableRng>(
        &self,
        chain: &SerialChain,
        config: WorkspaceConfig,
        rng: &mut R,
    ) -> Result<Workspace, WorkspaceError> {
        self.sample_with_tcp(chain, config, None, rng)
    }

    /// Sample with an optional TCP frame.
    ///
    /// If `tcp` is `Some`, samples the TCP position. If `None`, samples
    /// the flange (end effector) position.
    pub fn sample_with_tcp<R: Rng + SeedableRng>(
        &self,
        chain: &SerialChain,
        config: WorkspaceConfig,
        tcp: Option<&ToolFrame>,
        rng: &mut R,
    ) -> Result<Workspace, WorkspaceError> {
        if config.samples == 0 {
            return Err(WorkspaceError::InvalidSampleCount(0));
        }

        let mut samples = Vec::with_capacity(config.samples);

        let n_dof: usize = chain.segments.iter().map(|s| s.joint.dof()).sum();

        for _ in 0..config.samples {
            // Sample a random q within each joint's limits (R6).
            // Fixed joints no se samplean (no contribuyen DOF).
            let mut q = Vec::with_capacity(n_dof);
            for segment in &chain.segments {
                if segment.joint.dof() == 0 {
                    continue;
                }
                let limits = segment.joint.limits();
                let q_i = uniform_within(rng, limits);
                q.push(q_i);
            }

            // FK (R2: position == FK(q).ee_position() or FK(q).tcp_position(tcp)).
            // We re-construct FK per sample so the caller's chain is
            // untouched (D14 immutability of input).
            let fk = crate::kinematics::forward::ForwardKinematics::new(chain.clone());
            let result = fk.evaluate(&q);

            let position = if let Some(tcp) = tcp {
                result
                    .tcp_position(tcp)
                    .ok_or_else(|| WorkspaceError::EmptyWorkspace)?
            } else {
                result
                    .ee_position()
                    .ok_or_else(|| WorkspaceError::EmptyWorkspace)?
            };

            samples.push(WorkspaceSample { q, position });
        }

        Workspace::from_samples(samples)
    }
}

fn uniform_within<R: Rng>(rng: &mut R, limits: JointLimits) -> f64 {
    if !limits.enabled {
        // Joint has no mechanical bounds (e.g. URDF continuous without
        // an explicit <limit>). Sample one full rotation for workspace
        // visualization purposes.
        return -PI + rand::Rng::r#gen::<f64>(rng) * (2.0 * PI);
    }
    limits.min + rand::Rng::r#gen::<f64>(rng) * (limits.max - limits.min)
}
