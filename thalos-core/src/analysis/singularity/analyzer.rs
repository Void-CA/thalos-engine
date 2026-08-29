//! Orchestrator that iterates a [`Workspace`] and produces a
//! [`SingularityAnalysis`].

use crate::analysis::workspace::Workspace;
use crate::kinematics::jacobian::{JacobianSolver, SingularityReport};

use super::config::SingularityConfig;
use super::report::{SingularityAnalysis, SingularitySample, SingularityState};

/// Stateless analyzer that runs singularity analysis over every sample
pub struct SingularityAnalyzer;

impl SingularityAnalyzer {
    pub fn analyze(
        workspace: &Workspace,
        jacobian_solver: &impl JacobianSolver,
        config: &SingularityConfig,
    ) -> SingularityAnalysis {
        let samples: Vec<SingularitySample> = workspace
            .samples()
            .iter()
            .map(|ws_sample| {
                let q = &ws_sample.q;
                let position = ws_sample.position;

                // Step 1: evaluate J(q)
                let jacobian = jacobian_solver.evaluate(q);

                // Step 2: atomic SVD-based analysis
                let analysis = SingularityReport::analyze(&jacobian);

                // Step 3: classify
                let state = classify(&analysis, config);

                SingularitySample {
                    q: q.clone(),
                    position,
                    analysis,
                    state,
                }
            })
            .collect();

        SingularityAnalysis::from_samples(samples)
    }
}

/// Classify a per-Jacobian report into a state.
fn classify(report: &SingularityReport, config: &SingularityConfig) -> SingularityState {
    if report.condition_number.is_infinite() || report.rank < 2 {
        return SingularityState::Singular;
    }
    if report.condition_number > config.near_singular_condition_threshold {
        return SingularityState::NearSingular;
    }
    SingularityState::Normal
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::workspace::{WorkspaceConfig, WorkspaceSampler};
    use crate::kinematics::forward::ForwardKinematics;
    use crate::kinematics::jacobian::GeometricJacobian;
    use crate::robot::serial_chain::SerialChain;
    use thalos_math::Vector3;

    use rand::SeedableRng;
    use rand::rngs::StdRng;

    /// Build a simple planar 2R chain for testing.
    fn build_planar_2r() -> (SerialChain, GeometricJacobian) {
        use crate::robot::builder::SerialChainBuilder;
        use crate::robot::joint::*;
        use crate::robot::link::Link;
        use crate::robot::segment::Segment;
        use crate::spatial::frame::FrameId;
        use std::f64::consts::PI;
        use thalos_math::Transform3D;
        use thalos_math::UnitVector3;

        let mut builder = SerialChainBuilder::new();

        let shoulder = builder.create_frame("shoulder");
        let ee = builder.create_frame("ee");

        // Segment 1: World → shoulder
        let joint1 = JointType::Revolute(RevoluteJoint::new(
            0,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        let link1 = Link::new(
            0,
            Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
        );
        builder.add_segment(Segment::new(
            FrameId::World,
            shoulder.clone(),
            joint1,
            link1,
        ));

        // Segment 2: shoulder → ee
        let joint2 = JointType::Revolute(RevoluteJoint::new(
            1,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        let link2 = Link::new(
            1,
            Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
        );
        builder.add_segment(Segment::new(shoulder, ee.clone(), joint2, link2));

        builder.set_end_effector(ee.clone());

        let chain = builder.build().expect("planar 2R: builder failed");
        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, ee);

        (chain, jac)
    }

    #[test]
    fn analyze_empty_workspace() {
        // An empty workspace can't be constructed (Workspace::from_samples rejects empty),
        // but if it had 0 samples the analysis should handle it gracefully.
        // This test verifies the 0-sample edge case via SingularityAnalysis::from_samples.
        let analysis = SingularityAnalysis::from_samples(vec![]);
        assert_eq!(analysis.metrics.total_samples, 0);
        assert!(analysis.samples.is_empty());
    }

    #[test]
    fn analyze_planar_2r_basic() {
        let (chain, jac) = build_planar_2r();
        let sampler = WorkspaceSampler;
        let config = WorkspaceConfig {
            samples: 100,
            seed: 42,
            tolerance: 0.001,
        };
        let mut rng = StdRng::seed_from_u64(config.seed);
        let workspace = sampler
            .sample(&chain, config, &mut rng)
            .expect("sampling failed");

        let singularity_config = SingularityConfig::default();
        let analysis = SingularityAnalyzer::analyze(&workspace, &jac, &singularity_config);

        // Must have same number of samples as workspace
        assert_eq!(analysis.metrics.total_samples, 100);
        assert_eq!(analysis.samples.len(), 100);

        // Every sample must have populated analysis
        for sample in &analysis.samples {
            assert_eq!(sample.q.len(), 2);
            assert!(!sample.analysis.singular_values.is_empty());
        }

        // Some samples should reach each state (planar 2R has known singularities)
        println!(
            "Planar 2R: {} singular, {} near, {} normal",
            analysis.metrics.singular_count,
            analysis.metrics.near_singular_count,
            analysis.metrics.normal_count,
        );

        // With random sampling, we expect at least some normal samples
        assert!(
            analysis.metrics.normal_count > 0,
            "Expected at least some normal samples, got 0"
        );
    }

    #[test]
    fn detect_known_planar_2r_singularity() {
        let (_chain, jac) = build_planar_2r();

        let q = vec![0.0, 0.0];

        let jacobian = jac.evaluate(&q);
        let report = SingularityReport::analyze(&jacobian);

        println!("rank = {}", report.rank);
        println!("cond = {}", report.condition_number);
        println!("sv = {:?}", report.singular_values);

        assert_eq!(report.rank, 1);

        assert!(
            report.condition_number.is_infinite(),
            "expected infinite condition number, got {}",
            report.condition_number
        );

        let state = classify(&report, &SingularityConfig::default());

        assert_eq!(state, SingularityState::Singular);
    }

    #[test]
    fn analyze_icebot_urdf_pipeline() {
        // Load the icebot URDF and verify the full pipeline
        // (FK → workspace sampling → singularity analysis) does not panic.
        let src = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/icebot.urdf"
        ));
        let robot =
            thalos_importer::import_urdf(src).expect("icebot URDF should parse");
        let chain =
            crate::robot::adapter::auto(&robot).expect("icebot should produce a valid chain");

        // Verify the chain structure
        // Icebot has 4 actuated joints + 1 fixed joint (tcp_joint for tool0 frame)
        assert_eq!(
            chain.segments.len(),
            5,
            "icebot has 5 segments (4 actuated + 1 fixed)"
        );
        assert_eq!(chain.dof_count(), 4, "icebot has 4 DOF (3 rev + 1 prism)");

        // FK with zero config
        let fk = ForwardKinematics::new(chain.clone());
        let q = vec![0.0; chain.dof_count()];
        let fk_result = fk.evaluate(&q);
        assert!(
            fk_result.ee_position().is_some(),
            "FK should produce an EE position"
        );

        // Workspace sampling
        let config = WorkspaceConfig {
            samples: 50,
            seed: 42,
            tolerance: 0.001,
        };
        let mut rng = StdRng::seed_from_u64(config.seed);
        let ws = WorkspaceSampler
            .sample(&chain, config, &mut rng)
            .expect("workspace sampling should succeed");
        assert_eq!(ws.samples().len(), 50);

        // Verify all q vectors have correct length
        for s in ws.samples() {
            assert_eq!(s.q.len(), 4, "each sample q should have 4 elements (DOF)");
        }

        // Singularity analysis
        let jac = GeometricJacobian::new(
            ForwardKinematics::new(chain.clone()),
            chain.end_effector.clone(),
        );
        let singularity_config = SingularityConfig::default();
        let analysis = SingularityAnalyzer::analyze(&ws, &jac, &singularity_config);
        assert_eq!(analysis.samples.len(), 50);
        assert_eq!(analysis.metrics.total_samples, 50);
    }
}
