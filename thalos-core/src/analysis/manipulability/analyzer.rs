use crate::analysis::workspace::Workspace;
use crate::kinematics::jacobian::{JacobianSolver, ManipulabilityReport, SingularityReport};
use crate::robot::serial_chain::SerialChain;

use super::report::{ManipulabilityAnalysis, ManipulabilitySample};

/// Stateless analyzer that derives manipulability for every sample
/// in a [`Workspace`].
///
/// Internally calls `SingularityReport::analyze` (SVD on the ORIGINAL
/// Jacobian) and derives the raw `ManipulabilityReport` from its singular
/// values, then re-SVDs the scale-normalized linear Jacobian `J'` for the
/// normalized measure + grade (design "Where re-SVD on J' happens").
pub struct ManipulabilityAnalyzer;

impl ManipulabilityAnalyzer {
    pub fn analyze(
        workspace: &Workspace,
        jacobian_solver: &impl JacobianSolver,
        chain: &SerialChain,
    ) -> ManipulabilityAnalysis {
        let samples: Vec<ManipulabilitySample> = workspace
            .samples()
            .iter()
            .map(|ws_sample| {
                let q = &ws_sample.q;
                let jacobian = jacobian_solver.evaluate(q);
                let singularity = SingularityReport::analyze(&jacobian);
                let manipulability = ManipulabilityReport::compute_with_normalization(
                    &singularity,
                    &jacobian,
                    chain,
                );

                ManipulabilitySample {
                    q: q.clone(),
                    position: ws_sample.position,
                    singularity,
                    manipulability,
                    // Staged by `ManipulabilityAnalysis::from_samples` once
                    // the FULL sample set is collected (design
                    // "relative_manipulability" needs the distribution).
                    relative_manipulability: 0.0,
                }
            })
            .collect();

        ManipulabilityAnalysis::from_samples(
            samples,
            crate::robot::scale::manipulability_reference_dimension(chain),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::workspace::{WorkspaceConfig, WorkspaceSampler};
    use crate::kinematics::forward::ForwardKinematics;
    use crate::kinematics::jacobian::GeometricJacobian;
    use crate::robot::builder::SerialChainBuilder;
    use crate::robot::joint::*;
    use crate::robot::link::Link;
    use crate::robot::segment::Segment;
    use crate::robot::serial_chain::SerialChain;
    use crate::spatial::frame::FrameId;
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use std::f64::consts::PI;
    use thalos_math::{Transform3D, UnitVector3, Vector3};

    fn build_planar_2r() -> (SerialChain, GeometricJacobian) {
        let mut builder = SerialChainBuilder::new();
        let shoulder = builder.create_frame("shoulder");
        let ee = builder.create_frame("ee");

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
        let chain = builder.build().expect("planar 2R");
        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, ee);
        (chain, jac)
    }

    #[test]
    fn analyze_planar_2r() {
        let (chain, jac) = build_planar_2r();
        let mut rng = StdRng::seed_from_u64(42);
        let ws = WorkspaceSampler
            .sample(
                &chain,
                WorkspaceConfig {
                    samples: 100,
                    seed: 42,
                    tolerance: 1e-3,
                },
                &mut rng,
            )
            .expect("sampling failed");

        let analysis = ManipulabilityAnalyzer::analyze(&ws, &jac, &chain);
        assert_eq!(analysis.metrics.total_samples, 100);
        assert_eq!(analysis.samples.len(), 100);

        // All samples should have valid manipulability
        for s in &analysis.samples {
            assert!(s.manipulability.yoshikawa >= 0.0);
            assert!(s.manipulability.isotropy >= 0.0);
            assert!(s.manipulability.isotropy <= 1.0);
            // Task 2.1: normalized + grade populated per sample
            assert!(s.manipulability.manipulability_grade.is_some());
            assert!(s.manipulability.normalized_yoshikawa.is_finite());
            // Design "relative_manipulability": every sample is staged
            // against the robot's own distribution and clamped to [0, 1].
            assert!(
                (0.0..=1.0).contains(&s.relative_manipulability),
                "relative score {} must live in [0, 1]",
                s.relative_manipulability
            );
        }

        // The 100-sample planar 2R spans a real distribution (near-singular
        // configurations at joint limits vs fully dexterous ones) — the
        // relative metric must DISCRIMINATE, not collapse to a constant.
        assert!(
            analysis
                .samples
                .iter()
                .any(|s| s.relative_manipulability < 1.0),
            "a real workspace must produce relative scores below 1.0"
        );
        assert!(
            analysis.metrics.p05 <= analysis.metrics.p50
                && analysis.metrics.p50 <= analysis.metrics.p95,
            "percentiles must be ordered P05 ≤ P50 ≤ P95"
        );
        assert!(
            (0.0..=1.0).contains(&analysis.metrics.avg_relative),
            "avg_relative must stay in [0, 1]"
        );

        println!(
            "Planar 2R: avg_yoshikawa={:.4}, avg_isotropy={:.4}, p05={:.4}, p95={:.4}, avg_relative={:.4}",
            analysis.metrics.avg_yoshikawa,
            analysis.metrics.avg_isotropy,
            analysis.metrics.p05,
            analysis.metrics.p95,
            analysis.metrics.avg_relative,
        );
    }

    #[test]
    fn analyze_scara_populates_normalized_and_grade() {
        // Task 2.1 integration: SCARA canonical through the analyzer must
        // carry the normalized measure and a classified grade on every
        // sample, while the raw metrics remain untouched (raw preservation).
        use crate::models::scara::ScaraSpec;

        let chain = ScaraSpec::canonical().build();
        let ee = chain.end_effector.clone();
        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, ee);

        let mut rng = StdRng::seed_from_u64(7);
        let ws = WorkspaceSampler
            .sample(
                &chain,
                WorkspaceConfig {
                    samples: 80,
                    seed: 7,
                    tolerance: 1e-3,
                },
                &mut rng,
            )
            .expect("sampling failed");

        let analysis = ManipulabilityAnalyzer::analyze(&ws, &jac, &chain);
        assert_eq!(analysis.samples.len(), 80);

        for s in &analysis.samples {
            let report = &s.manipulability;
            assert!(
                report.manipulability_grade.is_some(),
                "grade must be classified"
            );
            assert!(
                report.normalized_yoshikawa.is_finite() && report.normalized_yoshikawa >= 0.0,
                "normalized must be finite and non-negative"
            );
        }

        // Spot-check raw preservation on the first sample: raw yoshikawa must
        // equal the product of the ORIGINAL singular values.
        let first = &analysis.samples[0];
        let raw_product: f64 = first.singularity.singular_values.iter().product();
        assert!(
            (first.manipulability.yoshikawa - raw_product).abs() < 1e-12,
            "raw yoshikawa must stay the product of the original singular values"
        );
    }
}
