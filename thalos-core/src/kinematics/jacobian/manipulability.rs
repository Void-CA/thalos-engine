use super::singularity::SingularityReport;
use crate::kinematics::jacobian::Jacobian;
use crate::robot::joint::JointKind;
use crate::robot::scale::manipulability_reference_dimension;
use crate::robot::serial_chain::SerialChain;
use serde::{Deserialize, Serialize};
use thalos_math::DynamicMatrix;

/// Constant dimensionless thresholds for [`ManipulabilityGrade`] classification.
///
/// Same values for EVERY robot regardless of scale — the normalized metric is
/// dimensionless, so a single robot-independent partition is the point of the
/// change (spec manipulability-normalization "Manipulability Grade
/// Classification").
///
/// Calibration origin (task 6.1, re-calibrated after the moving-only L_ref
/// remediation): SCARA canonical (base 0.5, a1 1.0, a2 0.8, moving-only
/// `manipulability_reference_dimension` = 1.8 — fixed base excluded),
/// workspace-sampled with seed 42 / 5000 samples. The raw reference
/// partition (0.3 / 0.5) maps to disjoint normalized ranges:
/// raw-LOW ∈ [0.000133, 0.092564], raw-MED ∈ [0.092826, 0.154301],
/// raw-HIGH ∈ [0.154341, 0.246914]. The gaps are stable across seeds
/// 42/1/7/123, so `T_LOW = 0.0926` and `T_HIGH = 0.15433` reproduce the
/// SCARA canonical grade partition point-to-point (oracle check 1). The
/// values were NOT fit to force any other robot's outcome: the icebot
/// (moving-only L_ref = 0.385, short arms + wrist on the tool axis) truly
/// maxes at normalized ≈ 0.084 < T_LOW, so it stays all-LOW by design — the
/// fixed-terminal-joint "dead weight" is gone, but its proportions remain
/// genuinely below the SCARA-calibrated partition (verified by brute-force
/// grid, not the ratio³ simulation).
pub const T_LOW: f64 = 0.0926;
pub const T_HIGH: f64 = 0.15433;

/// Categorical manipulability grade over `normalized_yoshikawa`.
///
/// `Low < T_LOW ≤ Medium < T_HIGH ≤ High`. Classified by the backend;
/// `None` on the wire means "legacy payload" and triggers the frontend
/// fallback (design "Grade as Option for presence signal").
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ManipulabilityGrade {
    Low,
    Medium,
    High,
}

impl ManipulabilityGrade {
    /// Wire string: `"low" | "medium" | "high"`.
    pub fn as_str(&self) -> &'static str {
        match self {
            ManipulabilityGrade::Low => "low",
            ManipulabilityGrade::Medium => "medium",
            ManipulabilityGrade::High => "high",
        }
    }
}

/// Manipulability metrics derived from the singular values of a Jacobian.
///
/// Zero-cost derivation from [`SingularityReport`] — no additional SVD needed.
/// The singular values are already computed; this just interprets them.
#[derive(Debug, Clone, Copy, Default)]
pub struct ManipulabilityReport {
    /// Yoshikawa manipulability measure: `w = ∏ σᵢ`.
    ///
    /// Product of all significant singular values. Zero when the Jacobian
    /// is rank-deficient (singular). Higher = more dexterous.
    pub yoshikawa: f64,

    /// Isotropy ratio: `σ_min / σ_max` in range [0, 1].
    ///
    /// - 1.0 = perfectly isotropic (equal dexterity in all directions)
    /// - 0.0 = degenerate (at least one direction has zero manipulability)
    pub isotropy: f64,

    /// Dimensionless normalized Yoshikawa measure: `∏ σ′ᵢ` from the SVD of
    /// `J'` — the linear Jacobian with revolute columns scaled by `1/L_ref`
    /// and prismatic columns unscaled (design "Column scaling approach").
    ///
    /// NOT `raw_yoshikawa / L_ref^n` — the SVD mixes column scales and a
    /// uniform divisor would incorrectly scale the prismatic contribution.
    /// `0.0` here is the default for reports computed without normalization
    /// (and a valid normalized value when the robot is singular).
    pub normalized_yoshikawa: f64,

    /// Backend-classified grade over `normalized_yoshikawa`. `None` marks
    /// reports computed without normalization (legacy path → frontend
    /// fallback), NOT a classification result.
    pub manipulability_grade: Option<ManipulabilityGrade>,
}

impl ManipulabilityReport {
    /// Derive manipulability from an already-computed [`SingularityReport`].
    ///
    /// This is O(n) in the number of singular values — essentially free
    /// after the SVD in `SingularityReport::analyze`. Raw path
    /// (behavior-preserving, S1): `normalized_yoshikawa` stays `0.0` and
    /// `manipulability_grade` stays `None` — callers that don't need
    /// normalization (planning, intelligence) keep consuming raw semantics.
    pub fn compute(singularity: &SingularityReport) -> Self {
        let yoshikawa: f64 = singularity.singular_values.iter().product();

        let max_sv = singularity.singular_values.first().copied().unwrap_or(0.0);
        let min_sv = singularity.singular_values.last().copied().unwrap_or(0.0);

        let isotropy = if max_sv > 0.0 { min_sv / max_sv } else { 0.0 };

        Self {
            yoshikawa,
            isotropy,
            ..Default::default()
        }
    }

    /// Compute manipulability WITH normalization: raw fields derive from the
    /// [`SingularityReport`] (computed over the ORIGINAL Jacobian —
    /// behavior-preserving), while `normalized_yoshikawa` comes from a
    /// re-SVD of the scale-normalized linear Jacobian `J'`
    /// (design "Where re-SVD on J' happens").
    ///
    /// The `SingularityReport` and the raw `yoshikawa`/`isotropy` NEVER see
    /// the scaled columns — spec "det_jtj Exclusion from Grade" requires the
    /// singular report to stay on the original Jacobian.
    pub fn compute_with_normalization(
        singularity: &SingularityReport,
        jacobian: &Jacobian,
        chain: &SerialChain,
    ) -> Self {
        let mut report = Self::compute(singularity);

        let l_ref = manipulability_reference_dimension(chain);
        let scaled = scale_jacobian_columns(&jacobian.linear, chain, l_ref);
        // Zero-DOF chain (no moving joints → no Jacobian columns): the
        // empty-product would be 1.0 → High, a fabricated grade for a chain
        // with nothing to measure. The grade stays undefined (None, legacy
        // signal) and the normalized measure stays 0.0.
        if scaled.ncols() == 0 {
            report.normalized_yoshikawa = 0.0;
            report.manipulability_grade = None;
            return report;
        }
        report.normalized_yoshikawa = scaled.singular_values().iter().product();
        report.manipulability_grade = Some(classify(report.normalized_yoshikawa, T_LOW, T_HIGH));

        report
    }
}

/// Classify a dimensionless normalized value against the given thresholds
/// (design data flow: `grade = classify(normalized, T_LOW, T_HIGH)`).
///
/// Partition: `Low < t_low ≤ Medium < t_high ≤ High` — boundaries inclusive
/// on the upper side (exactly `t_low` → medium, exactly `t_high` → high).
pub fn classify(normalized_yoshikawa: f64, t_low: f64, t_high: f64) -> ManipulabilityGrade {
    if normalized_yoshikawa < t_low {
        ManipulabilityGrade::Low
    } else if normalized_yoshikawa < t_high {
        ManipulabilityGrade::Medium
    } else {
        ManipulabilityGrade::High
    }
}

/// Build `J'`: the linear Jacobian with each revolute/continuous column
/// divided by `l_ref` and prismatic columns unscaled (design "Column scaling
/// approach").
///
/// Replays the [`crate::kinematics::jacobian::geom::GeometricJacobian`]
/// iteration order (segments, skip fixed, col++) so the column mapping is
/// identical to the matrix that produced `linear`.
///
/// Guardrail: when `l_ref ≤ ε` (degenerate/broken chain) the matrix is
/// returned unscaled — a finite guard, never a NaN/Inf (spec
/// reference-dimension-fix guardrail; `manipulability_reference_dimension`
/// already floors, this is defense in depth at the normalization point).
pub fn scale_jacobian_columns(
    linear: &DynamicMatrix,
    chain: &SerialChain,
    l_ref: f64,
) -> DynamicMatrix {
    let mut scaled = linear.clone();
    if l_ref <= crate::robot::scale::REFERENCE_DIMENSION_EPS {
        return scaled;
    }

    let mut col = 0;
    for segment in &chain.segments {
        if segment.joint.dof() == 0 {
            continue;
        }
        // The chain may declare more moving joints than the reference frame
        // produces columns for (TCP on an intermediate frame):
        // `GeometricJacobian::evaluate` only iterates segments up to the
        // end-effector frame, so `linear` has fewer columns than the chain's
        // moving joints. Stop when `col` passes the last existing column
        // instead of indexing out of bounds.
        if col >= linear.ncols() {
            break;
        }
        match segment.joint.kind() {
            JointKind::Revolute | JointKind::Continuous => {
                for row in 0..3 {
                    if col < scaled.ncols() {
                        scaled[(row, col)] /= l_ref;
                    }
                }
            }
            JointKind::Prismatic => {
                // Dimensionless unit axis — unscaled by design.
            }
            JointKind::Fixed | JointKind::Floating | JointKind::Planar => {
                // Filtered above (dof() == 0).
            }
        }
        col += 1;
    }

    scaled
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::jacobian::SingularityReport;

    fn make_singularity(sv: Vec<f64>, rank: usize, cond: f64) -> SingularityReport {
        let det_jtj: f64 = sv.iter().map(|s| s * s).product();
        SingularityReport {
            det_jtj,
            condition_number: cond,
            rank,
            singular_values: sv,
        }
    }

    #[test]
    fn isotropic_yoshikawa_product() {
        // σ = [3, 3, 3] → w = 27, isotropy = 1.0
        let sr = make_singularity(vec![3.0, 3.0, 3.0], 3, 1.0);
        let m = ManipulabilityReport::compute(&sr);
        assert!((m.yoshikawa - 27.0).abs() < 1e-12);
        assert!((m.isotropy - 1.0).abs() < 1e-12);
    }

    #[test]
    fn anisotropic() {
        // σ = [5, 1] → w = 5, isotropy = 0.2
        let sr = make_singularity(vec![5.0, 1.0], 2, 5.0);
        let m = ManipulabilityReport::compute(&sr);
        assert!((m.yoshikawa - 5.0).abs() < 1e-12);
        assert!((m.isotropy - 0.2).abs() < 1e-12);
    }

    #[test]
    fn rank_deficient_zero_manipulability() {
        // σ = [2, 0] → w = 0, isotropy = 0
        let sr = make_singularity(vec![2.0, 0.0], 1, f64::INFINITY);
        let m = ManipulabilityReport::compute(&sr);
        assert!((m.yoshikawa - 0.0).abs() < 1e-12);
        assert!((m.isotropy - 0.0).abs() < 1e-12);
    }

    #[test]
    fn empty_singular_values() {
        let sr = make_singularity(vec![], 0, f64::INFINITY);
        let m = ManipulabilityReport::compute(&sr);
        assert!((m.yoshikawa - 1.0).abs() < 1e-12); // empty product = 1
        assert!((m.isotropy - 0.0).abs() < 1e-12);
    }

    // ─── Task 1.2: pre-SVD column scaling (spec manipulability-normalization
    // "Normalized Yoshikawa via Pre-SVD Scaling") ────────────────────────────

    use crate::kinematics::forward::ForwardKinematics;
    use crate::kinematics::jacobian::{GeometricJacobian, JacobianSolver};
    use crate::robot::builder::SerialChainBuilder;
    use crate::robot::joint::*;
    use crate::robot::link::Link;
    use crate::robot::scale::{
        manipulability_reference_dimension, scene_reference_dimension,
    };
    use crate::robot::segment::Segment;
    use crate::robot::serial_chain::SerialChain;
    use crate::spatial::frame::FrameId;
    use std::f64::consts::PI;
    use thalos_math::{Transform3D, UnitVector3, Vector3};

    /// 2R+1P robot: two revolute (z axis, links 1.0 each) + one prismatic
    /// (z axis, identity link). L_ref = 2.0.
    fn build_2r1p() -> SerialChain {
        let mut builder = SerialChainBuilder::new();
        let f1 = builder.create_frame("f1");
        let f2 = builder.create_frame("f2");
        let ee = builder.create_frame("ee");

        let r1 = JointType::Revolute(RevoluteJoint::new(
            0,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        builder.add_segment(Segment::new(
            FrameId::World,
            f1.clone(),
            r1,
            Link::new(0, Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0))),
        ));

        let r2 = JointType::Revolute(RevoluteJoint::new(
            1,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        builder.add_segment(Segment::new(
            f1,
            f2.clone(),
            r2,
            Link::new(1, Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0))),
        ));

        let p1 = JointType::Prismatic(PrismaticJoint::new(
            2,
            UnitVector3::z_axis(),
            JointLimits::new(-1.0, 1.0),
            Transform3D::identity(),
        ));
        builder.add_segment(Segment::new(
            f2,
            ee.clone(),
            p1,
            Link::new(2, Transform3D::identity()),
        ));

        builder.set_end_effector(ee);
        builder.build().expect("2R+1P chain")
    }

    /// Planar 2R robot with all translations scaled by `scale` (uniform
    /// homothety: link translations × scale, joint origins × scale).
    fn build_planar_2r_scaled(scale: f64) -> SerialChain {
        let mut builder = SerialChainBuilder::new();
        let shoulder = builder.create_frame("shoulder");
        let ee = builder.create_frame("ee");

        let j1 = JointType::Revolute(RevoluteJoint::new(
            0,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        builder.add_segment(Segment::new(
            FrameId::World,
            shoulder.clone(),
            j1,
            Link::new(
                0,
                Transform3D::from_translation(Vector3::new(1.0 * scale, 0.0, 0.0)),
            ),
        ));

        let j2 = JointType::Revolute(RevoluteJoint::new(
            1,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        builder.add_segment(Segment::new(
            shoulder,
            ee.clone(),
            j2,
            Link::new(
                1,
                Transform3D::from_translation(Vector3::new(1.0 * scale, 0.0, 0.0)),
            ),
        ));

        builder.set_end_effector(ee);
        builder.build().expect("planar 2R chain")
    }

    fn evaluate_report(chain: &SerialChain, q: &[f64]) -> ManipulabilityReport {
        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, chain.end_effector.clone());
        let jacobian = jac.evaluate(q);
        let singularity = SingularityReport::analyze(&jacobian);
        ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, chain)
    }

    #[test]
    fn scale_jacobian_columns_scales_only_revolute_columns() {
        // Spec "Mixed revolute-prismatic robot": in a 2R+1P Jacobian only the
        // 2 revolute columns are divided by L_ref; the prismatic column stays
        // unscaled (it is dimensionless — dividing it would inject a fake
        // scale into a unit axis).
        let chain = build_2r1p();
        let l_ref = manipulability_reference_dimension(&chain);
        assert!((l_ref - 2.0).abs() < 1e-12, "L_ref = 1.0 + 1.0 = 2.0");

        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, chain.end_effector.clone());
        let jacobian = jac.evaluate(&[0.6, -0.4, 0.2]);

        let scaled = scale_jacobian_columns(&jacobian.linear, &chain, l_ref);

        for row in 0..3 {
            // Revolute columns ÷ L_ref
            assert!(
                (scaled[(row, 0)] - jacobian.linear[(row, 0)] / 2.0).abs() < 1e-12,
                "revolute col 0 must be ÷L_ref"
            );
            assert!(
                (scaled[(row, 1)] - jacobian.linear[(row, 1)] / 2.0).abs() < 1e-12,
                "revolute col 1 must be ÷L_ref"
            );
            // Prismatic column untouched
            assert!(
                (scaled[(row, 2)] - jacobian.linear[(row, 2)]).abs() < 1e-12,
                "prismatic col must stay unscaled"
            );
        }
    }

    #[test]
    fn homothety_scale_invariance_normalized_and_grade() {
        // Spec "Scale invariance under uniform robot scaling": robot B = robot A
        // scaled by s must yield identical normalized_yoshikawa AND identical
        // grade per waypoint (dimensionless, invariant under uniform scaling).
        let chain_a = build_planar_2r_scaled(1.0); // L_ref = 2.0
        let chain_b = build_planar_2r_scaled(2.0); // L_ref = 4.0 (s = 2)

        let waypoints: Vec<Vec<f64>> = vec![
            vec![0.0, 0.0],
            vec![0.5, -0.3],
            vec![1.2, 0.9],
            vec![-0.8, 0.4],
        ];

        for q in &waypoints {
            let report_a = evaluate_report(&chain_a, q);
            let report_b = evaluate_report(&chain_b, q);

            assert!(
                (report_a.normalized_yoshikawa - report_b.normalized_yoshikawa).abs() < 1e-9,
                "normalized must be identical under homothety at q={q:?}: {} vs {}",
                report_a.normalized_yoshikawa,
                report_b.normalized_yoshikawa
            );
            assert_eq!(
                report_a.manipulability_grade, report_b.manipulability_grade,
                "grade must be identical under homothety at q={q:?}"
            );
        }
    }

    #[test]
    fn rank_deficient_configuration_stays_structurally_low() {
        // Spec "Singularity preservation": full extension (q = [0, 0]) makes
        // both revolute columns collinear (z × r along the same direction) →
        // structural rank loss of the ORIGINAL Jacobian. Column scaling with
        // finite factors cannot repair a lost rank, so the normalized measure
        // must stay structurally low and classify LOW — checked by rank, not
        // by an absolute epsilon.
        let chain = build_planar_2r_scaled(1.0);
        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, chain.end_effector.clone());

        let jacobian = jac.evaluate(&[0.0, 0.0]);
        let singularity = SingularityReport::analyze(&jacobian);
        assert_eq!(
            singularity.rank, 1,
            "full extension must lose rank structurally (rank 1)"
        );

        let report = ManipulabilityReport::compute_with_normalization(
            &singularity,
            &jacobian,
            &chain,
        );
        assert_eq!(
            report.manipulability_grade,
            Some(ManipulabilityGrade::Low),
            "structural rank loss must classify LOW"
        );
        assert!(
            report.normalized_yoshikawa < 1e-9,
            "structural rank loss must keep normalized ≈ 0, got {}",
            report.normalized_yoshikawa
        );
    }

    // ─── Task 1.4: grade boundaries + raw preservation ─────────────────────

    #[test]
    fn grade_boundaries_at_thresholds() {
        // Design "Threshold Calibration" + spec "Grade assignment": partition
        // is Low < T_LOW ≤ Medium < T_HIGH ≤ High. Boundaries are inclusive
        // on the upper side: exactly T_LOW → medium, exactly T_HIGH → high.
        let eps = 1e-12;

        assert_eq!(classify(T_LOW - eps, T_LOW, T_HIGH), ManipulabilityGrade::Low);
        assert_eq!(classify(T_LOW, T_LOW, T_HIGH), ManipulabilityGrade::Medium);
        assert_eq!(
            classify((T_LOW + T_HIGH) / 2.0, T_LOW, T_HIGH),
            ManipulabilityGrade::Medium
        );
        assert_eq!(classify(T_HIGH - eps, T_LOW, T_HIGH), ManipulabilityGrade::Medium);
        assert_eq!(classify(T_HIGH, T_LOW, T_HIGH), ManipulabilityGrade::High);
        assert_eq!(classify(T_HIGH + eps, T_LOW, T_HIGH), ManipulabilityGrade::High);
    }

    #[test]
    fn spec_grade_assignment_example() {
        // Spec manipulability-normalization "Grade assignment": normalized
        // 0.15 with T_LOW = 0.1, T_HIGH = 0.4 → "medium" (the spec scenario
        // uses its own GIVEN thresholds — classification is a pure function
        // of the three inputs).
        assert_eq!(classify(0.15, 0.1, 0.4), ManipulabilityGrade::Medium);
        // The same normalized value with the calibrated constants classifies
        // by the re-calibrated partition (T_HIGH = 0.15433) — the scenario
        // premise (threshold values) changed with the calibration, the
        // partition logic did not.
        assert_eq!(classify(0.15, T_LOW, T_HIGH), ManipulabilityGrade::Medium);
        assert_eq!(classify(0.2, T_LOW, T_HIGH), ManipulabilityGrade::High);
    }

    #[test]
    fn compute_with_normalization_keeps_raw_unchanged() {
        // Spec "Raw Yoshikawa Preservation": the normalized path must NOT
        // perturb the raw metrics — same input, same yoshikawa/isotropy as
        // the plain `compute` path (SingularityReport stays on the original
        // Jacobian, spec "det_jtj Exclusion from Grade").
        let chain = build_2r1p();
        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, chain.end_effector.clone());

        for q in [vec![0.0, 0.0, 0.0], vec![0.6, -0.4, 0.2], vec![-1.2, 0.9, -0.5]] {
            let jacobian = jac.evaluate(&q);
            let singularity = SingularityReport::analyze(&jacobian);

            let raw = ManipulabilityReport::compute(&singularity);
            let normalized =
                ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, &chain);

            assert!(
                (normalized.yoshikawa - raw.yoshikawa).abs() < 1e-12,
                "yoshikawa must be identical: {} vs {}",
                normalized.yoshikawa,
                raw.yoshikawa
            );
            assert!(
                (normalized.isotropy - raw.isotropy).abs() < 1e-12,
                "isotropy must be identical"
            );
        }
    }

    #[test]
    fn compute_without_normalization_defaults_are_legacy_markers() {
        // `compute()` (raw path) must leave the additive fields at their
        // legacy defaults — `normalized_yoshikawa = 0.0`,
        // `manipulability_grade = None` — the frontend's presence signal
        // (design "Grade as Option for presence signal").
        let sr = make_singularity(vec![3.0, 3.0, 3.0], 3, 1.0);
        let report = ManipulabilityReport::compute(&sr);
        assert_eq!(report.normalized_yoshikawa, 0.0);
        assert_eq!(report.manipulability_grade, None);
    }

    #[test]
    fn degenerate_l_ref_guardrail_produces_no_nan() {
        // Guardrail `l_ref > ε`: a degenerate chain (identity translations →
        // L_ref floors at EPS) must not produce NaN/Inf — the column scaling
        // refuses to divide by a degenerate factor and the report stays
        // finite.
        let mut builder = SerialChainBuilder::new();
        let f1 = builder.create_frame("f1");
        let ee = builder.create_frame("ee");
        builder.add_segment(Segment::new(
            FrameId::World,
            f1.clone(),
            JointType::Revolute(RevoluteJoint::new(
                0,
                UnitVector3::z_axis(),
                JointLimits::new(-PI, PI),
                Transform3D::identity(),
            )),
            Link::new(0, Transform3D::identity()),
        ));
        builder.add_segment(Segment::new(
            f1,
            ee.clone(),
            JointType::Revolute(RevoluteJoint::new(
                1,
                UnitVector3::z_axis(),
                JointLimits::new(-PI, PI),
                Transform3D::identity(),
            )),
            Link::new(1, Transform3D::identity()),
        ));
        builder.set_end_effector(ee);
        let chain = builder.build().expect("degenerate 2R");

        let fk = ForwardKinematics::new(chain.clone());
        let jac = GeometricJacobian::new(fk, chain.end_effector.clone());
        let jacobian = jac.evaluate(&[0.4, 0.2]);
        let singularity = SingularityReport::analyze(&jacobian);
        let report =
            ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, &chain);

        assert!(report.normalized_yoshikawa.is_finite(), "no NaN/Inf from degenerate L_ref");
        assert!(
            report.manipulability_grade.is_some(),
            "grade still classified on the unscaled guard path"
        );
    }

    // ─── Remediation: fixed-joint independence (moving-only L_ref) ──────────
    //
    // User decision (source of truth): the normalization divisor must NOT
    // include fixed terminal joints (a fixed TCP contributes no Jacobian
    // columns — its length would penalize as "dead weight"). Robot A and
    // Robot B share the SAME moving chain (2 revolute, L_ref = 2.0) and
    // differ ONLY in the fixed TCP z-offset (0.12 vs 0.50).

    /// 2R (z-axis) moving chain + fixed terminal TCP joint with z-offset.
    fn build_2r_with_fixed_tcp(tcp_z: f64) -> SerialChain {
        let mut builder = SerialChainBuilder::new();
        let f1 = builder.create_frame("f1");
        let f2 = builder.create_frame("f2");
        let tcp = builder.create_frame("tcp");

        let r1 = JointType::Revolute(RevoluteJoint::new(
            0,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        builder.add_segment(Segment::new(
            FrameId::World,
            f1.clone(),
            r1,
            Link::new(0, Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0))),
        ));

        let r2 = JointType::Revolute(RevoluteJoint::new(
            1,
            UnitVector3::z_axis(),
            JointLimits::new(-PI, PI),
            Transform3D::identity(),
        ));
        builder.add_segment(Segment::new(
            f1,
            f2.clone(),
            r2,
            Link::new(1, Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0))),
        ));

        let tcp_fixed = JointType::Fixed(FixedJoint::new(Transform3D::from_translation(
            Vector3::new(0.0, 0.0, tcp_z),
        )));
        builder.add_segment(Segment::new(
            f2,
            tcp.clone(),
            tcp_fixed,
            Link::new(2, Transform3D::identity()),
        ));

        builder.set_end_effector(tcp);
        builder.build().expect("2R + fixed TCP chain")
    }

    #[test]
    fn fixed_terminal_joint_does_not_affect_normalization() {
        // INVARIANT (design remediation): "Adding/removing/changing a fixed
        // terminal joint with the same moving chain MUST NOT change
        // normalized_yoshikawa nor manipulability_grade."
        //
        // Both robots share the moving chain (2 revolute z-axis, L_ref = 2.0);
        // the fixed TCP z-offset differs (0.12 vs 0.50). A translation along
        // the rotation axis leaves the linear Jacobian columns unchanged
        // (z × Δz = 0), so the Jacobian is identical — and so must be the
        // normalized measure + grade. The SCENE reference dimension, by
        // contrast, MUST differ (it counts all segments).
        let robot_a = build_2r_with_fixed_tcp(0.12);
        let robot_b = build_2r_with_fixed_tcp(0.50);

        // Scene (all segments): fixed TCP contributes → differs.
        let scene_a = scene_reference_dimension(&robot_a);
        let scene_b = scene_reference_dimension(&robot_b);
        assert!((scene_a - 2.12).abs() < 1e-12);
        assert!((scene_b - 2.50).abs() < 1e-12);
        assert!(
            (scene_a - scene_b).abs() > 0.1,
            "scene L_ref must differ (fixed TCP counts for the scene)"
        );

        // Manipulability (moving-only): fixed TCP excluded → identical.
        let moving_a = manipulability_reference_dimension(&robot_a);
        let moving_b = manipulability_reference_dimension(&robot_b);
        assert!((moving_a - 2.0).abs() < 1e-12);
        assert_eq!(moving_a, moving_b);

        // Same Jacobian → same normalized + grade, waypoint by waypoint.
        for q in [vec![0.5, -0.3], vec![1.2, 0.9], vec![-0.8, 0.4], vec![0.0, 0.0]] {
            let report_a = evaluate_report(&robot_a, &q);
            let report_b = evaluate_report(&robot_b, &q);

            assert!(
                (report_a.normalized_yoshikawa - report_b.normalized_yoshikawa).abs() < 1e-9,
                "fixed TCP must not change normalized_yoshikawa at q={q:?}: {} vs {}",
                report_a.normalized_yoshikawa,
                report_b.normalized_yoshikawa
            );
            assert_eq!(
                report_a.manipulability_grade, report_b.manipulability_grade,
                "fixed TCP must not change manipulability_grade at q={q:?}"
            );
        }
    }

    // ─── Review fixes ──────────────────────────────────────────────────────

    /// 3R (z-axis) chain whose end effector is the SECOND frame — the third
    /// moving joint sits below the reference frame, so the GeometricJacobian
    /// produces only 2 columns (segments past the reference frame contribute
    /// nothing to the TCP Jacobian).
    fn build_3r_tcp_on_frame_2() -> SerialChain {
        let mut builder = SerialChainBuilder::new();
        let f1 = builder.create_frame("f1");
        let f2 = builder.create_frame("f2");
        let f3 = builder.create_frame("f3");
        for i in 0..3 {
            let joint = JointType::Revolute(RevoluteJoint::new(
                i,
                UnitVector3::z_axis(),
                JointLimits::new(-PI, PI),
                Transform3D::identity(),
            ));
            let (parent, child) = match i {
                0 => (FrameId::World, f1.clone()),
                1 => (f1.clone(), f2.clone()),
                _ => (f2.clone(), f3.clone()),
            };
            builder.add_segment(Segment::new(
                parent,
                child,
                joint,
                Link::new(
                    i,
                    Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                ),
            ));
        }
        builder.set_end_effector(f2);
        builder.build().expect("3R chain, TCP on frame 2")
    }

    #[test]
    fn scale_jacobian_columns_tcp_on_intermediate_frame_no_panic() {
        // Review blocker: `scale_jacobian_columns` iterated ALL moving segments
        // (col += 1 per moving joint) while a Jacobian built for a reference
        // frame carries only the columns up to that frame. A TCP on an
        // intermediate frame (moving joints AFTER the frame) made `col` reach
        // `linear.ncols()` → nalgebra index-out-of-bounds panic.
        //
        // The fix: break once `col` passes the last existing column and scale
        // exactly the columns the Jacobian actually has.
        let chain = build_3r_tcp_on_frame_2();
        assert_eq!(chain.dof_count(), 3, "3 moving joints in the chain");

        // Jacobian of the TCP at frame 2: 2 columns (a solver that sizes by
        // reference frame, not by total chain DOF). The 3rd moving joint has
        // NO column — this is the shape that used to drive `col` out of bounds.
        let mut linear = DynamicMatrix::zeros(3, 2);
        for (row, col) in [(0, 0), (1, 1)] {
            linear[(row, col)] = 1.0;
        }
        let jacobian = Jacobian::new(linear, DynamicMatrix::zeros(3, 2));

        let l_ref = manipulability_reference_dimension(&chain);
        let scaled = scale_jacobian_columns(&jacobian.linear, &chain, l_ref);

        for row in 0..3 {
            assert!(
                (scaled[(row, 0)] - jacobian.linear[(row, 0)] / l_ref).abs() < 1e-12,
                "existing col 0 must be ÷L_ref"
            );
            assert!(
                (scaled[(row, 1)] - jacobian.linear[(row, 1)] / l_ref).abs() < 1e-12,
                "existing col 1 must be ÷L_ref"
            );
        }
    }

    #[test]
    fn zero_dof_chain_is_not_classified() {
        // Review suggestion: an all-fixed chain produces a 3×0 linear Jacobian
        // — no singular values to multiply. The empty-product 1.0 → High would
        // be a fabricated grade for a chain with nothing to measure, so the
        // normalized measure stays 0.0 and the grade stays None (undefined,
        // legacy signal) — never classified.
        let mut builder = SerialChainBuilder::new();
        let f1 = builder.create_frame("f1");
        let ee = builder.create_frame("ee");
        builder.add_segment(Segment::new(
            FrameId::World,
            f1.clone(),
            JointType::Fixed(FixedJoint::new(Transform3D::identity())),
            Link::new(0, Transform3D::identity()),
        ));
        builder.add_segment(Segment::new(
            f1,
            ee.clone(),
            JointType::Fixed(FixedJoint::new(Transform3D::identity())),
            Link::new(1, Transform3D::identity()),
        ));
        builder.set_end_effector(ee);
        let chain = builder.build().expect("all-fixed chain");
        assert_eq!(chain.dof_count(), 0);

        // The 3×0 Jacobian has no singular values; `SingularityReport::analyze`
        // panics on the empty SVD, so the report is constructed directly (the
        // raw path already tolerates an empty singular-value list).
        let jacobian =
            Jacobian::new(DynamicMatrix::zeros(3, 0), DynamicMatrix::zeros(3, 0));
        let singularity = make_singularity(vec![], 0, f64::INFINITY);
        let report =
            ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, &chain);

        assert_eq!(report.normalized_yoshikawa, 0.0);
        assert_eq!(report.manipulability_grade, None);
    }
}
