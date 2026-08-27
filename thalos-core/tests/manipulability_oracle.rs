//! Oracle test (task 7.1) — the 4 verification checks from design
//! "Threshold Calibration / Verification" + spec manipulability-normalization
//! "SCARA Regression Oracle".
//!
//! 1. SCARA canonical grade distribution reproduced point-to-point with the
//!    calibrated constant thresholds (T_LOW = 0.0926, T_HIGH = 0.15433 —
//!    calibrated after the moving-only L_ref decision; see design.md
//!    "Calibration outcome").
//! 2. Icebot is NOT force-promoted: its grades derive ONLY from the constant
//!    thresholds, and a homothetic copy yields IDENTICAL grades — the scale
//!    artifact is eliminated (the raw metric would differ).
//! 3. Homothety: robot + uniformly scaled copy → identical normalized +
//!    grade per waypoint (the principal guardrail).
//! 4. Singularity preservation: a structurally rank-deficient configuration
//!    stays LOW (checked by structural rank loss, not an absolute epsilon).

use rand::SeedableRng;
use rand::rngs::StdRng;

use thalos_core::analysis::workspace::{WorkspaceConfig, WorkspaceSampler};
use thalos_core::kinematics::forward::ForwardKinematics;
use thalos_core::kinematics::jacobian::manipulability::{ManipulabilityGrade, T_HIGH, T_LOW};
use thalos_core::kinematics::jacobian::{
    GeometricJacobian, JacobianSolver, ManipulabilityReport, SingularityReport,
};
use thalos_core::models::scara::ScaraSpec;
use thalos_core::robot::adapter;
use thalos_core::robot::joint::JointType;
use thalos_core::robot::scale::manipulability_reference_dimension;
use thalos_core::robot::serial_chain::SerialChain;
use thalos_math::Vector3;

/// Raw reference partition (backend planning thresholds, ~1 m robots).
fn raw_grade(w: f64) -> ManipulabilityGrade {
    if w < 0.3 {
        ManipulabilityGrade::Low
    } else if w < 0.5 {
        ManipulabilityGrade::Medium
    } else {
        ManipulabilityGrade::High
    }
}

/// Classify a normalized value with the CALIBRATED constant thresholds.
fn normalized_grade(n: f64) -> ManipulabilityGrade {
    if n < T_LOW {
        ManipulabilityGrade::Low
    } else if n < T_HIGH {
        ManipulabilityGrade::Medium
    } else {
        ManipulabilityGrade::High
    }
}

/// Evaluate raw + normalized at a configuration.
fn evaluate(chain: &SerialChain, q: &[f64]) -> (f64, f64, ManipulabilityGrade) {
    let fk = ForwardKinematics::new(chain.clone());
    let jac = GeometricJacobian::new(fk, chain.end_effector.clone());
    let jacobian = jac.evaluate(q);
    let singularity = SingularityReport::analyze(&jacobian);
    let report = ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, chain);
    (
        report.yoshikawa,
        report.normalized_yoshikawa,
        report.manipulability_grade.expect("grade classified"),
    )
}

/// Uniformly scale every translation in a chain by `s` (link transforms AND
/// joint origins) — a homothetic copy for the scale-invariance oracle.
fn scale_chain(chain: &SerialChain, s: f64) -> SerialChain {
    let mut scaled = chain.clone();
    for segment in &mut scaled.segments {
        segment.link.transform.translation = segment.link.transform.translation * s;
        match &mut segment.joint {
            JointType::Revolute(j) => j.origin.translation = j.origin.translation * s,
            JointType::Prismatic(j) => j.origin.translation = j.origin.translation * s,
            JointType::Fixed(j) => j.origin.translation = j.origin.translation * s,
        }
    }
    scaled
}

// ─── Check 1: SCARA canonical point-to-point ──────────────────────────────

#[test]
fn oracle_scara_grade_partition_reproduced_point_to_point() {
    // Calibration run: SCARA canonical, seed 42, 5000 samples — the exact
    // dataset the thresholds were locked against (design "Threshold
    // Calibration"). Each waypoint's normalized grade must match the raw
    // reference grade (0.3/0.5) AT THAT WAYPOINT.
    let chain = ScaraSpec::canonical().build();
    let fk = ForwardKinematics::new(chain.clone());
    let jac = GeometricJacobian::new(fk, chain.end_effector.clone());

    let mut rng = StdRng::seed_from_u64(42);
    let ws = WorkspaceSampler
        .sample(
            &chain,
            WorkspaceConfig {
                samples: 5000,
                seed: 42,
                tolerance: 1e-3,
            },
            &mut rng,
        )
        .expect("sampling failed");

    let mut checked = 0;
    for sample in ws.samples() {
        let jacobian = jac.evaluate(&sample.q);
        let singularity = SingularityReport::analyze(&jacobian);
        let report =
            ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, &chain);
        let reference = raw_grade(report.yoshikawa);
        let actual = normalized_grade(report.normalized_yoshikawa);
        assert_eq!(
            actual, reference,
            "waypoint grade mismatch: raw={:.6} (ref {reference:?}) normalized={:.6} (got {actual:?})",
            report.yoshikawa, report.normalized_yoshikawa
        );
        checked += 1;
    }
    assert_eq!(checked, 5000, "oracle must check every waypoint");
}

// ─── Check 2: icebot NOT force-promoted + scale artifact eliminated ───────

#[test]
fn oracle_icebot_not_force_promoted_and_scale_artifact_gone() {
    let src = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/icebot.urdf"
    ));
    let robot = thalos_models::urdf::parser::parse_robot(src).expect("icebot URDF");
    let chain = adapter::auto(&robot).expect("icebot chain");

    // Moving-only L_ref (remediation): the fixed tcp_joint (0.12 m) is
    // excluded from the normalization divisor (it generates no Jacobian
    // columns). Empirically (task 6.1 re-run, REAL SVD): the icebot truly
    // maxes at normalized ≈ 0.084 < T_LOW = 0.0926 (brute-force grid over
    // the 4-DOF box, 40⁴ points — NOT the ratio³ simulation, which
    // over-predicted ~51% HIGH). Its proportions (short arms relative to
    // L_ref = 0.385, wrist on the tool axis, limited joint ranges) keep it
    // genuinely below the SCARA-calibrated partition: 100% LOW, honest, no
    // force-promotion and no force-suppression.
    let fk = ForwardKinematics::new(chain.clone());
    let jac = GeometricJacobian::new(fk, chain.end_effector.clone());

    let mut rng = StdRng::seed_from_u64(42);
    let ws = WorkspaceSampler
        .sample(
            &chain,
            WorkspaceConfig {
                samples: 500,
                seed: 42,
                tolerance: 1e-3,
            },
            &mut rng,
        )
        .expect("sampling failed");

    let mut all_grades_match_thresholds = true;
    let mut max_normalized = 0.0_f64;
    for sample in ws.samples() {
        let jacobian = jac.evaluate(&sample.q);
        let singularity = SingularityReport::analyze(&jacobian);
        let report =
            ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, &chain);
        // Honest classification: the grade IS the constant-threshold
        // classification of the normalized value — nothing robot-specific.
        all_grades_match_thresholds &=
            report.manipulability_grade == Some(normalized_grade(report.normalized_yoshikawa));
        max_normalized = max_normalized.max(report.normalized_yoshikawa);
    }
    assert!(
        all_grades_match_thresholds,
        "every icebot grade must equal the constant-threshold classification (no force-promotion)"
    );
    assert!(
        max_normalized < T_LOW,
        "icebot normalizes below T_LOW (max {max_normalized}) — all-LOW is the honest outcome"
    );

    // Scale artifact elimination: a ×2 homothetic copy yields IDENTICAL
    // normalized values — the raw metric would change with scale, the
    // normalized one cannot. That is the property that kills the
    // "TODO rojo por artefacto de escala".
    let chain_x2 = scale_chain(&chain, 2.0);
    let fk2 = ForwardKinematics::new(chain_x2.clone());
    let jac2 = GeometricJacobian::new(fk2, chain_x2.end_effector.clone());
    let mut rng2 = StdRng::seed_from_u64(42);
    let ws2 = WorkspaceSampler
        .sample(
            &chain_x2,
            WorkspaceConfig {
                samples: 500,
                seed: 42,
                tolerance: 1e-3,
            },
            &mut rng2,
        )
        .expect("sampling failed");

    let zipped = ws.samples().iter().zip(ws2.samples());
    let mut any_raw_differed = false;
    for (a, b) in zipped {
        let j_a = jac.evaluate(&a.q);
        let j_b = jac2.evaluate(&b.q);
        let sr_a = SingularityReport::analyze(&j_a);
        let sr_b = SingularityReport::analyze(&j_b);
        let rep_a = ManipulabilityReport::compute_with_normalization(&sr_a, &j_a, &chain);
        let rep_b = ManipulabilityReport::compute_with_normalization(&sr_b, &j_b, &chain_x2);

        any_raw_differed |= (rep_a.yoshikawa - rep_b.yoshikawa).abs() > 1e-9;
        assert!(
            (rep_a.normalized_yoshikawa - rep_b.normalized_yoshikawa).abs() < 1e-9,
            "normalized must be identical under icebot homothety: {} vs {}",
            rep_a.normalized_yoshikawa,
            rep_b.normalized_yoshikawa
        );
        assert_eq!(rep_a.manipulability_grade, rep_b.manipulability_grade);
    }
    assert!(
        any_raw_differed,
        "raw must differ under scale (that is the artifact the normalized metric removes)"
    );
}

// ─── Check 3: homothety — identical normalized + grade per waypoint ───────

#[test]
fn oracle_homothety_identical_normalized_and_grade() {
    // SCARA canonical (moving-only `manipulability_reference_dimension` = 1.8)
    // vs a ×2 copy (L_ref = 3.6), evaluated at the SAME configurations:
    // normalized + grade must match per waypoint.
    let chain_a = ScaraSpec::canonical().build();
    let chain_b = ScaraSpec::canonical().build();
    let chain_b = scale_chain(&chain_b, 2.0);

    assert!(
        (manipulability_reference_dimension(&chain_a) * 2.0
            - manipulability_reference_dimension(&chain_b))
        .abs()
            < 1e-9
    );

    let waypoints: Vec<Vec<f64>> = vec![
        vec![0.0, 0.0, 0.0, 0.0],
        vec![0.5, -0.3, -0.2, 0.4],
        vec![1.2, 0.9, -0.1, -1.0],
        vec![-0.8, 0.4, 0.0, 0.5],
        vec![2.0, -1.5, -0.5, 3.0],
    ];

    for q in &waypoints {
        let (_, normalized_a, grade_a) = evaluate(&chain_a, q);
        let (_, normalized_b, grade_b) = evaluate(&chain_b, q);
        assert!(
            (normalized_a - normalized_b).abs() < 1e-9,
            "normalized must be identical under homothety at q={q:?}: {normalized_a} vs {normalized_b}"
        );
        assert_eq!(grade_a, grade_b, "grade must be identical at q={q:?}");
    }
}

// ─── Check 4: singularity preservation (structural rank loss) ─────────────

#[test]
fn oracle_structural_rank_loss_stays_low() {
    // SCARA at full extension (q1 = q2 = 0): both revolute columns are
    // collinear (z × r along the same direction) → the ORIGINAL Jacobian
    // loses rank structurally. Column scaling with finite factors cannot
    // repair a lost rank, so the normalized measure stays LOW — checked by
    // the structural rank of the original Jacobian, not an absolute epsilon.
    let chain = ScaraSpec::canonical().build();
    let fk = ForwardKinematics::new(chain.clone());
    let jac = GeometricJacobian::new(fk, chain.end_effector.clone());

    let jacobian = jac.evaluate(&[0.0, 0.0, 0.0, 0.0]);
    let singularity = SingularityReport::analyze(&jacobian);
    assert!(
        singularity.rank < 3,
        "full extension must structurally lose rank (rank {}, expected < 3)",
        singularity.rank
    );

    let report = ManipulabilityReport::compute_with_normalization(&singularity, &jacobian, &chain);
    assert_eq!(
        report.manipulability_grade,
        Some(ManipulabilityGrade::Low),
        "structural rank loss must keep the grade LOW"
    );
    assert!(
        report.normalized_yoshikawa < 1e-9,
        "structural rank loss must keep normalized ≈ 0, got {}",
        report.normalized_yoshikawa
    );

    // Triangulation: a non-singular configuration of the same robot must NOT
    // be forced low — the singularity preservation is specific, not a
    // blanket depression.
    let (_, normalized_healthy, grade_healthy) = evaluate(&chain, &[0.5, -0.7, -0.2, 0.3]);
    assert!(
        normalized_healthy > report.normalized_yoshikawa * 10.0,
        "a healthy config must normalize well above the singular one"
    );
    assert_ne!(grade_healthy, ManipulabilityGrade::Low);
}
