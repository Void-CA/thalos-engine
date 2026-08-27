use super::*;

// ═══════════════════════════════════════════════════════════════════════
// SINGULARITY DETECTION (solver-agnostic)
// ═══════════════════════════════════════════════════════════════════════

/// det(JᵀJ) ≈ 0 en q=[0,0] (brazo extendido): Jacobiano rank-deficient,
/// condition_number = ∞.
#[test]
fn detection_at_full_extension() {
    let (fk, ee) = build_2dof_planar_arm();
    let jac = GeometricJacobian::new(fk, ee);

    let j = jac.evaluate(&[0.0, 0.0]);
    let report = SingularityReport::analyze(&j);

    println!(
        "  q=[0,0]: det(JᵀJ)={:.6e}, rank={}, cond={:.6e}, sv={:?}",
        report.det_jtj, report.rank, report.condition_number, report.singular_values
    );

    // Rango 1 (solo puede mover en Y)
    assert_eq!(
        report.rank, 1,
        "Rango del Jacobiano en q=[0,0] debe ser 1, got {}",
        report.rank
    );

    // Número de condición debe ser ∞ (valor singular mínimo ≈ 0)
    assert!(
        report.condition_number.is_infinite(),
        "Condition number en q=[0,0] debe ser ∞, got {:.2}",
        report.condition_number
    );

    // det(JᵀJ) debe ser ≈ 0 (un valor singular es 0)
    assert!(
        report.det_jtj.abs() < 1e-12,
        "det(JᵀJ) en q=[0,0] debe ser ≈ 0, got {:.6e}",
        report.det_jtj
    );
}

/// det(JᵀJ) >> 0 en q=[π/3, π/4]: rango completo.
#[test]
fn detection_at_articulated() {
    let (fk, ee) = build_2dof_planar_arm();
    let jac = GeometricJacobian::new(fk, ee);

    let j = jac.evaluate(&[PI / 3.0, PI / 4.0]);
    let report = SingularityReport::analyze(&j);

    println!(
        "  q=[π/3, π/4]: det(JᵀJ)={:.6e}, rank={}, cond={:.6e}, sv={:?}",
        report.det_jtj, report.rank, report.condition_number, report.singular_values
    );

    assert!(
        report.det_jtj > 0.1,
        "det(JᵀJ) en q=[π/3,π/4] debe ser >> 0, got {:.6e}",
        report.det_jtj
    );

    assert_eq!(
        report.rank, 2,
        "Rango del Jacobiano en q=[π/3,π/4] debe ser 2, got {}",
        report.rank
    );

    assert!(
        report.condition_number.is_finite(),
        "Condition number en q=[π/3,π/4] debe ser finito, got {:.2}",
        report.condition_number
    );
    assert!(
        report.condition_number < 100.0,
        "Condition number en q=[π/3,π/4] debe ser moderado (< 100), got {:.2}",
        report.condition_number
    );
}

/// Comparativa singular vs no-singular.
#[test]
fn condition_number_comparison() {
    let (fk, ee) = build_2dof_planar_arm();
    let jac = GeometricJacobian::new(fk, ee);

    let j_sing = jac.evaluate(&[0.0, 0.0]);
    let rep_sing = SingularityReport::analyze(&j_sing);

    let j_nonsing = jac.evaluate(&[PI / 3.0, PI / 4.0]);
    let rep_nonsing = SingularityReport::analyze(&j_nonsing);

    println!(
        "  Cond. number: singular={:.2e}, no-singular={:.2e}",
        rep_sing.condition_number, rep_nonsing.condition_number
    );
    println!(
        "  det(JᵀJ): singular={:.6e}, no-singular={:.6e}",
        rep_sing.det_jtj, rep_nonsing.det_jtj
    );

    assert!(
        rep_sing.condition_number.is_infinite(),
        "Condition number singular debe ser ∞ (rango 1)"
    );
    assert!(
        rep_nonsing.condition_number.is_finite(),
        "Condition number no-singular debe ser finito, got {:.2}",
        rep_nonsing.condition_number
    );
    assert!(
        rep_sing.det_jtj.abs() < rep_nonsing.det_jtj * 1e-6,
        "det(JᵀJ) singular ({:.6e}) debe ser mucho menor que \
         no-singular ({:.6e})",
        rep_sing.det_jtj,
        rep_nonsing.det_jtj
    );
}
