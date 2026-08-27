use super::*;

// ═══════════════════════════════════════════════════════════════════════
// FASE 3: DAMPED LEAST SQUARES SOLVER
// ═══════════════════════════════════════════════════════════════════════

/// 13. DLS CONVERGE EN TARGETS ALCANZABLES: el solver debe converger
///     al target (1, 1, 0) con status Converged y error < tolerancia.
#[test]
fn converges_reachable_target() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1);

    let target = Vector3::new(1.0, 1.0, 0.0);
    let result = solver
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("DLS solve should succeed");

    assert!(
        result.status.is_converged(),
        "DLS debe converger para target alcanzable: status={:?}, error={:.2e}",
        result.status,
        result.final_error
    );
    assert!(
        result.final_error < 1e-5,
        "Error final ({:.2e}) debe estar cerca de la tolerancia (1e-6)",
        result.final_error
    );
    assert!(result.iterations > 0, "Debe haber al menos 1 iteración");
    assert!(
        result.iterations < 500,
        "No debe agotar max_iters: {}",
        result.iterations
    );

    println!(
        "  DLS: {} iter, error final = {:.2e}, λ = {}",
        result.iterations, result.final_error, 0.1
    );
}

/// 14. DLS EN SINGULARIDAD: a diferencia de JT, DLS puede converger
///     desde q=[0,0] aunque el target tenga componente radial.
///     Target (1.2, 0.5, 0) — bien dentro del workspace.
#[test]
fn converges_from_singular() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1);

    // q=[0,0] es singular; JT se queda atascado con J^T·e radial
    let target = Vector3::new(1.2, 0.5, 0.0);
    let result = solver
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("DLS solve should succeed");

    assert!(
        result.status.is_converged(),
        "DLS debe converger desde singular: status={:?}, error={:.2e}, iter={}",
        result.status,
        result.final_error,
        result.iterations
    );

    println!(
        "  DLS desde singular: {} iter, error = {:.2e}, λ = {}",
        result.iterations, result.final_error, 0.1
    );
}

/// 15. DLS SUPERA A JT DESDE SINGULARIDAD: mismo target y q0, DLS
///     converge en menos iteraciones que JT porque el damping evita
///     el estancamiento cerca de la singularidad.
#[test]
fn faster_than_jt_from_singular() {
    let (fk, ee) = build_2dof_planar_arm();

    let target = Vector3::new(1.2, 0.5, 0.0);

    // DLS
    let dls = DampedLeastSquaresSolver::new(fk.clone(), ee.clone(), 500, 1e-6, 0.1);
    let r_dls = dls
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("DLS solve should succeed");

    // JT
    let jt = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5);
    let r_jt = jt
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("JT solve should succeed");

    assert!(
        r_dls.status.is_converged(),
        "DLS debe converger desde singular: status={:?}",
        r_dls.status
    );
    assert!(
        r_jt.status.is_converged(),
        "JT debe converger desde singular: status={:?}",
        r_jt.status
    );

    // DLS debe converger en menos iteraciones
    assert!(
        r_dls.iterations < r_jt.iterations,
        "DLS ({}) debe converger más rápido que JT ({}) desde singular",
        r_dls.iterations,
        r_jt.iterations
    );

    println!(
        "  DLS: {} iter, error = {:.2e} | JT: {} iter, error = {:.2e} | λ = 0.1",
        r_dls.iterations, r_dls.final_error, r_jt.iterations, r_jt.final_error
    );
}

/// 16. DLS CON TARGET INALCANZABLE: debe devolver MaxIterations sin NaN.
#[test]
fn unreachable_target_no_nan() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = DampedLeastSquaresSolver::new(fk, ee, 200, 1e-6, 0.1);

    // Target muy fuera del workspace
    let targets = [
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(5.0, 5.0, 0.0),
    ];

    for &target in &targets {
        let result = solver
            .solve(&[0.5, 0.0], IKGoal::Position(target))
            .expect("DLS solve should succeed");

        assert_eq!(
            result.status,
            IKStatus::MaxIterations,
            "Target inalcanzable debe dar MaxIterations"
        );

        for &q_val in &result.q {
            assert!(q_val.is_finite(), "q debe ser finito, got {}", q_val);
        }

        assert!(
            result.final_error.is_finite(),
            "final_error debe ser finito, got {}",
            result.final_error
        );
    }
}

/// 17. DLS CON HISTORIAL: verificar que with_history(true) funciona.
#[test]
fn error_history() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1).with_history(true);

    let target = Vector3::new(1.0, 1.0, 0.0);
    let result = solver
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("DLS solve should succeed");

    let history = result
        .error_history
        .expect("with_history(true) debe registrar historial");

    assert_eq!(
        history.len(),
        result.iterations,
        "Historial debe tener {} entradas",
        result.iterations
    );

    // DLS no garantiza monotonicidad estricta (el damping puede
    // producir overshoot), pero el error final debe ser MUCHO menor
    // que el inicial.
    let first_error = history[0];
    let last_error = *history.last().unwrap();
    assert!(
        last_error < first_error * 0.1,
        "Error final ({:.4}) debe ser << inicial ({:.4})",
        last_error,
        first_error
    );

    // Último valor debe coincidir con final_error
    assert!(
        (last_error - result.final_error).abs() < 1e-12,
        "Último historial debe coincidir con final_error"
    );
}

/// T6 (M2): the DLS solver exposes the robot chain it operates on, so the
/// advisor can recompile/re-analyze edited programs with the SAME kinematic
/// model (end-to-end availability verification, design ADR-2/ADR-3).
#[test]
fn exposes_the_robot_chain_it_was_built_with() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1);

    let robot = solver
        .robot()
        .expect("a solver wrapping a ForwardKinematics must expose its robot");
    assert_eq!(robot.dof_count(), 2, "the 2-DOF planar test arm");
}
