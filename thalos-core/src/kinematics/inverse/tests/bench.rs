use super::*;

#[test]
fn test_transpose_vs_dls_reachable() {
    let (fk, ee) = build_2dof_planar_arm();
    let target = Vector3::new(1.0, 1.0, 0.0);

    let jt = JacobianTransposeSolver::new(fk.clone(), ee.clone(), 500, 1e-6, 0.5);
    let dls = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1);

    let r_jt = jt
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("JT solve should succeed");
    let r_dls = dls
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("DLS solve should succeed");

    assert!(
        r_jt.status.is_converged(),
        "JT debe converger: status={:?}",
        r_jt.status
    );
    assert!(
        r_dls.status.is_converged(),
        "DLS debe converger: status={:?}",
        r_dls.status
    );

    // DLS debe converger en ≤ 50% de las iteraciones de JT
    assert!(
        r_dls.iterations <= r_jt.iterations / 2,
        "DLS ({}) debe converger en ≤ 50% de JT ({})",
        r_dls.iterations,
        r_jt.iterations
    );

    println!(
        "  [REACHABLE] JT: {} iter, error={:.2e} | DLS: {} iter, error={:.2e} | ratio={:.3}",
        r_jt.iterations,
        r_jt.final_error,
        r_dls.iterations,
        r_dls.final_error,
        r_dls.iterations as f64 / r_jt.iterations as f64
    );
}

/// 19. CONFIGURACIÓN SINGULAR: desde q=[0,0] (brazo extendido),
///     DLS converge mientras que JT requiere muchas más iteraciones.
#[test]
fn test_transpose_vs_dls_singular() {
    let (fk, ee) = build_2dof_planar_arm();
    let target = Vector3::new(1.2, 0.5, 0.0);

    let jt = JacobianTransposeSolver::new(fk.clone(), ee.clone(), 500, 1e-6, 0.5);
    let dls = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1);

    let r_jt = jt
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("JT solve should succeed");
    let r_dls = dls
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("DLS solve should succeed");

    assert!(
        r_jt.status.is_converged(),
        "JT debe converger desde singular: status={:?}",
        r_jt.status
    );
    assert!(
        r_dls.status.is_converged(),
        "DLS debe converger desde singular: status={:?}",
        r_dls.status
    );

    // DLS debe converger en menos iteraciones
    assert!(
        r_dls.iterations < r_jt.iterations,
        "DLS ({}) debe converger más rápido que JT ({}) desde singular",
        r_dls.iterations,
        r_jt.iterations
    );

    println!(
        "  [SINGULAR] JT: {} iter, error={:.2e} | DLS: {} iter, error={:.2e} | ratio={:.3}",
        r_jt.iterations,
        r_jt.final_error,
        r_dls.iterations,
        r_dls.final_error,
        r_dls.iterations as f64 / r_jt.iterations as f64
    );
}

/// 20. TARGET INALCANZABLE: ambos solvers deben dar MaxIterations
///     con valores finitos (sin NaN/Inf).
#[test]
fn test_transpose_vs_dls_unreachable() {
    let (fk, ee) = build_2dof_planar_arm();

    let jt = JacobianTransposeSolver::new(fk.clone(), ee.clone(), 200, 1e-6, 0.5);
    let dls = DampedLeastSquaresSolver::new(fk, ee, 200, 1e-6, 0.1);

    let targets = [Vector3::new(3.0, 0.0, 0.0), Vector3::new(0.0, 3.0, 0.0)];

    for &target in &targets {
        let r_jt = jt
            .solve(&[0.5, 0.0], IKGoal::Position(target))
            .expect("JT solve should succeed");
        let r_dls = dls
            .solve(&[0.5, 0.0], IKGoal::Position(target))
            .expect("DLS solve should succeed");

        // Ambos deben agotar iteraciones
        assert_eq!(
            r_jt.status,
            IKStatus::MaxIterations,
            "JT debe dar MaxIterations para target inalcanzable"
        );
        assert_eq!(
            r_dls.status,
            IKStatus::MaxIterations,
            "DLS debe dar MaxIterations para target inalcanzable"
        );

        // Sin NaN
        for &q in &r_jt.q {
            assert!(q.is_finite(), "JT q debe ser finito, got {}", q);
        }
        for &q in &r_dls.q {
            assert!(q.is_finite(), "DLS q debe ser finito, got {}", q);
        }
        assert!(r_jt.final_error.is_finite(), "JT error debe ser finito");
        assert!(r_dls.final_error.is_finite(), "DLS error debe ser finito");

        // Errores en el mismo orden de magnitud
        let ratio =
            r_jt.final_error.max(r_dls.final_error) / r_jt.final_error.min(r_dls.final_error);
        assert!(
            ratio < 10.0,
            "Errores deben estar en el mismo orden: JT={:.2}, DLS={:.2}, ratio={:.2}",
            r_jt.final_error,
            r_dls.final_error,
            ratio
        );
    }
}
