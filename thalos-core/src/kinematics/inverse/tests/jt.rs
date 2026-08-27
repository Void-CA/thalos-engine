use super::*;

// ─── IKResult metadata ───────────────────────────────────────────────

/// El resultado expone status, iterations y final_error correctamente.
#[test]
fn result_exposes_metadata() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5);

    let q0 = vec![0.0, 0.0];
    let target = Vector3::new(1.0, 1.0, 0.0);
    let result = solver
        .solve(&q0, IKGoal::Position(target))
        .expect("JT solve should succeed");

    assert_eq!(result.q.len(), 2, "Solución debe tener 2 joint values");
    assert!(
        result.status.is_converged(),
        "Debe converger: status={:?}, final_error={:.2e}, iterations={}",
        result.status,
        result.final_error,
        result.iterations
    );
    assert!(result.iterations > 0, "Debe haber al menos 1 iteración");
    assert!(result.iterations < 500, "No debería agotar max_iters");
    assert!(
        result.iterations < 100,
        "JT con alpha=0.5 debería converger en <100 iteraciones, tomó {}",
        result.iterations
    );
    assert!(
        result.final_error < 1e-5,
        "Error final ({:.2e}) muy por encima de tolerancia (1e-6)",
        result.final_error
    );

    // Por defecto no hay historial
    assert!(
        result.error_history.is_none(),
        "Historial debe ser None por defecto"
    );
}

// ─── Error history ────────────────────────────────────────────────────

/// Historial de errores se registra con with_history(true).
#[test]
fn error_history_is_recorded() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5).with_history(true);

    let q0 = vec![0.0, 0.0];
    let target = Vector3::new(1.0, 1.0, 0.0);
    let result = solver
        .solve(&q0, IKGoal::Position(target))
        .expect("JT solve should succeed");

    let history = result
        .error_history
        .expect("with_history(true) debe registrar historial");

    assert_eq!(
        history.len(),
        result.iterations,
        "Historial debe tener {} entradas (== iterations)",
        result.iterations
    );

    // Primer error significativo (ee en (2,0,0), target (1,1,0))
    assert!(
        history[0] > 0.5,
        "Error inicial debe ser grande, fue {:.4}",
        history[0]
    );

    // Último historial coincide con final_error
    let last = history.last().unwrap();
    assert!(
        (last - result.final_error).abs() < 1e-12,
        "Último historial ({:.2e}) debe coincidir con final_error ({:.2e})",
        last,
        result.final_error
    );
}

// ─── Known solutions ──────────────────────────────────────────────────

/// Brazo planar 2R (L1=L2=1), target (1, 1, 0).
/// Solución esperada: q1 ≈ 0°, q2 ≈ 90°.
#[test]
fn two_dof_planar_arm_known_solution() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5);

    let q0 = vec![0.0, 0.0];
    let target = Vector3::new(1.0, 1.0, 0.0);
    let result = solver
        .solve(&q0, IKGoal::Position(target))
        .expect("JT solve should succeed");

    println!(
        "  q1 = {:.6} rad ({:.2}°)",
        result.q[0],
        result.q[0].to_degrees()
    );
    println!(
        "  q2 = {:.6} rad ({:.2}°)",
        result.q[1],
        result.q[1].to_degrees()
    );

    assert!(
        result.q[0].abs() < 1e-2,
        "Esperado q1 ≈ 0, got {}",
        result.q[0]
    );
    assert!(
        (result.q[1] - PI / 2.0).abs() < 1e-2,
        "Esperado q2 ≈ π/2, got {}",
        result.q[1]
    );
    assert!(result.status.is_converged(), "IK debe converger");
}

/// Consistencia FK: después de IK, FK(position(q)) ≈ target.
#[test]
fn fk_ik_consistency() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk.clone(), ee, 500, 1e-6, 0.5);

    let q0 = vec![0.0, 0.0];
    let target = Vector3::new(1.0, 1.0, 0.0);
    let result = solver
        .solve(&q0, IKGoal::Position(target))
        .expect("JT solve should succeed");

    let fk_result = fk.evaluate(&result.q);
    let reached = fk_result.ee_position().unwrap();
    let final_error = (target - reached).magnitude();

    println!(
        "  target  = ({:.4}, {:.4}, {:.4})",
        target.x, target.y, target.z
    );
    println!(
        "  reached = ({:.4}, {:.4}, {:.4})",
        reached.x, reached.y, reached.z
    );
    println!("  final error = {:.2e}", final_error);

    assert!(
        final_error < 1e-5,
        "FK/IK mismatch: error = {:.2e} (tolerancia IK = 1e-6)",
        final_error
    );
}

/// 1-DOF: L=1, target (0, 1, 0). Solución analítica: θ = π/2.
#[test]
fn one_dof_reaches_known_target() {
    let (fk, ee) = build_1dof_arm(1.0);
    let solver = JacobianTransposeSolver::new(fk, ee, 100, 1e-6, 0.5);

    let q0 = vec![0.0];
    let target = Vector3::new(0.0, 1.0, 0.0);
    let result = solver
        .solve(&q0, IKGoal::Position(target))
        .expect("JT solve should succeed");

    println!(
        "  θ = {:.6} rad ({:.2}°)",
        result.q[0],
        result.q[0].to_degrees()
    );

    assert!(
        (result.q[0] - PI / 2.0).abs() < 1e-3,
        "Esperado θ ≈ π/2, got {}",
        result.q[0]
    );
    assert!(result.status.is_converged(), "IK debe converger en 1-DOF");
    assert!(
        result.iterations <= 100,
        "No debe exceder max_iters, usó {}",
        result.iterations
    );
}

// ─── Jacobian verification ────────────────────────────────────────────

/// Jacobiano geométrico vs. numérico (diferencias finitas).
/// Verifica cada columna del Jacobiano lineal en 5 configuraciones.
#[test]
fn jacobian_matches_numerical() {
    let (fk, ee) = build_2dof_planar_arm();

    let geometric = GeometricJacobian::new(fk.clone(), ee.clone());
    let numerical = NumericalJacobian::new(fk, ee);

    let test_configs: Vec<Vec<f64>> = vec![
        vec![0.0, 0.0],
        vec![0.5, 0.3],
        vec![1.0, -0.5],
        vec![-0.8, 1.2],
        vec![PI / 4.0, PI / 3.0],
    ];

    let tolerance = 1e-4;

    for q in &test_configs {
        let j_geom = geometric.evaluate(q);
        let j_num = numerical.evaluate(q);

        for i in 0..3 {
            for j in 0..q.len() {
                let g = j_geom.linear()[(i, j)];
                let n = j_num.linear()[(i, j)];
                let diff = (g - n).abs();
                assert!(
                    diff < tolerance,
                    "Jacobiano mismatch en q = [{:.3}, {:.3}]: \
                        J_geom[{}][{}] = {:.6}, J_num[{}][{}] = {:.6}, diff = {:.2e}",
                    q[0],
                    q[1],
                    i,
                    j,
                    g,
                    i,
                    j,
                    n,
                    diff
                );
            }
        }
    }
}

// ─── Unreachable targets ──────────────────────────────────────────────

/// Target fuera del workspace → MaxIterations, error = distancia.
///
/// Brazo 2R con L1=L2=1: reach max = 2. Target (3, 0, 0) está a 1 más
/// allá. Desde q=[0,0] el error es radial puro → J^T·e = 0 → solver no
/// se mueve → test determinista.
#[test]
fn unreachable_target_returns_max_iterations() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5);

    let q0 = vec![0.0, 0.0];
    let target = Vector3::new(3.0, 0.0, 0.0);
    let result = solver
        .solve(&q0, IKGoal::Position(target))
        .expect("JT solve should succeed");

    assert_eq!(
        result.status,
        IKStatus::MaxIterations,
        "Unreachable target debe dar MaxIterations, status={:?}, error={:.4}",
        result.status,
        result.final_error
    );

    // reach_max = 2, target_distance = 3 → min_error = 1
    let expected_min_error = 1.0;
    assert!(
        (result.final_error - expected_min_error).abs() < 1e-12,
        "final_error ({:.4}) debe ser exactamente la distancia inalcanzable ({:.4}) \
            cuando q0 = [0,0] (singular)",
        result.final_error,
        expected_min_error
    );

    // Solver no modificó q (singularidad → dq = 0)
    assert_eq!(result.q[0], 0.0, "q1 no debe cambiar desde q0 = [0,0]");
    assert_eq!(result.q[1], 0.0, "q2 no debe cambiar desde q0 = [0,0]");

    // Sin NaN ni Inf
    for (i, &q_val) in result.q.iter().enumerate() {
        assert!(q_val.is_finite(), "q[{}] debe ser finito, got {}", i, q_val);
    }
}

/// Múltiples targets inalcanzables: error escala con la distancia.
#[test]
fn unreachable_target_error_equals_distance() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5);

    let max_reach = 2.0;

    let test_cases = [
        (Vector3::new(3.0, 0.0, 0.0), 3.0),
        (Vector3::new(4.0, 0.0, 0.0), 4.0),
        (Vector3::new(2.5, 0.0, 0.0), 2.5),
    ];

    for (target, target_distance) in test_cases {
        let result = solver
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("JT solve should succeed");

        assert_eq!(
            result.status,
            IKStatus::MaxIterations,
            "Target a distancia {:.1} debe dar MaxIterations",
            target_distance
        );

        let expected_min_error = target_distance - max_reach;
        assert!(
            (result.final_error - expected_min_error).abs() < 1e-12,
            "final_error ({:.4}) debe ser exactamente {} para target a distancia {:.1}",
            result.final_error,
            expected_min_error,
            target_distance
        );

        assert_eq!(result.q[0], 0.0, "q1 no debe cambiar");
        assert_eq!(result.q[1], 0.0, "q2 no debe cambiar");

        for &q_val in &result.q {
            assert!(q_val.is_finite(), "q debe ser finito, got {}", q_val);
        }
    }
}

/// Robuster: targets inalcanzables no producen explosión numérica.
#[test]
fn unreachable_target_does_not_explode() {
    let (fk, ee) = build_2dof_planar_arm();

    let start_configs = [
        vec![0.0, 0.0],
        vec![1.0, 0.5],
        vec![-0.8, 1.2],
        vec![PI, -0.5],
    ];

    let targets = [
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(5.0, 5.0, 0.0),
    ];

    for q0 in &start_configs {
        for &target in &targets {
            let solver = JacobianTransposeSolver::new(fk.clone(), ee.clone(), 200, 1e-6, 0.5);
            let result = solver
                .solve(q0, IKGoal::Position(target))
                .expect("JT solve should succeed");

            assert_eq!(
                result.status,
                IKStatus::MaxIterations,
                "Target inalcanzable debe dar MaxIterations, no converger"
            );

            for &q_val in &result.q {
                assert!(
                    q_val.is_finite(),
                    "q debe ser finito para q0={:?}, target=({},{}), got {}",
                    q0,
                    target.x,
                    target.y,
                    q_val
                );
            }

            assert!(
                result.final_error.is_finite(),
                "final_error debe ser finito, got {}",
                result.final_error
            );
        }
    }
}

// ─── Singularities ────────────────────────────────────────────────────

/// Error radial desde q=[0,0] (brazo extendido) produce J^T·e = 0 →
/// el error no decrece → solver atascado. Misma config no-singular
/// converge sin problema.
#[test]
fn singular_radial_error_blocks_convergence() {
    let (fk, ee) = build_2dof_planar_arm();
    let target = Vector3::new(1.5, 0.0, 0.0);

    // Desde singular: J^T · e = 0 → stuck
    let solver_sing = JacobianTransposeSolver::new(fk.clone(), ee.clone(), 100, 1e-6, 0.5);
    let result_singular = solver_sing
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("JT solve should succeed");

    // Desde no-singular: converge
    let solver_nonsing = JacobianTransposeSolver::new(fk, ee, 100, 1e-6, 0.5);
    let result_nonsingular = solver_nonsing
        .solve(&[PI / 4.0, PI / 4.0], IKGoal::Position(target))
        .expect("JT solve should succeed");

    println!(
        "  singular (q=[0,0]):       {} iter, error = {:.2e}, q = [{:.4}, {:.4}], status={:?}",
        result_singular.iterations,
        result_singular.final_error,
        result_singular.q[0],
        result_singular.q[1],
        result_singular.status
    );
    println!(
        "  no-singular (q=[π/4,π/4]): {} iter, error = {:.2e}, q = [{:.4}, {:.4}], status={:?}",
        result_nonsingular.iterations,
        result_nonsingular.final_error,
        result_nonsingular.q[0],
        result_nonsingular.q[1],
        result_nonsingular.status
    );

    assert_eq!(
        result_singular.status,
        IKStatus::MaxIterations,
        "Singular con error radial no debe converger"
    );

    let initial_error = (target - Vector3::new(2.0, 0.0, 0.0)).magnitude();
    assert!(
        (result_singular.final_error - initial_error).abs() < 1e-12,
        "Error singular ({:.4}) debe ser ≈ error inicial ({:.4})",
        result_singular.final_error,
        initial_error
    );

    assert!(
        result_nonsingular.status.is_converged(),
        "No-singular debe converger, status={:?}",
        result_nonsingular.status
    );
}

/// Historial monotónico en singularidad: error nunca aumenta.
#[test]
fn singular_config_error_history_monotonic() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5).with_history(true);

    let target = Vector3::new(1.2, 0.5, 0.0);
    let result = solver
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("JT solve should succeed");

    assert!(
        result.status.is_converged(),
        "Debe converger: status={:?}",
        result.status
    );

    let history = result
        .error_history
        .expect("with_history(true) debe registrar historial");

    for i in 0..history.len() - 1 {
        assert!(
            history[i + 1] <= history[i] + 1e-12,
            "Error aumentó en iteración {}: {:.6e} → {:.6e}",
            i,
            history[i],
            history[i + 1]
        );
    }

    println!(
        "  Historial monotónico verificado ({} iteraciones)",
        history.len()
    );
}

/// Sin oscilación: diferencias consecutivas de error decrecen.
#[test]
fn singular_config_no_oscillation() {
    let (fk, ee) = build_2dof_planar_arm();
    let solver = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.5).with_history(true);

    let target = Vector3::new(1.2, 0.5, 0.0);
    let result = solver
        .solve(&[0.0, 0.0], IKGoal::Position(target))
        .expect("JT solve should succeed");

    assert!(
        result.status.is_converged(),
        "Debe converger: status={:?}",
        result.status
    );

    let history = result
        .error_history
        .expect("with_history(true) debe registrar historial");

    // diff[i] = error[i] - error[i+1] (positivo porque error decrece)
    let diffs: Vec<f64> = history.windows(2).map(|w| w[0] - w[1]).collect();

    assert!(
        diffs.len() >= 10,
        "Muy pocas iteraciones ({}) para evaluar oscilación",
        diffs.len()
    );

    let n = diffs.len();
    let first_third: &[f64] = &diffs[..n / 3];
    let last_third: &[f64] = &diffs[2 * n / 3..];

    let mean_first: f64 = first_third.iter().sum::<f64>() / first_third.len() as f64;
    let mean_last: f64 = last_third.iter().sum::<f64>() / last_third.len() as f64;

    println!(
        "  diff medio (1er tercio) = {:.6e}, diff medio (3er tercio) = {:.6e}",
        mean_first, mean_last
    );

    assert!(
        mean_last < mean_first * 0.5,
        "Las diferencias de error no decrecen lo suficiente: \
         1er tercio={:.6e}, 3er tercio={:.6e}",
        mean_first,
        mean_last
    );
}
