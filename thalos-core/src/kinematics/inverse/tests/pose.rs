use super::*;
use crate::spatial::pose::Pose;
use thalos_math::UnitQuaternion;

fn make_target_pose(ee: FrameId, q: &[f64]) -> Pose {
    let x = q[0].cos() + (q[0] + q[1]).cos();
    let y = q[0].sin() + (q[0] + q[1]).sin();
    let pos = Vector3::new(x, y, 0.0);

    let rot = UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), q[0] + q[1]);

    let transform = thalos_math::Transform3D {
        translation: pos,
        rotation: rot,
    };

    Pose::new(FrameId::World, ee, transform)
}

/// Error de orientación entre dos UnitQuaternions como ángulo absoluto.
fn orientation_angle(a: &UnitQuaternion, b: &UnitQuaternion) -> f64 {
    let rel = *b * a.inverse();
    let v_norm = Vector3::new(rel.inner().x, rel.inner().y, rel.inner().z).magnitude();
    let w = rel.inner().w.clamp(-1.0, 1.0);
    if v_norm < 1e-14 {
        0.0
    } else {
        2.0 * v_norm.atan2(w)
    }
}

// ─── DLS ──────────────────────────────────────────────────────────────

/// DLS converge a una pose conocida desde q0=[0,0].
#[test]
fn dls_pose_ik_reaches_known_pose() {
    let (fk, ee) = build_2dof_planar_arm();
    let target_pose = make_target_pose(ee, &[PI / 4.0, PI / 3.0]);
    let solver = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1);

    let result = solver
        .solve(&[0.0, 0.0], IKGoal::Pose(target_pose))
        .expect("DLS solve should succeed");

    assert!(
        result.status.is_converged(),
        "DLS debe converger para pose alcanzable: status={:?}, error={:.2e}, iter={}",
        result.status,
        result.final_error,
        result.iterations
    );
    assert!(
        result.final_error < 1e-4,
        "Error final ({:.2e}) demasiado alto para pose IK",
        result.final_error
    );
    assert!(
        result.iterations < 500,
        "No debe agotar max_iters: {}",
        result.iterations
    );

    println!(
        "  DLS pose IK: {} iter, error final = {:.2e}, q = [{:.6}, {:.6}]",
        result.iterations, result.final_error, result.q[0], result.q[1]
    );
}

/// Consistencia FK/IK para pose: IK(FK(q)) ≈ q para pose completa.
#[test]
fn pose_ik_fk_consistency() {
    let (fk, ee) = build_2dof_planar_arm();

    let q_orig = vec![0.5, 1.0];
    let target_pose = make_target_pose(ee, &q_orig);

    let solver = DampedLeastSquaresSolver::new(fk.clone(), ee, 500, 1e-6, 0.1);
    let result = solver
        .solve(&[0.0, 0.0], IKGoal::Pose(target_pose.clone()))
        .expect("DLS solve should succeed");

    assert!(
        result.status.is_converged(),
        "DLS debe converger: status={:?}, error={:.2e}",
        result.status,
        result.final_error
    );

    // FK desde la solución IK
    let fk_result = fk.evaluate(&result.q);
    let reached_pose = fk_result.ee_pose().unwrap();

    // Error de posición
    let pos_error = (target_pose.translation() - reached_pose.translation()).magnitude();
    println!("  Error de posición FK/IK: {:.2e}", pos_error);
    assert!(
        pos_error < 1e-4,
        "Error de posición FK/IK ({:.2e}) excede tolerancia",
        pos_error
    );

    // Error de orientación
    let r_target = target_pose.transform().rotation;
    let r_reached = reached_pose.transform().rotation;
    let orient_error = orientation_angle(&r_target, &r_reached);
    println!("  Error de orientación FK/IK: {:.2e} rad", orient_error);
    assert!(
        orient_error < 1e-4,
        "Error de orientación FK/IK ({:.2e} rad) excede tolerancia",
        orient_error
    );

    println!(
        "  q_original = [{:.6}, {:.6}], q_ik = [{:.6}, {:.6}]",
        q_orig[0], q_orig[1], result.q[0], result.q[1]
    );
}

/// DLS converge más rápido que JT para pose desde q0=[0,0] (el damping
/// evita el overshoot en el espacio 6D). JT necesita α más chico.
#[test]
fn pose_faster_than_jt_from_singular() {
    let (fk, ee) = build_2dof_planar_arm();

    let target_pose = make_target_pose(ee, &[PI / 3.0, PI / 6.0]);

    let dls = DampedLeastSquaresSolver::new(fk.clone(), ee.clone(), 500, 1e-6, 0.1);
    let r_dls = dls
        .solve(&[0.0, 0.0], IKGoal::Pose(target_pose.clone()))
        .expect("DLS solve should succeed");

    // JT con α más chico para evitar overshoot en 6D
    let jt = JacobianTransposeSolver::new(fk, ee, 500, 1e-6, 0.1);
    let r_jt = jt
        .solve(&[0.0, 0.0], IKGoal::Pose(target_pose))
        .expect("JT solve should succeed");

    assert!(
        r_dls.status.is_converged(),
        "DLS debe converger: status={:?}",
        r_dls.status
    );

    // DLS debe converger en menos iteraciones (o al menos no más)
    assert!(
        r_dls.iterations <= r_jt.iterations,
        "DLS ({}) debe converger en ≤ iteraciones que JT ({}) para pose IK",
        r_dls.iterations,
        r_jt.iterations
    );

    println!(
        "  [POSE] DLS: {} iter, error={:.2e} | JT: {} iter, error={:.2e} | ratio={:.3}",
        r_dls.iterations,
        r_dls.final_error,
        r_jt.iterations,
        r_jt.final_error,
        r_dls.iterations as f64 / r_jt.iterations as f64
    );
}

// ─── DLS converge donde position-IK se estanca ────────────────────────

/// DLS con pose completa converge desde q=[0,0] con target radial puro
/// mientras que position IK se estanca.
///
/// Para un brazo 2R, un punto (x, 0, 0) en el eje X con orientación φ
/// es alcanzable solo cuando x = 2·cos(φ). Usamos φ = π/4 → x = √2.
/// Position IK no puede moverse (J_lin^T·e=0), pero pose IK tiene
/// gradiente de orientación en el Jacobiano completo 6×n.
#[test]
fn pose_converges_where_position_ik_stagnates() {
    let (fk, ee) = build_2dof_planar_arm();

    // Target radial: posición (√2, 0, 0) con orientación Z-45°.
    // Es alcanzable (q = [-π/4, π/2] lo satisface).
    let target_x = 2.0 * (PI / 4.0).cos(); // √2 ≈ 1.414
    let pos_target = Vector3::new(target_x, 0.0, 0.0);
    let pose_target = {
        let transform = thalos_math::Transform3D {
            translation: pos_target,
            rotation: UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), PI / 4.0),
        };
        Pose::new(FrameId::World, ee, transform)
    };

    // DLS con posición sola → se estanca (error radial puro)
    let dls_pos = DampedLeastSquaresSolver::new(fk.clone(), ee.clone(), 200, 1e-6, 0.1);
    let r_pos = dls_pos
        .solve(&[0.0, 0.0], IKGoal::Position(pos_target))
        .expect("DLS solve should succeed");

    // DLS con pose → converge (gradiente de orientación)
    let dls_pose = DampedLeastSquaresSolver::new(fk, ee, 500, 1e-6, 0.1);
    let r_pose = dls_pose
        .solve(&[0.0, 0.0], IKGoal::Pose(pose_target))
        .expect("DLS solve should succeed");

    println!(
        "  position-ik: status={:?}, error={:.2e}, {} iter",
        r_pos.status, r_pos.final_error, r_pos.iterations
    );
    println!(
        "  pose-ik:     status={:?}, error={:.2e}, {} iter",
        r_pose.status, r_pose.final_error, r_pose.iterations
    );

    // Position IK se estanca (error radial puro → sin gradiente lineal)
    assert_eq!(
        r_pos.status,
        IKStatus::MaxIterations,
        "Position IK debe estancarse en singularidad radial"
    );

    // Pose IK converge (el gradiente de orientación da descenso)
    assert!(
        r_pose.status.is_converged(),
        "Pose IK debe converger (tiene gradiente de orientación en J_full)"
    );
}
