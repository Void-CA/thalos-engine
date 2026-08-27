use crate::models::manipulator_3dof::Manipulator3DOFSpec;
use crate::prelude::*;
use thalos_math::constants::*;
use thalos_math::*;

fn fresh_pair() -> (GeometricJacobian, NumericalJacobian) {
    let robot = Manipulator3DOFSpec::ideal().build();
    let ee = robot.end_effector().clone();
    let fk1 = ForwardKinematics::new(robot.clone());
    let fk2 = ForwardKinematics::new(robot);
    let geo = GeometricJacobian::new(fk1, ee.clone());
    let num = NumericalJacobian::new(fk2, ee);
    (geo, num)
}

#[test]
fn geometric_matches_numerical_at_zero_config() {
    let (geo, num) = fresh_pair();
    let q = [0.0, 0.0, 0.0];

    let jg = geo.evaluate(&q);
    let jn = num.evaluate(&q);

    for r in 0..3 {
        for c in 0..3 {
            assert!(
                (jg.linear[(r, c)] - jn.linear()[(r, c)]).abs() < 1e-5,
                "Linear mismatch at ({},{}): geo={}, num={}",
                r,
                c,
                jg.linear[(r, c)],
                jn.linear()[(r, c)]
            );
        }
    }
}

#[test]
fn geometric_matches_numerical_at_multiple_configs() {
    let configs = [
        [0.0, 0.0, 0.0],
        [PI / 4.0, -PI / 6.0, PI / 8.0],
        [0.3, 0.5, -0.4],
        [-PI / 5.0, PI / 7.0, -PI / 3.0],
        [PI / 2.0, -PI / 4.0, 0.0],
    ];

    for q in configs {
        let (geo, num) = fresh_pair();
        let jg = geo.evaluate(&q);
        let jn = num.evaluate(&q);

        for r in 0..3 {
            for c in 0..3 {
                assert!(
                    (jg.linear[(r, c)] - jn.linear()[(r, c)]).abs() < 1e-5,
                    "Linear mismatch at q={:?}, ({},{}): geo={}, num={}",
                    q,
                    r,
                    c,
                    jg.linear[(r, c)],
                    jn.linear()[(r, c)]
                );
            }
        }
    }
}

#[test]
fn at_zero_config_analytical_values() {
    // Z-up: l1=l2=l3=1.
    //   Joint 1 = Z-axis (yaw),  joints 2/3 = Y-axis (pitch)
    //   p_1 = (0, 0, 0),  ω_1 = (0, 0, 1)
    //   p_2 = (0, 0, 1),  ω_2 = (0, 1, 0)
    //   p_3 = (1, 0, 1),  ω_3 = (0, 1, 0)
    //   p_ee = (2, 0, 1)
    //
    // J_v[:, i] = ω_i × (p_ee - p_i):
    //   col 0 = (0, 0, 1) × (2, 0, 1) = (0, 2, 0)
    //   col 1 = (0, 1, 0) × (2, 0, 0) = (0, 0, -2)
    //   col 2 = (0, 1, 0) × (1, 0, 0) = (0, 0, -1)
    //
    // Singularidad estructural: columna X de J_v es toda cero.
    let (geo, _) = fresh_pair();
    let j = geo.evaluate(&[0.0, 0.0, 0.0]);

    // Lineal
    assert!(
        j.linear[(0, 0)].abs() < EPS,
        "dx/dq1 should be 0, got {}",
        j.linear[(0, 0)]
    );
    assert!(
        (j.linear[(1, 0)] - 2.0).abs() < EPS,
        "dy/dq1 should be 2, got {}",
        j.linear[(1, 0)]
    );
    assert!(
        j.linear[(2, 0)].abs() < EPS,
        "dz/dq1 should be 0, got {}",
        j.linear[(2, 0)]
    );

    assert!(
        j.linear[(0, 1)].abs() < EPS,
        "dx/dq2 should be 0, got {}",
        j.linear[(0, 1)]
    );
    assert!(
        j.linear[(1, 1)].abs() < EPS,
        "dy/dq2 should be 0, got {}",
        j.linear[(1, 1)]
    );
    assert!(
        (j.linear[(2, 1)] - -2.0).abs() < EPS,
        "dz/dq2 should be -2, got {}",
        j.linear[(2, 1)]
    );

    assert!(
        j.linear[(0, 2)].abs() < EPS,
        "dx/dq3 should be 0, got {}",
        j.linear[(0, 2)]
    );
    assert!(
        j.linear[(1, 2)].abs() < EPS,
        "dy/dq3 should be 0, got {}",
        j.linear[(1, 2)]
    );
    assert!(
        (j.linear[(2, 2)] - -1.0).abs() < EPS,
        "dz/dq3 should be -1, got {}",
        j.linear[(2, 2)]
    );

    // Angular: cada joint aporta su eje en frame mundo
    // Joint 1 = Z → ω = (0, 0, 1)
    assert!(
        j.angular[(0, 0)].abs() < EPS && j.angular[(1, 0)].abs() < EPS,
        "ωx,ωy/dq1 should be 0"
    );
    assert!((j.angular[(2, 0)] - 1.0).abs() < EPS, "ωz/dq1 should be 1");

    // Joint 2 = Y → ω = (0, 1, 0)
    assert!(
        j.angular[(0, 1)].abs() < EPS && j.angular[(2, 1)].abs() < EPS,
        "ωx,ωz/dq2 should be 0"
    );
    assert!((j.angular[(1, 1)] - 1.0).abs() < EPS, "ωy/dq2 should be 1");

    // Joint 3 = Y → ω = (0, 1, 0)
    assert!(
        j.angular[(0, 2)].abs() < EPS && j.angular[(2, 2)].abs() < EPS,
        "ωx,ωz/dq3 should be 0"
    );
    assert!((j.angular[(1, 2)] - 1.0).abs() < EPS, "ωy/dq3 should be 1");
}

#[test]
fn at_arm_vertical_analytical_values() {
    // Z-up: q = (0, -π/2, 0): brazo alineado con +Z mundial.
    //   p_1 = (0, 0, 0),  ω_1 = (0, 0, 1)   (z_axis)
    //   p_2 = (0, 0, 1),  ω_2 = (0, 1, 0)   (y_axis)
    //   p_3 = (1, 0, 1),  ω_3 = (0, 1, 0)   (y_axis)
    //   p_ee = (0, 0, 3)
    //
    // J_v col 0 = (0, 0, 1) × (0, 0, 3) = (0, 0, 0)  ← singular
    // J_v col 1 = (0, 1, 0) × (0, 0, 2) = (2, 0, 0)
    // J_v col 2 = (0, 1, 0) × (0, 0, 1) = (1, 0, 0)
    let (geo, _) = fresh_pair();
    let j = geo.evaluate(&[0.0, -PI / 2.0, 0.0]);

    for r in 0..3 {
        assert!(
            j.linear[(r, 0)].abs() < EPS,
            "J_v[r, 0] should be 0 (singular), got row {}: {}",
            r,
            j.linear[(r, 0)]
        );
    }
    assert!((j.linear[(0, 1)] - 2.0).abs() < EPS, "dx/dq2 should be 2");
    assert!((j.linear[(0, 2)] - 1.0).abs() < EPS, "dx/dq3 should be 1");
    assert!(j.linear[(1, 1)].abs() < EPS && j.linear[(2, 1)].abs() < EPS);
    assert!(j.linear[(1, 2)].abs() < EPS && j.linear[(2, 2)].abs() < EPS);
}

#[test]
fn singularity_detected_via_jjt_determinant() {
    let (geo, _) = fresh_pair();

    // Singular 1: q = (0, 0, 0) — brazo en X, columna X de J_v nula
    let j_sing = geo.evaluate(&[0.0, 0.0, 0.0]);
    let jjt_sing = &j_sing.linear * &j_sing.linear.transpose();
    let det_sing = jjt_sing.determinant();
    assert!(
        det_sing.abs() < 1e-6,
        "Singular config should have det~0, got {}",
        det_sing
    );

    // Singular 2: q = (0, -π/2, 0) — brazo en -Y, q1 no genera vel lineal
    let j_vert = geo.evaluate(&[0.0, -PI / 2.0, 0.0]);
    let jjt_vert = &j_vert.linear * &j_vert.linear.transpose();
    let det_vert = jjt_vert.determinant();
    assert!(
        det_vert.abs() < 1e-6,
        "Vertical config should be singular, got {}",
        det_vert
    );

    // No singular: q con todas las juntas contribuyendo
    let j_ok = geo.evaluate(&[PI / 4.0, -PI / 4.0, PI / 6.0]);
    let jjt_ok = &j_ok.linear * &j_ok.linear.transpose();
    let det_ok = jjt_ok.determinant();
    assert!(
        det_ok.abs() > 1e-3,
        "Non-singular config should have det>0, got {}",
        det_ok
    );
}

#[test]
fn propagates_velocities_via_geometric_jacobian() {
    // v_ee = J · q̇ debe coincidir con finite difference de la FK.
    let (geo, fk) = {
        let robot = Manipulator3DOFSpec::ideal().build();
        let ee = robot.end_effector().clone();
        let fk = ForwardKinematics::new(robot);
        let geo = GeometricJacobian::new(fk.clone(), ee);
        (geo, fk)
    };

    let q = [PI / 4.0, -PI / 6.0, PI / 8.0];
    let q_dot = [0.3, 0.2, 0.15];
    let dt = 1e-5;

    let j = geo.evaluate(&q);
    let v_pred = &j.linear * DynamicVector::from_vec(q_dot.to_vec());

    let q_next = [
        q[0] + q_dot[0] * dt,
        q[1] + q_dot[1] * dt,
        q[2] + q_dot[2] * dt,
    ];

    let ee = fk.robot().end_effector().clone();
    let p_curr = fk.evaluate(&q).pose(&ee).unwrap().transform().translation;
    let p_next = fk
        .evaluate(&q_next)
        .pose(&ee)
        .unwrap()
        .transform()
        .translation;
    let v_actual = [
        (p_next.x - p_curr.x) / dt,
        (p_next.y - p_curr.y) / dt,
        (p_next.z - p_curr.z) / dt,
    ];

    for axis in 0..3 {
        assert!(
            (v_pred[axis] - v_actual[axis]).abs() < 1e-4,
            "Axis {}: predicted {}, actual {}",
            axis,
            v_pred[axis],
            v_actual[axis]
        );
    }
}

#[test]
fn angular_velocity_at_canonical_config() {
    // Z-up: joint 1 = Z, joints 2/3 = Y
    // → ω = q̇₁·ẑ + q̇₂·ŷ + q̇₃·ŷ = (0, q̇₂+q̇₃, q̇₁)
    let (geo, _) = fresh_pair();
    let j = geo.evaluate(&[0.0, 0.0, 0.0]);

    let q_dot = [0.3, 0.2, 0.15];
    let omega_pred = &j.angular * DynamicVector::from_vec(q_dot.to_vec());

    let expected_x = 0.0;
    let expected_y = q_dot[1] + q_dot[2];
    let expected_z = q_dot[0];

    assert!(
        (omega_pred[0] - expected_x).abs() < EPS,
        "ωx: expected {}, got {}",
        expected_x,
        omega_pred[0]
    );
    assert!(
        (omega_pred[1] - expected_y).abs() < EPS,
        "ωy: expected {}, got {}",
        expected_y,
        omega_pred[1]
    );
    assert!(
        (omega_pred[2] - expected_z).abs() < EPS,
        "ωz: expected {}, got {}",
        expected_z,
        omega_pred[2]
    );
}

#[test]
fn angular_block_has_unit_axes() {
    // Cada columna de J_ω debe ser un versor (norma = 1) — son ejes
    // de rotación, no contribuciones compuestas.
    let (geo, _) = fresh_pair();
    let configs = [
        [0.0, 0.0, 0.0],
        [PI / 3.0, 0.0, 0.0],
        [0.0, -PI / 4.0, 0.0],
        [PI / 4.0, -PI / 4.0, PI / 6.0],
    ];

    for q in configs {
        let j = geo.evaluate(&q);

        for col in 0..3 {
            let norm_sq = j.angular[(0, col)].powi(2)
                + j.angular[(1, col)].powi(2)
                + j.angular[(2, col)].powi(2);
            assert!(
                (norm_sq - 1.0).abs() < 1e-9,
                "Angular column {} should be a unit vector at q={:?}, got |ω|² = {}",
                col,
                q,
                norm_sq
            );
        }
    }
}
