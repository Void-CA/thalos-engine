use crate::models::manipulator_3dof::Manipulator3DOFSpec;
use crate::prelude::*;
use thalos_math::constants::*;
use thalos_math::*;

fn setup() -> (NumericalJacobian, ForwardKinematics, FrameId) {
    let robot = Manipulator3DOFSpec::ideal().build();
    let ee = robot.end_effector().clone();
    let fk = ForwardKinematics::new(robot);
    let jacobian = NumericalJacobian::new(fk.clone(), ee.clone());
    (jacobian, fk, ee)
}

#[test]
fn dimensions_are_correct() {
    let (jacobian, _, _) = setup();
    let q = [0.1, -0.2, 0.3];
    let j = jacobian.evaluate(&q);

    assert_eq!(j.linear().nrows(), 3, "Linear part: 3 rows (x, y, z)");
    assert_eq!(j.linear().ncols(), 3, "Linear part: 3 columns (3 joints)");
    assert_eq!(j.angular().nrows(), 3, "Angular part: 3 rows");
    assert_eq!(j.angular().ncols(), 3, "Angular part: 3 columns");
}

#[test]
fn predicts_small_motion_for_each_joint() {
    let (jacobian, fk, ee) = setup();
    let q = [0.3, -0.4, 0.2];
    let j = jacobian.evaluate(&q);

    for joint_idx in 0..3 {
        let mut dq = vec![0.0, 0.0, 0.0];
        dq[joint_idx] = 1e-6;

        let dx_pred = j.linear() * DynamicVector::from_vec(dq.clone());

        let q2 = [q[0] + dq[0], q[1] + dq[1], q[2] + dq[2]];

        let p1 = fk.evaluate(&q).pose(&ee).unwrap().transform().translation;
        let p2 = fk.evaluate(&q2).pose(&ee).unwrap().transform().translation;

        let dx_actual = DynamicVector::from_vec(vec![p2.x - p1.x, p2.y - p1.y, p2.z - p1.z]);
        for axis in 0..3 {
            assert!(
                (dx_pred[axis] - dx_actual[axis]).abs() < 1e-5,
                "Joint {}, axis {}: predicted {}, actual {}",
                joint_idx + 1,
                axis,
                dx_pred[axis],
                dx_actual[axis]
            );
        }
    }
}

#[test]
fn velocity_matches_finite_difference() {
    let (jacobian, fk, ee) = setup();
    let q = [0.4, -0.5, 0.6];
    let q_dot = [0.3, 0.2, 0.15];
    let dt = 1e-5;

    let j = jacobian.evaluate(&q);
    let v_pred = j.linear() * DynamicVector::from_vec(q_dot.to_vec());

    let q_next = [
        q[0] + q_dot[0] * dt,
        q[1] + q_dot[1] * dt,
        q[2] + q_dot[2] * dt,
    ];

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
fn at_zero_config_only_xz_and_y_motion() {
    // Z-up: joint 1 = Z-axis (yaw), joints 2/3 = Y-axis (pitch).
    // ee at (l2+l3, 0, l1) = (2, 0, 1).
    //
    // J_v0 = (0,0,1) × (2,0,1) = (0, 2, 0) → solo afecta Y
    // J_v1 = (0,1,0) × (1,0,0) = (0, 0, -2) → solo afecta Z
    // J_v2 = (0,1,0) × (0,0,0) = (0, 0, -1) → solo afecta Z
    //
    // Columna X del Jacobiano es toda cero (singular en 1D).
    let (jacobian, _, _) = setup();
    let j = jacobian.evaluate(&[0.0, 0.0, 0.0]);

    // Columna de q1 (eje Z): velocidad en Y
    assert!(j.linear()[(0, 0)].abs() < 1e-6, "dx/dq1 should be 0");
    assert!(
        j.linear()[(1, 0)].abs() > 0.5,
        "dy/dq1 should be non-zero (l2+l3=2)"
    );
    assert!(j.linear()[(2, 0)].abs() < 1e-6, "dz/dq1 should be 0");

    // Columnas de q2 y q3 (eje Y): velocidad en Z
    for col in 1..3 {
        assert!(
            j.linear()[(0, col)].abs() < 1e-6,
            "dx/dq{} should be 0",
            col + 1
        );
        assert!(
            j.linear()[(1, col)].abs() < 1e-6,
            "dy/dq{} should be 0",
            col + 1
        );
        assert!(
            j.linear()[(2, col)].abs() > 0.5,
            "dz/dq{} should be significant",
            col + 1
        );
    }
}

#[test]
fn zero_config_is_singular() {
    // det(J · J^T) = 0 cuando la columna de X es nula.
    let (jacobian, _, _) = setup();
    let j = jacobian.evaluate(&[0.0, 0.0, 0.0]);
    let jjt = j.linear() * j.linear().transpose();
    let det = jjt.determinant();

    assert!(
        det.abs() < 1e-6,
        "J·J^T determinant at q=(0,0,0) should be ~0 (singular), got {}",
        det
    );
}

#[test]
fn vertical_arm_is_also_singular() {
    // q = (0, -π/2, 0) deja el brazo alineado con -Y mundial.
    // Joint 1 (eje Y) queda colineal con el brazo: no genera velocidad lineal.
    let (jacobian, _, _) = setup();
    let j = jacobian.evaluate(&[0.0, -PI / 2.0, 0.0]);

    // Columna de q1 debe ser (0, 0, 0)
    for axis in 0..3 {
        assert!(
            j.linear()[(axis, 0)].abs() < 1e-6,
            "d(axis {})/dq1 should be 0 when arm is vertical, got {}",
            axis,
            j.linear()[(axis, 0)]
        );
    }
}

#[test]
fn linearity_holds() {
    let (jacobian, _, _) = setup();
    let q = [0.4, -0.3, 0.2];
    let j = jacobian.evaluate(&q);

    let v1 = [0.1, 0.2, 0.15];
    let v2 = [0.05, 0.15, 0.1];
    let (a, b) = (2.0, 3.0);

    let v_combined = [
        a * v1[0] + b * v2[0],
        a * v1[1] + b * v2[1],
        a * v1[2] + b * v2[2],
    ];
    let jv_combined = j.linear() * DynamicVector::from_vec(v_combined.to_vec());
    let jv1 = j.linear() * DynamicVector::from_vec(v1.to_vec());
    let jv2 = j.linear() * DynamicVector::from_vec(v2.to_vec());
    let jv_linear = a * jv1 + b * jv2;

    for axis in 0..3 {
        assert!(
            (jv_combined[axis] - jv_linear[axis]).abs() < 1e-10,
            "Linearity fails on axis {}: combined {}, linear {}",
            axis,
            jv_combined[axis],
            jv_linear[axis]
        );
    }
}

#[test]
fn reconstruction_from_motion_at_multiple_configs() {
    // Verifica J · q̇ ≈ Δp/Δt en varias configuraciones no singulares.
    let (jacobian, fk, ee) = setup();
    let test_configs = [
        ([0.0, -PI / 4.0, 0.0], [0.1, 0.05, 0.0]),
        ([PI / 4.0, -PI / 6.0, PI / 8.0], [0.2, 0.1, 0.15]),
        ([0.3, 0.4, 0.5], [0.1, 0.2, 0.3]),
        ([-PI / 6.0, PI / 5.0, -PI / 7.0], [0.15, 0.25, 0.1]),
    ];
    let dt = 1e-5;

    for (q, q_dot) in test_configs {
        let j = jacobian.evaluate(&q);
        let v_pred = j.linear() * DynamicVector::from_vec(q_dot.to_vec());

        let q_next = [
            q[0] + q_dot[0] * dt,
            q[1] + q_dot[1] * dt,
            q[2] + q_dot[2] * dt,
        ];

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
            let error = (v_pred[axis] - v_actual[axis]).abs();
            assert!(
                error < 1e-4,
                "q={:?}, q̇={:?}, axis {}: predicted {}, actual {}, error {}",
                q,
                q_dot,
                axis,
                v_pred[axis],
                v_actual[axis],
                error
            );
        }
    }
}

#[test]
fn base_yaw_analytical_formula() {
    // Z-up: joint 1 es eje Z. ω = (0, 0, 1). v = ω × p = (-y, x, 0).
    //
    // En q = (π/2, -π/4, 0), el efector está en (1.414214, 1.414214, 1.0)
    // (aproximadamente: l1=1, l2=l3=1, Rz(π/2) rota (2,0) a (0,2), luego
    // Ry(-π/4) del hombro inclina los links 2+3 hacia XZ).
    let (jacobian, fk, ee) = setup();
    let q = [PI / 2.0, -PI / 4.0, 0.0];
    let j = jacobian.evaluate(&q);

    let t = fk.evaluate(&q).pose(&ee).unwrap().transform().translation;

    // ω = (0,0,1): v = (-y_ee, x_ee, 0)
    assert!(
        (j.linear()[(0, 0)] - (-t.y)).abs() < 1e-4,
        "dx/dq1 should be -y_ee = {}, got {}",
        -t.y,
        j.linear()[(0, 0)]
    );
    assert!(
        (j.linear()[(1, 0)] - t.x).abs() < 1e-4,
        "dy/dq1 should be x_ee = {}, got {}",
        t.x,
        j.linear()[(1, 0)]
    );
    assert!(
        j.linear()[(2, 0)].abs() < 1e-6,
        "dz/dq1 should be 0 (Z rotation doesn't change z), got {}",
        j.linear()[(2, 0)]
    );
}
