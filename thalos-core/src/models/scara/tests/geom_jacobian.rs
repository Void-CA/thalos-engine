use crate::models::scara::ScaraSpec;
use crate::prelude::*;
use thalos_math::constants::*;
use thalos_math::*;

#[test]
fn geometric_matches_numerical() {
    let q = [0.4, -0.7, 0.2, 0.5];

    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;

    let fk1 = ForwardKinematics::new(robot.clone());
    let fk2 = ForwardKinematics::new(robot);

    let geometric = GeometricJacobian::new(fk1, end_effector);
    let numerical = NumericalJacobian::new(fk2, end_effector);

    let jg = geometric.evaluate(&q);
    let jn = numerical.evaluate(&q);

    for r in 0..3 {
        for c in 0..4 {
            assert!(
                (jg.linear[(r, c)] - jn.linear[(r, c)]).abs() < 1e-5,
                "Linear mismatch at ({},{}): geometric={}, numerical={}",
                r,
                c,
                jg.linear[(r, c)],
                jn.linear[(r, c)]
            );
        }
    }
}

#[test]
fn at_zero() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[0.0, 0.0, 0.0, 0.0]);

    // Z-up: revolutos en Z → J_v = ẑ × (p - r) genera velocidad en XY
    // ee at (2, 0, 0). Joint 0 en (0,0,0): (0,0,1)×(2,0,0) = (0, 2, 0)
    // Joint 1 en (1,0,0):          (0,0,1)×(1,0,0) = (0, 1, 0)
    // Prismática 2: eje Z (0,0,1)
    // Muñeca 3: en ee → J_v = 0
    assert!(result.linear[(0, 0)].abs() < EPS, "dx/dθ1 should be 0");
    assert!(
        (result.linear[(1, 0)] - 2.0).abs() < EPS,
        "dy/dθ1 should be 2.0"
    );
    assert!(result.linear[(2, 0)].abs() < EPS, "dz/dθ1 should be 0");

    assert!(result.linear[(0, 1)].abs() < EPS, "dx/dθ2 should be 0");
    assert!(
        (result.linear[(1, 1)] - 1.0).abs() < EPS,
        "dy/dθ2 should be 1.0"
    );
    assert!(result.linear[(2, 1)].abs() < EPS, "dz/dθ2 should be 0");

    // Prismatic joint: solo afecta Z
    assert!(result.linear[(0, 2)].abs() < EPS, "dx/dd3 should be 0");
    assert!(result.linear[(1, 2)].abs() < EPS, "dy/dd3 should be 0");
    assert!(
        (result.linear[(2, 2)] - 1.0).abs() < EPS,
        "dz/dd3 should be 1.0"
    );

    // Wrist: no afecta posición (está en el ee)
    assert!(result.linear[(0, 3)].abs() < EPS, "dx/dθ4 should be 0");
    assert!(result.linear[(1, 3)].abs() < EPS, "dy/dθ4 should be 0");
    assert!(result.linear[(2, 3)].abs() < EPS, "dz/dθ4 should be 0");

    // Angular part (Z-up: revolutos en Z → ωz)
    // Revolute 0: ωz/dθ1 = 1
    assert!(
        (result.angular[(2, 0)] - 1.0).abs() < EPS,
        "ωz/dθ1 should be 1.0"
    );
    // Revolute 1: ωz/dθ2 = 1
    assert!(
        (result.angular[(2, 1)] - 1.0).abs() < EPS,
        "ωz/dθ2 should be 1.0"
    );
    // Prismatic: no angular velocity
    assert!(result.angular[(0, 2)].abs() < EPS, "ωx/dd3 should be 0");
    assert!(result.angular[(1, 2)].abs() < EPS, "ωy/dd3 should be 0");
    assert!(result.angular[(2, 2)].abs() < EPS, "ωz/dd3 should be 0");
    // Wrist: ωz/dθ4 = 1
    assert!(
        (result.angular[(2, 3)] - 1.0).abs() < EPS,
        "ωz/dθ4 should be 1.0"
    );
}

#[test]
fn at_ninety_degrees() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[PI / 2.0, 0.0, 0.0, 0.0]);

    // Z-up: θ1 = π/2, brazos en Y positivo, ee at (0, 2, 0)
    // ∂x/∂θ1 = -2, ∂x/∂θ2 = -1 (same cross product result as Y-up)
    assert!(
        (result.linear[(0, 0)] + 2.0).abs() < EPS,
        "dx/dθ1 should be -2.0"
    );
    assert!(
        (result.linear[(0, 1)] + 1.0).abs() < EPS,
        "dx/dθ2 should be -1.0"
    );
    assert!(result.linear[(1, 0)].abs() < EPS, "dy/dθ1 should be 0");
    assert!(result.linear[(1, 1)].abs() < EPS, "dy/dθ2 should be 0");
}

#[test]
fn prismatic_joint_only_affects_z() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let test_configs = [
        [0.0, 0.0, 0.0, 0.0],
        [PI / 4.0, 0.0, 0.5, 0.0],
        [PI / 2.0, PI / 4.0, -0.3, PI / 3.0],
    ];

    for q in test_configs {
        let j = jacobian.evaluate(&q);

        // Z-up: columna prismática (índice 2): solo Z
        assert!(
            j.linear[(0, 2)].abs() < 1e-6,
            "Prismatic should not affect X at q={:?}",
            q
        );
        assert!(
            j.linear[(1, 2)].abs() < 1e-6,
            "Prismatic should not affect Y at q={:?}",
            q
        );
        assert!(
            (j.linear[(2, 2)] - 1.0).abs() < 1e-4,
            "Prismatic dz/dd3 should be 1.0 at q={:?}",
            q
        );

        // Sin contribución angular
        assert!(
            j.angular[(0, 2)].abs() < 1e-6,
            "Prismatic ωx should be 0 at q={:?}",
            q
        );
        assert!(
            j.angular[(1, 2)].abs() < 1e-6,
            "Prismatic ωy should be 0 at q={:?}",
            q
        );
        assert!(
            j.angular[(2, 2)].abs() < 1e-6,
            "Prismatic ωz should be 0 at q={:?}",
            q
        );
    }
}

#[test]
fn wrist_joint_does_not_affect_position() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let test_configs = [
        [0.0, 0.0, 0.0, 0.0],
        [PI / 4.0, PI / 6.0, 0.3, PI / 2.0],
        [PI / 3.0, -PI / 4.0, -0.2, PI / 4.0],
    ];

    for q in test_configs {
        let j = jacobian.evaluate(&q);

        // La muñeca no afecta posición
        assert!(
            j.linear[(0, 3)].abs() < 1e-6,
            "Wrist should not affect X at q={:?}",
            q
        );
        assert!(
            j.linear[(1, 3)].abs() < 1e-6,
            "Wrist should not affect Y at q={:?}",
            q
        );
        assert!(
            j.linear[(2, 3)].abs() < 1e-6,
            "Wrist should not affect Z at q={:?}",
            q
        );

        // Pero sí afecta orientación (ωz = 1)
        assert!(
            (j.angular[(2, 3)] - 1.0).abs() < EPS,
            "Wrist ωz/dθ4 should be 1.0 at q={:?}",
            q
        );
    }
}

#[test]
fn angular_accumulation() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    // ωz = q_dot[0] + q_dot[1] + q_dot[3] (revolutos Z, prismático no contribuye)
    let test_configs = [
        [0.0, 0.0, 0.0, 0.0],
        [PI / 4.0, PI / 6.0, 0.3, PI / 3.0],
        [PI / 2.0, -PI / 3.0, -0.2, PI / 4.0],
    ];

    for q in test_configs {
        let j = jacobian.evaluate(&q);

        let q_dot = [0.2, 0.1, 0.05, 0.15];
        let omega_z_pred = j.angular[(2, 0)] * q_dot[0]
            + j.angular[(2, 1)] * q_dot[1]
            + j.angular[(2, 2)] * q_dot[2]
            + j.angular[(2, 3)] * q_dot[3];

        let omega_z_expected = q_dot[0] + q_dot[1] + q_dot[3];

        assert!(
            (omega_z_pred - omega_z_expected).abs() < EPS,
            "Angular mismatch at q={:?}: pred={}, expected={}",
            q,
            omega_z_pred,
            omega_z_expected
        );
    }
}

#[test]
fn linear_velocity_consistency() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot.clone());
    let jacobian = GeometricJacobian::new(fk, end_effector.clone());
    let numerical =
        NumericalJacobian::new(ForwardKinematics::new(robot.clone()), end_effector.clone());

    let test_configs = [
        [0.2, 0.3, 0.1, 0.0],
        [0.5, -0.4, 0.2, 0.3],
        [1.0, 0.8, -0.3, 0.5],
        [-0.5, 1.2, 0.4, -0.2],
        [PI / 3.0, -PI / 5.0, 0.0, PI / 4.0],
    ];

    for q in test_configs {
        let jg = jacobian.evaluate(&q);
        let jn = numerical.evaluate(&q);

        for r in 0..3 {
            for c in 0..4 {
                assert!(
                    (jg.linear[(r, c)] - jn.linear[(r, c)]).abs() < 1e-5,
                    "Linear mismatch at q={:?}, ({},{}): geo={}, num={}",
                    q,
                    r,
                    c,
                    jg.linear[(r, c)],
                    jn.linear[(r, c)]
                );
            }
        }
    }
}

#[test]
fn propagates_velocities() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot.clone());
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0, PI / 6.0, 0.3, PI / 3.0];
    let q_dot = [0.2, 0.1, 0.05, 0.15];

    let j = jacobian.evaluate(&q);

    // v = J * q_dot
    let v_x = (0..4).map(|i| j.linear[(0, i)] * q_dot[i]).sum::<f64>();
    let v_y = (0..4).map(|i| j.linear[(1, i)] * q_dot[i]).sum::<f64>();
    let v_z = (0..4).map(|i| j.linear[(2, i)] * q_dot[i]).sum::<f64>();

    let omega_z = (0..4).map(|i| j.angular[(2, i)] * q_dot[i]).sum::<f64>();

    // Finite difference FK
    let dt = 1e-5;
    let q_next = [
        q[0] + q_dot[0] * dt,
        q[1] + q_dot[1] * dt,
        q[2] + q_dot[2] * dt,
        q[3] + q_dot[3] * dt,
    ];

    let fk_solver = ForwardKinematics::new(robot);
    let current = fk_solver.evaluate(&q);
    let next = fk_solver.evaluate(&q_next);

    let p_current = current.pose(&end_effector).unwrap().transform().translation;

    let p_next = next.pose(&end_effector).unwrap().transform().translation;

    let v_actual_x = (p_next.x - p_current.x) / dt;
    let v_actual_y = (p_next.y - p_current.y) / dt;
    let v_actual_z = (p_next.z - p_current.z) / dt;

    assert!((v_x - v_actual_x).abs() < 1e-4, "X velocity mismatch");
    assert!((v_y - v_actual_y).abs() < 1e-4, "Y velocity mismatch");
    assert!((v_z - v_actual_z).abs() < 1e-4, "Z velocity mismatch");

    // ωz = q_dot[0] + q_dot[1] + q_dot[3] (prismatic no contribuye)
    let omega_expected = q_dot[0] + q_dot[1] + q_dot[3];
    assert!(
        (omega_z - omega_expected).abs() < EPS,
        "Angular velocity mismatch"
    );
}

#[test]
fn singularity_detection() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    // Singularidad en XY con brazos extendidos (Z-up: revolutos en Z → plano XY)
    let q_singular = [0.0, 0.0, 0.0, 0.0];
    let j_singular = jacobian.evaluate(&q_singular);

    // Submatriz 2×2 de las primeras 2 juntas revolutas (XY: filas 0 y 1)
    let det_singular_xy = j_singular.linear[(0, 0)] * j_singular.linear[(1, 1)]
        - j_singular.linear[(0, 1)] * j_singular.linear[(1, 0)];

    // Configuración no singular
    let q_normal = [PI / 3.0, PI / 4.0, 0.0, 0.0];
    let j_normal = jacobian.evaluate(&q_normal);
    let det_normal_xy = j_normal.linear[(0, 0)] * j_normal.linear[(1, 1)]
        - j_normal.linear[(0, 1)] * j_normal.linear[(1, 0)];

    assert!(
        det_singular_xy.abs() < det_normal_xy.abs() * 0.1,
        "XY det near singularity ({}) should be much smaller than normal ({})",
        det_singular_xy,
        det_normal_xy
    );

    // Singularidad: brazos plegados θ2 = π
    let q_folded = [0.0, PI, 0.0, 0.0];
    let j_folded = jacobian.evaluate(&q_folded);
    let det_folded_xy = j_folded.linear[(0, 0)] * j_folded.linear[(1, 1)]
        - j_folded.linear[(0, 1)] * j_folded.linear[(1, 0)];

    assert!(
        det_folded_xy.abs() < 1e-4,
        "XY det at folded config should be near zero, got {}",
        det_folded_xy
    );
}

#[test]
fn linearity() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0, PI / 6.0, 0.3, PI / 3.0];
    let j = jacobian.evaluate(&q);

    let v1 = [0.1, 0.2, 0.05, 0.15];
    let v2 = [0.05, 0.15, 0.03, 0.1];
    let a = 2.0;
    let b = 3.0;

    // J*(a*v1 + b*v2)
    let combined = {
        let v_combined = [
            a * v1[0] + b * v2[0],
            a * v1[1] + b * v2[1],
            a * v1[2] + b * v2[2],
            a * v1[3] + b * v2[3],
        ];
        let jv_x = (0..4)
            .map(|i| j.linear[(0, i)] * v_combined[i])
            .sum::<f64>();
        let jv_y = (0..4)
            .map(|i| j.linear[(1, i)] * v_combined[i])
            .sum::<f64>();
        let jv_z = (0..4)
            .map(|i| j.linear[(2, i)] * v_combined[i])
            .sum::<f64>();
        (jv_x, jv_y, jv_z)
    };

    // a*J*v1 + b*J*v2
    let jv1 = {
        let x = (0..4).map(|i| j.linear[(0, i)] * v1[i]).sum::<f64>();
        let y = (0..4).map(|i| j.linear[(1, i)] * v1[i]).sum::<f64>();
        let z = (0..4).map(|i| j.linear[(2, i)] * v1[i]).sum::<f64>();
        (x, y, z)
    };

    let jv2 = {
        let x = (0..4).map(|i| j.linear[(0, i)] * v2[i]).sum::<f64>();
        let y = (0..4).map(|i| j.linear[(1, i)] * v2[i]).sum::<f64>();
        let z = (0..4).map(|i| j.linear[(2, i)] * v2[i]).sum::<f64>();
        (x, y, z)
    };

    let linear_combined = (
        a * jv1.0 + b * jv2.0,
        a * jv1.1 + b * jv2.1,
        a * jv1.2 + b * jv2.2,
    );

    assert!(
        (combined.0 - linear_combined.0).abs() < 1e-12,
        "Linearity fails in X"
    );
    assert!(
        (combined.1 - linear_combined.1).abs() < 1e-12,
        "Linearity fails in Y"
    );
    assert!(
        (combined.2 - linear_combined.2).abs() < 1e-12,
        "Linearity fails in Z"
    );
}

#[test]
fn independent_xy_and_z_motions() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0, PI / 6.0, 0.3, PI / 3.0];
    let j = jacobian.evaluate(&q);

    // Z-up: juntas revolutas (0, 1, 3) no afectan Z
    for joint_idx in [0, 1, 3] {
        assert!(
            j.linear[(2, joint_idx)].abs() < 1e-6,
            "Revolute joint {} should not affect Z, got {}",
            joint_idx,
            j.linear[(2, joint_idx)]
        );
    }

    // Junta prismática (2) solo afecta Z (vertical)
    assert!(
        j.linear[(0, 2)].abs() < 1e-6,
        "Prismatic should not affect X"
    );
    assert!(
        j.linear[(1, 2)].abs() < 1e-6,
        "Prismatic should not affect Y"
    );
    assert!(
        (j.linear[(2, 2)] - 1.0).abs() < 1e-4,
        "Prismatic dz/dd3 should be 1.0"
    );
}
