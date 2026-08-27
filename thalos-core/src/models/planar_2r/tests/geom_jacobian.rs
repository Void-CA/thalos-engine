use crate::models::planar_2r::Planar2RSpec;
use crate::prelude::*;

#[test]
fn geometric_matches_numerical() {
    let q = [0.4, -0.7];

    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;

    let fk1 = ForwardKinematics::new(robot.clone());
    let fk2 = ForwardKinematics::new(robot);

    let geometric = GeometricJacobian::new(fk1, end_effector);
    let numerical = NumericalJacobian::new(fk2, end_effector);

    let jg = geometric.evaluate(&q);
    let jn = numerical.evaluate(&q);

    for r in 0..3 {
        for c in 0..2 {
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
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[0.0, 0.0]);

    // Linear part
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

    // Angular part (for planar robot, angular velocity is around Z axis)
    assert!(
        (result.angular[(2, 0)] - 1.0).abs() < EPS,
        "ωz/dθ1 should be 1.0"
    );
    assert!(
        (result.angular[(2, 1)] - 1.0).abs() < EPS,
        "ωz/dθ2 should be 1.0"
    );
    assert!(result.angular[(0, 0)].abs() < EPS, "ωx/dθ1 should be 0");
    assert!(result.angular[(1, 0)].abs() < EPS, "ωy/dθ1 should be 0");
}

#[test]
fn at_ninety_degrees() {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[PI / 2.0, 0.0]);

    // Linear part: brazos verticales
    // x = 0, y = 2
    // dx/dθ1 = -2, dx/dθ2 = -1
    // dy/dθ1 = 0, dy/dθ2 = 0
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

    // Angular part
    assert!(
        (result.angular[(2, 0)] - 1.0).abs() < EPS,
        "ωz/dθ1 should be 1.0"
    );
    assert!(
        (result.angular[(2, 1)] - 1.0).abs() < EPS,
        "ωz/dθ2 should be 1.0"
    );
}

#[test]
fn at_folded_configuration() {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[PI / 2.0, -PI / 2.0]);

    // Configuración plegada: brazo1 arriba, brazo2 izquierda
    // Posición efector: (1, 1)

    // Verificar que el Jacobiano tiene sentido físico
    let linear_vel_x = result.linear[(0, 0)];
    let linear_vel_y = result.linear[(1, 0)];

    // En esta configuración, ambos brazos contribuyen al movimiento
    assert!(linear_vel_x.abs() > 0.1, "dx/dθ1 should be significant");
    assert!(linear_vel_y.abs() > 0.1, "dy/dθ1 should be significant");

    // La velocidad angular debe ser la suma
    assert!(
        (result.angular[(2, 0)] - 1.0).abs() < EPS,
        "ωz/dθ1 should be 1.0"
    );
    assert!(
        (result.angular[(2, 1)] - 1.0).abs() < EPS,
        "ωz/dθ2 should be 1.0"
    );
}

#[test]
fn angular_velocity_accumulation() {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    // Para un robot planar, la velocidad angular del efector final
    // es la suma de todas las velocidades de las juntas
    let test_configs = [
        [0.0, 0.0],
        [PI / 4.0, PI / 4.0],
        [PI / 2.0, -PI / 3.0],
        [PI / 3.0, PI / 6.0],
    ];

    for q in test_configs {
        let result = jacobian.evaluate(&q);

        // Cada junta contribuye con 1 rad/s a la velocidad angular Z
        assert!(
            (result.angular[(2, 0)] - 1.0).abs() < EPS,
            "ωz/dθ1 should be 1.0 at q={:?}, got {}",
            q,
            result.angular[(2, 0)]
        );
        assert!(
            (result.angular[(2, 1)] - 1.0).abs() < EPS,
            "ωz/dθ2 should be 1.0 at q={:?}, got {}",
            q,
            result.angular[(2, 1)]
        );
    }
}

#[test]
fn linear_velocity_consistency() {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot.clone());
    let jacobian = GeometricJacobian::new(fk, end_effector.clone());
    let numerical =
        NumericalJacobian::new(ForwardKinematics::new(robot.clone()), end_effector.clone());

    // Probar múltiples configuraciones aleatorias
    let test_configs = [
        [0.2, 0.3],
        [0.5, -0.4],
        [1.0, 0.8],
        [-0.5, 1.2],
        [PI / 3.0, -PI / 5.0],
    ];

    for q in test_configs {
        let jg = jacobian.evaluate(&q);
        let jn = numerical.evaluate(&q);

        for r in 0..3 {
            for c in 0..2 {
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
    let robot = Planar2RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child;

    let fk = ForwardKinematics::new(robot.clone());

    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0, PI / 6.0];

    let q_dot = [0.5, 0.3];

    let j = jacobian.evaluate(&q);

    // v = J qdot

    let v_x = j.linear[(0, 0)] * q_dot[0] + j.linear[(0, 1)] * q_dot[1];

    let v_y = j.linear[(1, 0)] * q_dot[0] + j.linear[(1, 1)] * q_dot[1];

    let omega_z = j.angular[(2, 0)] * q_dot[0] + j.angular[(2, 1)] * q_dot[1];

    // finite difference FK

    let dt = 1e-5;

    let q_next = [q[0] + q_dot[0] * dt, q[1] + q_dot[1] * dt];

    let fk_solver = ForwardKinematics::new(robot);

    let current = fk_solver.evaluate(&q);

    let next = fk_solver.evaluate(&q_next);

    let p_current = current.pose(&end_effector).unwrap().transform().translation;

    let p_next = next.pose(&end_effector).unwrap().transform().translation;

    let v_actual_x = (p_next.x - p_current.x) / dt;

    let v_actual_y = (p_next.y - p_current.y) / dt;

    assert!((v_x - v_actual_x).abs() < 1e-4);

    assert!((v_y - v_actual_y).abs() < 1e-4);

    // angular velocity

    let omega_expected = q_dot[0] + q_dot[1];

    assert!((omega_z - omega_expected).abs() < EPS);
}

#[test]
fn singularity_detection() {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    // Configuración singular: brazos completamente extendidos
    let q_singular = [0.0, 0.0];
    let J_singular = jacobian.evaluate(&q_singular);

    // Calcular determinante de la submatriz lineal 2x2
    let det_singular = J_singular.linear[(0, 0)] * J_singular.linear[(1, 1)]
        - J_singular.linear[(0, 1)] * J_singular.linear[(1, 0)];

    // Configuración no singular
    let q_normal = [PI / 3.0, PI / 4.0];
    let J_normal = jacobian.evaluate(&q_normal);
    let det_normal = J_normal.linear[(0, 0)] * J_normal.linear[(1, 1)]
        - J_normal.linear[(0, 1)] * J_normal.linear[(1, 0)];

    assert!(
        det_singular.abs() < det_normal.abs() * 0.1,
        "Determinant near singularity ({}) should be much smaller than normal ({})",
        det_singular,
        det_normal
    );

    // Otra singularidad: brazos plegados
    let q_folded = [0.0, PI];
    let J_folded = jacobian.evaluate(&q_folded);
    let det_folded = J_folded.linear[(0, 0)] * J_folded.linear[(1, 1)]
        - J_folded.linear[(0, 1)] * J_folded.linear[(1, 0)];

    assert!(
        det_folded.abs() < 1e-4,
        "Determinant at folded config should be near zero, got {}",
        det_folded
    );
}

#[test]
fn linearity() {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0, PI / 6.0];
    let J = jacobian.evaluate(&q);

    // Probar que J(q) es lineal en q_dot
    let q_dot1 = [0.2, 0.1];
    let q_dot2 = [0.05, 0.15];
    let a = 2.0;
    let b = 3.0;

    // Calcular J*(a*v1 + b*v2)
    let combined_linear = {
        let v_combined = [a * q_dot1[0] + b * q_dot2[0], a * q_dot1[1] + b * q_dot2[1]];
        let vx = J.linear[(0, 0)] * v_combined[0] + J.linear[(0, 1)] * v_combined[1];
        let vy = J.linear[(1, 0)] * v_combined[0] + J.linear[(1, 1)] * v_combined[1];
        (vx, vy)
    };

    // Calcular a*J*v1 + b*J*v2
    let linear_v1 = {
        let vx = J.linear[(0, 0)] * q_dot1[0] + J.linear[(0, 1)] * q_dot1[1];
        let vy = J.linear[(1, 0)] * q_dot1[0] + J.linear[(1, 1)] * q_dot1[1];
        (vx, vy)
    };

    let linear_v2 = {
        let vx = J.linear[(0, 0)] * q_dot2[0] + J.linear[(0, 1)] * q_dot2[1];
        let vy = J.linear[(1, 0)] * q_dot2[0] + J.linear[(1, 1)] * q_dot2[1];
        (vx, vy)
    };

    let linear_combined = (
        a * linear_v1.0 + b * linear_v2.0,
        a * linear_v1.1 + b * linear_v2.1,
    );

    assert!(
        (combined_linear.0 - linear_combined.0).abs() < 1e-12,
        "Linearity fails in X: combined {}, linear {}",
        combined_linear.0,
        linear_combined.0
    );
    assert!(
        (combined_linear.1 - linear_combined.1).abs() < 1e-12,
        "Linearity fails in Y: combined {}, linear {}",
        combined_linear.1,
        linear_combined.1
    );
}

#[test]
fn angular_consistency() {
    let robot = Planar2RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child;

    let fk = ForwardKinematics::new(robot);

    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0, PI / 6.0];

    let j = jacobian.evaluate(&q);

    let test_velocities = [[0.1, 0.0], [0.0, 0.1], [0.2, 0.3], [-0.5, 0.2]];

    for q_dot in test_velocities {
        let omega_z_pred = j.angular[(2, 0)] * q_dot[0] + j.angular[(2, 1)] * q_dot[1];

        let omega_z_expected = q_dot[0] + q_dot[1];

        assert!(
            (omega_z_pred - omega_z_expected).abs() < EPS,
            "Angular velocity mismatch: \
             predicted {}, expected {}",
            omega_z_pred,
            omega_z_expected
        );
    }
}

#[test]
fn joint_contributions() {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let test_configs = [
        ([0.0, 0.0], "extended"),
        ([PI / 2.0, 0.0], "first vertical"),
        ([PI / 3.0, PI / 3.0], "both angled"),
        ([PI / 4.0, -PI / 2.0], "folded"),
    ];

    for (q, name) in test_configs {
        let J = jacobian.evaluate(&q);

        // Verificar que la columna de cada junta tiene sentido físico
        for joint_idx in 0..2 {
            let vx = J.linear[(0, joint_idx)];
            let vy = J.linear[(1, joint_idx)];
            let magnitude = (vx * vx + vy * vy).sqrt();

            // La magnitud no debería exceder la longitud total de los brazos
            assert!(
                magnitude <= 2.1, // L1 + L2 + pequeño margen
                "Joint {} contribution magnitude {} exceeds maximum at config {}",
                joint_idx,
                magnitude,
                name
            );
        }

        // Verificar que la velocidad angular es 1 para cada junta
        assert!(
            (J.angular[(2, 0)] - 1.0).abs() < 1e-10,
            "Angular contribution of joint 1 should be 1.0 at config {}",
            name
        );
        assert!(
            (J.angular[(2, 1)] - 1.0).abs() < 1e-10,
            "Angular contribution of joint 2 should be 1.0 at config {}",
            name
        );
    }
}
