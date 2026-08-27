use crate::models::single_revolute::SingleRevoluteSpec;
use crate::prelude::*;
use thalos_math::constants::*;
use thalos_math::*;

#[test]
fn geometric_matches_numerical() {
    let q = [0.4];

    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;

    let fk1 = ForwardKinematics::new(robot.clone());
    let fk2 = ForwardKinematics::new(robot);

    let geometric = GeometricJacobian::new(fk1, end_effector);
    let numerical = NumericalJacobian::new(fk2, end_effector);

    let jg = geometric.evaluate(&q);
    let jn = numerical.evaluate(&q);

    for r in 0..3 {
        assert!(
            (jg.linear[(r, 0)] - jn.linear[(r, 0)]).abs() < 1e-5,
            "Linear mismatch at row {}: geometric={}, numerical={}",
            r,
            jg.linear[(r, 0)],
            jn.linear[(r, 0)]
        );
    }
}

#[test]
fn at_zero() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[0.0]);

    // q=0: ee at (1, 0, 0)
    // z × (p_e - p_0) = (0,0,1) × (1,0,0) = (0, 1, 0)
    assert!(result.linear[(0, 0)].abs() < EPS, "dx/dθ should be 0");
    assert!(
        (result.linear[(1, 0)] - 1.0).abs() < EPS,
        "dy/dθ should be 1.0"
    );
    assert!(result.linear[(2, 0)].abs() < EPS, "dz/dθ should be 0");

    // Angular: rotation about Z
    assert!(result.angular[(0, 0)].abs() < EPS, "ωx/dθ should be 0");
    assert!(result.angular[(1, 0)].abs() < EPS, "ωy/dθ should be 0");
    assert!(
        (result.angular[(2, 0)] - 1.0).abs() < EPS,
        "ωz/dθ should be 1.0"
    );
}

#[test]
fn at_ninety_degrees() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[PI / 2.0]);

    // q=π/2: ee at (0, 1, 0)
    // z × (p_e - p_0) = (0,0,1) × (0,1,0) = (-1, 0, 0)
    assert!(
        (result.linear[(0, 0)] + 1.0).abs() < EPS,
        "dx/dθ should be -1.0"
    );
    assert!(result.linear[(1, 0)].abs() < EPS, "dy/dθ should be 0");
    assert!(result.linear[(2, 0)].abs() < EPS, "dz/dθ should be 0");
}

#[test]
fn at_pi() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let result = jacobian.evaluate(&[PI]);

    // q=π: ee at (-1, 0, 0)
    // z × (p_e - p_0) = (0,0,1) × (-1,0,0) = (0, -1, 0)
    assert!(result.linear[(0, 0)].abs() < EPS, "dx/dθ should be 0 at π");
    assert!(
        (result.linear[(1, 0)] + 1.0).abs() < EPS,
        "dy/dθ should be -1.0 at π"
    );
    assert!(result.linear[(2, 0)].abs() < EPS, "dz/dθ should be 0");
}

#[test]
fn angular_consistency() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    // ωz = q_dot siempre (única junta)
    let test_configs = [0.0, PI / 4.0, PI / 2.0, -PI / 3.0];

    for q in test_configs {
        let result = jacobian.evaluate(&[q]);

        assert!(
            (result.angular[(2, 0)] - 1.0).abs() < EPS,
            "ωz/dθ should be 1.0 at q={}, got {}",
            q,
            result.angular[(2, 0)]
        );
    }
}

#[test]
fn propagates_velocity() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot.clone());
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0];
    let q_dot = [0.5];

    let j = jacobian.evaluate(&q);

    // v = J * q_dot
    let v_x = j.linear[(0, 0)] * q_dot[0];
    let v_y = j.linear[(1, 0)] * q_dot[0];
    let omega_z = j.angular[(2, 0)] * q_dot[0];

    // Finite difference FK
    let dt = 1e-5;
    let q_next = [q[0] + q_dot[0] * dt];

    let fk_solver = ForwardKinematics::new(robot);
    let current = fk_solver.evaluate(&q);
    let next = fk_solver.evaluate(&q_next);

    let p_current = current.pose(&end_effector).unwrap().transform().translation;

    let p_next = next.pose(&end_effector).unwrap().transform().translation;

    let v_actual_x = (p_next.x - p_current.x) / dt;
    let v_actual_y = (p_next.y - p_current.y) / dt;

    assert!((v_x - v_actual_x).abs() < 1e-4, "X velocity mismatch");
    assert!((v_y - v_actual_y).abs() < 1e-4, "Y velocity mismatch");

    // ωz = q_dot (única junta)
    assert!(
        (omega_z - q_dot[0]).abs() < EPS,
        "Angular velocity mismatch"
    );
}

#[test]
fn linearity() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    let q = [PI / 4.0];
    let j = jacobian.evaluate(&q);

    let q_dot1 = 0.2;
    let q_dot2 = 0.15;
    let a = 2.0;
    let b = 3.0;

    // J*(a*v1 + b*v2) = a*J*v1 + b*J*v2
    let combined = {
        let v = a * q_dot1 + b * q_dot2;
        let vx = j.linear[(0, 0)] * v;
        let vy = j.linear[(1, 0)] * v;
        (vx, vy)
    };

    let linear = {
        let vx = a * (j.linear[(0, 0)] * q_dot1) + b * (j.linear[(0, 0)] * q_dot2);
        let vy = a * (j.linear[(1, 0)] * q_dot1) + b * (j.linear[(1, 0)] * q_dot2);
        (vx, vy)
    };

    assert!(
        (combined.0 - linear.0).abs() < 1e-12,
        "Linearity fails in X: combined {}, linear {}",
        combined.0,
        linear.0
    );
    assert!(
        (combined.1 - linear.1).abs() < 1e-12,
        "Linearity fails in Y: combined {}, linear {}",
        combined.1,
        linear.1
    );
}

#[test]
fn velocity_consistency() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot.clone());
    let jacobian = GeometricJacobian::new(fk, end_effector.clone());
    let numerical =
        NumericalJacobian::new(ForwardKinematics::new(robot.clone()), end_effector.clone());

    let test_configs = [[0.0], [0.4], [PI / 2.0], [PI], [-0.5], [PI / 3.0]];

    for q in test_configs {
        let jg = jacobian.evaluate(&q);
        let jn = numerical.evaluate(&q);

        for r in 0..3 {
            assert!(
                (jg.linear[(r, 0)] - jn.linear[(r, 0)]).abs() < 1e-5,
                "Mismatch at q={:?}, row {}: geo={}, num={}",
                q,
                r,
                jg.linear[(r, 0)],
                jn.linear[(r, 0)]
            );
        }
    }
}

#[test]
fn linear_magnitude() {
    let robot = SingleRevoluteSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let jacobian = GeometricJacobian::new(fk, end_effector);

    // La magnitud de la velocidad lineal debería ser igual al radio (distancia al origen)
    // ||v|| = ||ω|| × r = 1.0 × 1.0 = 1.0
    let result = jacobian.evaluate(&[0.0]);
    let magnitude = (result.linear[(0, 0)].powi(2) + result.linear[(1, 0)].powi(2)).sqrt();
    assert!(
        (magnitude - 1.0).abs() < EPS,
        "Linear velocity magnitude should be 1.0 (radius), got {}",
        magnitude
    );

    // En cualquier configuración, la magnitud debe ser 1.0 (el radio del círculo)
    let test_qs = [PI / 4.0, PI / 2.0, 3.0 * PI / 4.0, PI, -PI / 3.0];
    for q in test_qs {
        let result = jacobian.evaluate(&[q]);
        let magnitude = (result.linear[(0, 0)].powi(2) + result.linear[(1, 0)].powi(2)).sqrt();
        assert!(
            (magnitude - 1.0).abs() < 1e-10,
            "Linear magnitude should be 1.0 at q={}, got {}",
            q,
            magnitude
        );
    }
}
