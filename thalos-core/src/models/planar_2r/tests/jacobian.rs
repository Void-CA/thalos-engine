use crate::models::planar_2r::Planar2RSpec;
use crate::prelude::*;

// Helper para crear un robot y sus componentes
fn setup_robot() -> (
    NumericalJacobian,
    ForwardKinematics,
    crate::spatial::frame::FrameId,
) {
    let robot = Planar2RSpec::ideal().build();
    let end_effector = robot.end_effector().clone();
    let fk = ForwardKinematics::new(robot);
    let jacobian = NumericalJacobian::new(fk.clone(), end_effector.clone());
    (jacobian, fk, end_effector)
}

#[test]
fn predicts_small_motion() {
    let (jacobian, fk, end_effector) = setup_robot();

    let q = [0.0, 0.0];
    let j = jacobian.evaluate(&q);

    // Small joint perturbation
    let dq = vec![1e-4, 0.0];

    // Predicted motion
    let dx_pred = j.linear() * DynamicVector::from_vec(dq.clone());

    // Real FK motion
    let q2 = [q[0] + dq[0], q[1] + dq[1]];

    let fk1 = fk.evaluate(&q);
    let fk2 = fk.evaluate(&q2);

    let p1 = fk1.pose(&end_effector).unwrap().transform().translation;

    let p2 = fk2.pose(&end_effector).unwrap().transform().translation;

    let dx_real = vec![p2.x - p1.x, p2.y - p1.y, p2.z - p1.z];

    assert!(
        (dx_pred[0] - dx_real[0]).abs() < 1e-5,
        "X motion mismatch: predicted {}, real {}",
        dx_pred[0],
        dx_real[0]
    );

    assert!(
        (dx_pred[1] - dx_real[1]).abs() < 1e-5,
        "Y motion mismatch: predicted {}, real {}",
        dx_pred[1],
        dx_real[1]
    );

    assert!(
        (dx_pred[2] - dx_real[2]).abs() < 1e-5,
        "Z motion mismatch: predicted {}, real {}",
        dx_pred[2],
        dx_real[2]
    );
}

#[test]
fn dimensions_are_correct() {
    let (jacobian, _, _) = setup_robot();

    let q = [0.0, 0.0];
    let j = jacobian.evaluate(&q);

    assert_eq!(
        j.linear().nrows(),
        3,
        "Jacobian should have 3 rows (x, y, z)"
    );
    assert_eq!(
        j.linear().ncols(),
        2,
        "Jacobian should have 2 columns for 2 joints"
    );
}

#[test]
fn at_zero_configuration() {
    let (jacobian, _, _) = setup_robot();

    let q = [0.0, 0.0];
    let j = jacobian.evaluate(&q);

    // En configuración cero (brazos extendidos en X):
    // ∂x/∂θ1 = -L1*sin(θ1) - L2*sin(θ1+θ2) = 0
    // ∂x/∂θ2 = -L2*sin(θ1+θ2) = 0
    // ∂y/∂θ1 = L1*cos(θ1) + L2*cos(θ1+θ2) = 1 + 1 = 2
    // ∂y/∂θ2 = L2*cos(θ1+θ2) = 1
    // ∂z/∂θi = 0 para planar

    let dx_dq1 = j.linear()[(0, 0)];
    let dx_dq2 = j.linear()[(0, 1)];
    let dy_dq1 = j.linear()[(1, 0)];
    let dy_dq2 = j.linear()[(1, 1)];
    let dz_dq1 = j.linear()[(2, 0)];
    let dz_dq2 = j.linear()[(2, 1)];

    assert!(dx_dq1.abs() < 1e-6, "dx/dθ1 should be 0, got {}", dx_dq1);
    assert!(dx_dq2.abs() < 1e-6, "dx/dθ2 should be 0, got {}", dx_dq2);
    assert!(
        (dy_dq1 - 2.0).abs() < 1e-4,
        "dy/dθ1 should be 2.0, got {}",
        dy_dq1
    );
    assert!(
        (dy_dq2 - 1.0).abs() < 1e-4,
        "dy/dθ2 should be 1.0, got {}",
        dy_dq2
    );
    assert!(dz_dq1.abs() < 1e-6, "dz/dθ1 should be 0 for planar robot");
    assert!(dz_dq2.abs() < 1e-6, "dz/dθ2 should be 0 for planar robot");
}

#[test]
fn at_ninety_degrees() {
    let (jacobian, _, _) = setup_robot();

    let q = [PI / 2.0, 0.0];
    let j = jacobian.evaluate(&q);

    // Para θ1=90°, θ2=0:
    // ∂x/∂θ1 = -L1*sin(90°) - L2*sin(90°) = -1 - 1 = -2
    // ∂x/∂θ2 = -L2*sin(90°) = -1
    // ∂y/∂θ1 = L1*cos(90°) + L2*cos(90°) = 0 + 0 = 0
    // ∂y/∂θ2 = L2*cos(90°) = 0

    let dx_dq1 = j.linear()[(0, 0)];
    let dx_dq2 = j.linear()[(0, 1)];
    let dy_dq1 = j.linear()[(1, 0)];
    let dy_dq2 = j.linear()[(1, 1)];

    assert!(
        (dx_dq1 + 2.0).abs() < 1e-4,
        "dx/dθ1 should be -2.0, got {}",
        dx_dq1
    );
    assert!(
        (dx_dq2 + 1.0).abs() < 1e-4,
        "dx/dθ2 should be -1.0, got {}",
        dx_dq2
    );
    assert!(dy_dq1.abs() < 1e-6, "dy/dθ1 should be 0, got {}", dy_dq1);
    assert!(dy_dq2.abs() < 1e-6, "dy/dθ2 should be 0, got {}", dy_dq2);
}

#[test]
fn at_folded_configuration() {
    let (jacobian, _, _) = setup_robot();

    let q = [PI / 2.0, -PI / 2.0];
    let j = jacobian.evaluate(&q);

    // Configuración plegada: primer brazo arriba, segundo brazo apuntando izquierda
    // El efector final está en (1, 1)

    // Verificar que el Jacobiano tiene sentido (no singular en esta configuración)
    let det = j.linear()[(0, 0)] * j.linear()[(1, 1)] - j.linear()[(0, 1)] * j.linear()[(1, 0)];

    assert!(
        det.abs() > 0.1,
        "Jacobian determinant should not be near zero at folded config, got {}",
        det
    );
}

#[test]
fn approximates_velocity_correctly() {
    let (jacobian, fk, end_effector) = setup_robot();

    let q = [PI / 4.0, PI / 6.0];
    let q_dot = [0.2, 0.1]; // Velocidades articulares

    let j = jacobian.evaluate(&q);

    // Calcular velocidad espacial predicha: v = J * q_dot
    let v_pred_x = j.linear()[(0, 0)] * q_dot[0] + j.linear()[(0, 1)] * q_dot[1];
    let v_pred_y = j.linear()[(1, 0)] * q_dot[0] + j.linear()[(1, 1)] * q_dot[1];

    // Verificar con diferencia finita
    let dt = 1e-5;
    let q_next = [q[0] + q_dot[0] * dt, q[1] + q_dot[1] * dt];

    let p_current = fk
        .evaluate(&q)
        .pose(&end_effector)
        .unwrap()
        .transform()
        .translation;

    let p_next = fk
        .evaluate(&q_next)
        .pose(&end_effector)
        .unwrap()
        .transform()
        .translation;

    let v_actual_x = (p_next.x - p_current.x) / dt;
    let v_actual_y = (p_next.y - p_current.y) / dt;

    assert!(
        (v_pred_x - v_actual_x).abs() < 1e-4,
        "X velocity mismatch: predicted {}, actual {}",
        v_pred_x,
        v_actual_x
    );
    assert!(
        (v_pred_y - v_actual_y).abs() < 1e-4,
        "Y velocity mismatch: predicted {}, actual {}",
        v_pred_y,
        v_actual_y
    );
}

#[test]
fn determinant_indicates_singularity() {
    let (jacobian, _, _) = setup_robot();

    // Configuración singular: brazos completamente extendidos (θ2 = 0)
    let q_singular = [0.0, 0.0];
    let j_singular = jacobian.evaluate(&q_singular);
    let det_singular = j_singular.linear()[(0, 0)] * j_singular.linear()[(1, 1)]
        - j_singular.linear()[(0, 1)] * j_singular.linear()[(1, 0)];

    // Configuración no singular
    let q_normal = [PI / 3.0, PI / 4.0];
    let j_normal = jacobian.evaluate(&q_normal);
    let det_normal = j_normal.linear()[(0, 0)] * j_normal.linear()[(1, 1)]
        - j_normal.linear()[(0, 1)] * j_normal.linear()[(1, 0)];

    // El determinante debería ser significativamente menor en singularidad
    assert!(
        det_singular.abs() < det_normal.abs() * 0.1,
        "Determinant near singularity ({}) should be much smaller than normal ({})",
        det_singular,
        det_normal
    );

    // Otra singularidad: brazos plegados (θ2 = π)
    let q_folded = [0.0, PI];
    let j_folded = jacobian.evaluate(&q_folded);
    let det_folded = j_folded.linear()[(0, 0)] * j_folded.linear()[(1, 1)]
        - j_folded.linear()[(0, 1)] * j_folded.linear()[(1, 0)];

    assert!(
        det_folded.abs() < 1e-4,
        "Determinant at folded config should be near zero, got {}",
        det_folded
    );
}

#[test]
fn reconstruction_from_motion() {
    let (jacobian, fk, end_effector) = setup_robot();

    // Para varias configuraciones, verificar que J * q_dot ≈ Δp/Δt
    let test_configs = [
        ([0.0, 0.0], [0.1, 0.05]),
        ([PI / 4.0, 0.0], [0.2, 0.1]),
        ([PI / 3.0, PI / 6.0], [0.15, 0.2]),
        ([PI / 2.0, -PI / 4.0], [0.1, 0.15]),
    ];

    let dt = 1e-5;

    for (q, q_dot) in test_configs {
        let j = jacobian.evaluate(&q);

        // Velocidad predicha
        let v_pred_x = j.linear()[(0, 0)] * q_dot[0] + j.linear()[(0, 1)] * q_dot[1];
        let v_pred_y = j.linear()[(1, 0)] * q_dot[0] + j.linear()[(1, 1)] * q_dot[1];

        // Velocidad real
        let q_next = [q[0] + q_dot[0] * dt, q[1] + q_dot[1] * dt];

        let p_curr = fk
            .evaluate(&q)
            .pose(&end_effector)
            .unwrap()
            .transform()
            .translation;

        let p_next = fk
            .evaluate(&q_next)
            .pose(&end_effector)
            .unwrap()
            .transform()
            .translation;

        let v_actual_x = (p_next.x - p_curr.x) / dt;
        let v_actual_y = (p_next.y - p_curr.y) / dt;

        let error_x = (v_pred_x - v_actual_x).abs();
        let error_y = (v_pred_y - v_actual_y).abs();

        assert!(
            error_x < 1e-4,
            "X velocity error too large at q={:?}: predicted {}, actual {}",
            q,
            v_pred_x,
            v_actual_x
        );
        assert!(
            error_y < 1e-4,
            "Y velocity error too large at q={:?}: predicted {}, actual {}",
            q,
            v_pred_y,
            v_actual_y
        );
    }
}

#[test]
fn maps_velocities_linearly() {
    let (jacobian, _, _) = setup_robot();

    let q = [PI / 4.0, PI / 6.0];
    let j = jacobian.evaluate(&q);

    // Probar linealidad: J*(a*v1 + b*v2) = a*J*v1 + b*J*v2
    let v1 = [0.1, 0.2];
    let v2 = [0.05, 0.15];
    let a = 2.0;
    let b = 3.0;

    let jv_combined = {
        let v_combined = [a * v1[0] + b * v2[0], a * v1[1] + b * v2[1]];
        let jv_x = j.linear()[(0, 0)] * v_combined[0] + j.linear()[(0, 1)] * v_combined[1];
        let jv_y = j.linear()[(1, 0)] * v_combined[0] + j.linear()[(1, 1)] * v_combined[1];
        (jv_x, jv_y)
    };

    let jv1 = {
        let jv_x = j.linear()[(0, 0)] * v1[0] + j.linear()[(0, 1)] * v1[1];
        let jv_y = j.linear()[(1, 0)] * v1[0] + j.linear()[(1, 1)] * v1[1];
        (jv_x, jv_y)
    };

    let jv2 = {
        let jv_x = j.linear()[(0, 0)] * v2[0] + j.linear()[(0, 1)] * v2[1];
        let jv_y = j.linear()[(1, 0)] * v2[0] + j.linear()[(1, 1)] * v2[1];
        (jv_x, jv_y)
    };

    let jv_linear = (a * jv1.0 + b * jv2.0, a * jv1.1 + b * jv2.1);

    assert!(
        (jv_combined.0 - jv_linear.0).abs() < 1e-10,
        "Linearity fails in X: combined {}, linear {}",
        jv_combined.0,
        jv_linear.0
    );
    assert!(
        (jv_combined.1 - jv_linear.1).abs() < 1e-10,
        "Linearity fails in Y: combined {}, linear {}",
        jv_combined.1,
        jv_linear.1
    );
}
