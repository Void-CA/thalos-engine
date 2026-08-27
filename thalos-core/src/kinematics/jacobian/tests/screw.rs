use crate::kinematics::jacobian::screw::ScrewJacobian;
use crate::models::planar_3r::Planar3RSpec;
use crate::models::scara::ScaraSpec;
use crate::prelude::*;

#[test]
fn scara_dimensions() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let sj = ScrewJacobian::new(fk, end_effector);

    let q = [0.0, 0.0, 0.0, 0.0];
    let j = sj.evaluate(&q);

    assert_eq!(j.linear.nrows(), 3);
    assert_eq!(j.linear.ncols(), 4);
    assert_eq!(j.angular.nrows(), 3);
    assert_eq!(j.angular.ncols(), 4);
}

#[test]
fn planar3r_dimensions() {
    let robot = Planar3RSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let sj = ScrewJacobian::new(fk, end_effector);

    let q = [0.0, 0.0, 0.0];
    let j = sj.evaluate(&q);

    assert_eq!(j.linear.nrows(), 3);
    assert_eq!(j.linear.ncols(), 3);
    assert_eq!(j.angular.nrows(), 3);
    assert_eq!(j.angular.ncols(), 3);
}

#[test]
fn scara_twist_column_structure() {
    let robot = ScaraSpec::ideal().build();
    let end_effector = robot.segments.last().unwrap().child;
    let fk = ForwardKinematics::new(robot);
    let sj = ScrewJacobian::new(fk, end_effector);

    let q = [0.0, 0.0, 0.0, 0.0];
    let j = sj.evaluate(&q);

    for col in [0, 1, 3] {
        assert!(
            (j.angular[(2, col)] - 1.0).abs() < 1e-6,
            "Revolute joint {} should have ωz=1, got {}",
            col,
            j.angular[(2, col)]
        );
        assert!(
            j.angular[(0, col)].abs() < 1e-6,
            "Revolute joint {} should have ωx=0",
            col
        );
        assert!(
            j.angular[(1, col)].abs() < 1e-6,
            "Revolute joint {} should have ωy=0",
            col
        );
    }

    for row in 0..3 {
        assert!(
            j.angular[(row, 2)].abs() < 1e-6,
            "Prismatic joint angular row {} should be 0",
            row
        );
    }
}

// ═══════════════════════════════════════════════════════════════════
// T14: Cross-validation
// ═══════════════════════════════════════════════════════════════════

use rand::Rng;

#[test]
fn scara_screw_vs_numerical() {
    let mut rng = rand::thread_rng();
    let configs: Vec<[f64; 4]> = (0..5)
        .map(|_| {
            [
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
                rng.gen_range(-0.5..0.5),
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
            ]
        })
        .collect();

    for q in configs {
        let robot = ScaraSpec::ideal().build();
        let ee = robot.segments.last().unwrap().child.clone();
        let fk_num = ForwardKinematics::new(robot.clone());
        let fk_screw = ForwardKinematics::new(robot);

        let num = NumericalJacobian::new(fk_num, ee.clone());
        let screw = ScrewJacobian::new(fk_screw, ee);

        let jn = num.evaluate(&q);
        let js = screw.evaluate(&q);

        for r in 0..3 {
            for c in 0..4 {
                assert!(
                    (js.linear[(r, c)] - jn.linear[(r, c)]).abs() < 1e-4,
                    "SCARA screw vs numerical mismatch at q={:?}, ({},{}): screw={}, num={}",
                    q,
                    r,
                    c,
                    js.linear[(r, c)],
                    jn.linear[(r, c)]
                );
            }
        }
    }
}

#[test]
fn planar3r_screw_vs_numerical() {
    let mut rng = rand::thread_rng();
    let configs: Vec<[f64; 3]> = (0..5)
        .map(|_| {
            [
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
            ]
        })
        .collect();

    for q in configs {
        let robot = Planar3RSpec::ideal().build();
        let ee = robot.segments.last().unwrap().child.clone();
        let fk_num = ForwardKinematics::new(robot.clone());
        let fk_screw = ForwardKinematics::new(robot);

        let num = NumericalJacobian::new(fk_num, ee.clone());
        let screw = ScrewJacobian::new(fk_screw, ee);

        let jn = num.evaluate(&q);
        let js = screw.evaluate(&q);

        for r in 0..3 {
            for c in 0..3 {
                assert!(
                    (js.linear[(r, c)] - jn.linear[(r, c)]).abs() < 1e-4,
                    "Planar3R screw vs numerical mismatch at q={:?}, ({},{}): screw={}, num={}",
                    q,
                    r,
                    c,
                    js.linear[(r, c)],
                    jn.linear[(r, c)]
                );
            }
        }
    }
}

#[test]
fn scara_screw_vs_geometric() {
    let mut rng = rand::thread_rng();
    let configs: Vec<[f64; 4]> = (0..5)
        .map(|_| {
            [
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
                rng.gen_range(-0.5..0.5),
                rng.gen_range(-std::f64::consts::FRAC_PI_2..std::f64::consts::FRAC_PI_2),
            ]
        })
        .collect();

    for q in configs {
        let robot = ScaraSpec::ideal().build();
        let ee = robot.segments.last().unwrap().child.clone();
        let fk_geom = ForwardKinematics::new(robot.clone());
        let fk_screw = ForwardKinematics::new(robot);

        let geom = GeometricJacobian::new(fk_geom, ee.clone());
        let screw = ScrewJacobian::new(fk_screw, ee);

        let jg = geom.evaluate(&q);
        let js = screw.evaluate(&q);

        for r in 0..6 {
            for c in 0..4 {
                let val_g = if r < 3 {
                    jg.linear[(r, c)]
                } else {
                    jg.angular[(r - 3, c)]
                };
                let val_s = if r < 3 {
                    js.linear[(r, c)]
                } else {
                    js.angular[(r - 3, c)]
                };
                assert!(
                    (val_g - val_s).abs() < 1e-8,
                    "SCARA screw vs geometric mismatch at q={:?}, ({},{}): geom={}, screw={}",
                    q,
                    r,
                    c,
                    val_g,
                    val_s
                );
            }
        }
    }
}
