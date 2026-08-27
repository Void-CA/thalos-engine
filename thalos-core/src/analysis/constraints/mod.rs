//! Restricciones simbólicas para configuraciones robóticas.
//!
//! Define tipos de restricciones que pueden evaluarse contra una
//! configuración articular (RobotState + chain) para validar si se
//! cumplen o no, y con qué penalización.
//!
//! # Ejemplo
//!
//! ```ignore
//! use thalos_core::analysis::constraints::{
//!     Constraint, ConstraintEvaluator, DefaultConstraintEvaluator, ConstraintViolation,
//! };
//!
//! let constraints = vec![
//!     Constraint::OrientationCone {
//!         frame: end_effector.clone(),
//!         axis: Vector3::new(0.0, 0.0, 1.0),
//!         half_angle: 30.0_f64.to_radians(),
//!     },
//! ];
//!
//! let evaluator = DefaultConstraintEvaluator;
//! let violations = evaluator.evaluate_trajectory(
//!     &constraints, &trajectory, &chain, &fk, Some(&tcp),
//! );
//! ```

use std::fmt;

use crate::kinematics::forward::ForwardKinematics;
use crate::robot::serial_chain::SerialChain;
use crate::robot::tool_frame::ToolFrame;
use crate::spatial::frame::FrameId;
use crate::trajectory::Trajectory;
use thalos_math::Dot;
use thalos_math::Vector3;

/// Una restricción simbólica sobre una configuración robótica.
#[derive(Debug, Clone, PartialEq)]
pub enum Constraint {
    /// Límite articular: un joint debe estar dentro de [min, max].
    JointLimit { joint: usize, min: f64, max: f64 },

    /// Cono de orientación: un frame debe mantener su orientación dentro
    /// de un cono alrededor de un eje dado (en grados para la interfaz,
    /// en radianes internamente).
    OrientationCone {
        frame: FrameId,
        axis: Vector3,
        half_angle: f64,
    },

    /// Caja cartesiana: un frame debe permanecer dentro de una caja
    /// alineada a los ejes del mundo.
    CartesianBox {
        frame: FrameId,
        min: Vector3,
        max: Vector3,
    },

    /// Composición AND: todas las sub-restricciones deben cumplirse.
    Composite(Vec<Constraint>),
}

impl fmt::Display for Constraint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Constraint::JointLimit { joint, min, max } => {
                write!(f, "JointLimit(j{}=[{:.3}, {:.3}])", joint, min, max)
            }
            Constraint::OrientationCone {
                frame, half_angle, ..
            } => {
                write!(
                    f,
                    "OrientationCone(frame={}, half={:.1}°)",
                    frame,
                    half_angle.to_degrees()
                )
            }
            Constraint::CartesianBox { frame, .. } => {
                write!(f, "CartesianBox(frame={})", frame)
            }
            Constraint::Composite(n) => {
                write!(f, "Composite({} constraints)", n.len())
            }
        }
    }
}

/// Resultado de evaluar una restricción contra un waypoint.
#[derive(Debug, Clone)]
pub struct ConstraintViolation {
    /// La restricción que se violó.
    pub constraint: Constraint,
    /// Índice del waypoint donde ocurrió la violación.
    pub waypoint: usize,
    /// Magnitud de la violación (0 = justo en el límite, > 0 = que excede).
    pub magnitude: f64,
    /// Mensaje legible para el usuario.
    pub message: String,
}

/// Evaluador de restricciones sobre trayectorias.
///
/// Implementa el chequeo de todas las variantes de [`Constraint`]
/// contra una trayectoria completa.
pub trait ConstraintEvaluator {
    /// Evalúa todas las restricciones contra todos los waypoints.
    fn evaluate_trajectory(
        &self,
        constraints: &[Constraint],
        trajectory: &Trajectory,
        chain: &SerialChain,
        fk: &ForwardKinematics,
        tcp: Option<&ToolFrame>,
    ) -> Vec<ConstraintViolation>;
}

/// Evaluador por defecto que implementa todos los tipos de constraint.
pub struct DefaultConstraintEvaluator;

impl ConstraintEvaluator for DefaultConstraintEvaluator {
    fn evaluate_trajectory(
        &self,
        constraints: &[Constraint],
        trajectory: &Trajectory,
        chain: &SerialChain,
        fk: &ForwardKinematics,
        tcp: Option<&ToolFrame>,
    ) -> Vec<ConstraintViolation> {
        let mut violations = Vec::new();

        for (wp_idx, waypoint) in trajectory.waypoints().iter().enumerate() {
            let q = waypoint.joints();
            let fk_result = fk.evaluate(q);

            for constraint in constraints {
                match constraint {
                    Constraint::JointLimit { joint, min, max } => {
                        if *joint < q.len() {
                            let val = q[*joint];
                            if val < *min {
                                violations.push(ConstraintViolation {
                                    constraint: constraint.clone(),
                                    waypoint: wp_idx,
                                    magnitude: *min - val,
                                    message: format!(
                                        "Joint {} = {:.3} below limit {:.3} at waypoint {}",
                                        joint, val, min, wp_idx
                                    ),
                                });
                            } else if val > *max {
                                violations.push(ConstraintViolation {
                                    constraint: constraint.clone(),
                                    waypoint: wp_idx,
                                    magnitude: val - *max,
                                    message: format!(
                                        "Joint {} = {:.3} above limit {:.3} at waypoint {}",
                                        joint, val, max, wp_idx
                                    ),
                                });
                            }
                        }
                    }
                    Constraint::OrientationCone {
                        frame,
                        axis,
                        half_angle,
                    } => {
                        let pose = if let Some(tcp) = tcp {
                            fk_result.tcp_pose(tcp)
                        } else {
                            fk_result.pose(frame).cloned()
                        };

                        if let Some(pose) = pose {
                            let current_axis = pose.transform().rotation.rotate_vector(*axis);
                            let cos_angle = (*axis).dot(current_axis)
                                / (axis.magnitude() * current_axis.magnitude());
                            let angle = cos_angle.clamp(-1.0, 1.0).acos();
                            if angle > *half_angle {
                                violations.push(ConstraintViolation {
                                    constraint: constraint.clone(),
                                    waypoint: wp_idx,
                                    magnitude: angle - half_angle,
                                    message: format!(
                                        "Orientation deviates {:.1}° (limit {:.1}°) at waypoint {}",
                                        angle.to_degrees(),
                                        half_angle.to_degrees(),
                                        wp_idx,
                                    ),
                                });
                            }
                        }
                    }
                    Constraint::CartesianBox { frame, min, max } => {
                        let pose = if let Some(tcp) = tcp {
                            fk_result.tcp_pose(tcp)
                        } else {
                            fk_result.pose(frame).cloned()
                        };

                        if let Some(pose) = pose {
                            let pos = pose.translation();
                            let mut mag = 0.0;
                            if pos.x < min.x {
                                mag += (min.x - pos.x).powi(2);
                            }
                            if pos.y < min.y {
                                mag += (min.y - pos.y).powi(2);
                            }
                            if pos.z < min.z {
                                mag += (min.z - pos.z).powi(2);
                            }
                            if pos.x > max.x {
                                mag += (pos.x - max.x).powi(2);
                            }
                            if pos.y > max.y {
                                mag += (pos.y - max.y).powi(2);
                            }
                            if pos.z > max.z {
                                mag += (pos.z - max.z).powi(2);
                            }
                            let magnitude = mag.sqrt();

                            if magnitude > 1e-9 {
                                violations.push(ConstraintViolation {
                                    constraint: constraint.clone(),
                                    waypoint: wp_idx,
                                    magnitude,
                                    message: format!(
                                        "Position ({:.3}, {:.3}, {:.3}) outside cartesian box at waypoint {}",
                                        pos.x, pos.y, pos.z, wp_idx,
                                    ),
                                });
                            }
                        }
                    }
                    Constraint::Composite(inner) => {
                        let inner_violations =
                            self.evaluate_trajectory(inner, trajectory, chain, fk, tcp);
                        violations.extend(
                            inner_violations
                                .into_iter()
                                .filter(|v| v.waypoint == wp_idx),
                        );
                    }
                }
            }
        }

        violations
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::forward::ForwardKinematics;
    use crate::models::{RobotModel, RobotRegistry};
    use crate::trajectory::{Trajectory, TrajectoryPoint};

    fn make_planar2r_trajectory() -> Trajectory {
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.3], 0.5),
            TrajectoryPoint::new(vec![1.0, 0.5], 1.0),
        ])
    }

    #[test]
    fn no_violations_for_valid_trajectory() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let fk = ForwardKinematics::new(chain.clone());
        let traj = make_planar2r_trajectory();
        let constraints = vec![
            Constraint::JointLimit {
                joint: 0,
                min: -2.0,
                max: 2.0,
            },
            Constraint::JointLimit {
                joint: 1,
                min: -2.0,
                max: 2.0,
            },
        ];

        let evaluator = DefaultConstraintEvaluator;
        let violations = evaluator.evaluate_trajectory(&constraints, &traj, &chain, &fk, None);
        assert!(
            violations.is_empty(),
            "expected no violations, got {:?}",
            violations
        );
    }

    #[test]
    fn detects_joint_limit_violation() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let fk = ForwardKinematics::new(chain.clone());
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![3.0, 0.0], 0.5), // joint 0 > 2.0
        ]);

        let constraints = vec![Constraint::JointLimit {
            joint: 0,
            min: -2.0,
            max: 2.0,
        }];

        let evaluator = DefaultConstraintEvaluator;
        let violations = evaluator.evaluate_trajectory(&constraints, &traj, &chain, &fk, None);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].waypoint, 1);
        assert!(violations[0].message.contains("above limit"));
    }

    #[test]
    fn composite_constraint_evaluates_all_sub() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let fk = ForwardKinematics::new(chain.clone());
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![3.0, 2.5], 0.5),
        ]);

        let constraints = vec![Constraint::Composite(vec![
            Constraint::JointLimit {
                joint: 0,
                min: -2.0,
                max: 2.0,
            },
            Constraint::JointLimit {
                joint: 1,
                min: -2.0,
                max: 2.0,
            },
        ])];

        let evaluator = DefaultConstraintEvaluator;
        let violations = evaluator.evaluate_trajectory(&constraints, &traj, &chain, &fk, None);
        assert_eq!(violations.len(), 2, "both joints violated at waypoint 1");
    }
}
