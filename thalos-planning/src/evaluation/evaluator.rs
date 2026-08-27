use thalos_core::trajectory::Trajectory;

use crate::evaluation::metrics::PlanMetrics;

/// Convierte análisis de waypoints en métricas agregadas.
///
/// Stateless.
pub struct PlanEvaluator;

impl PlanEvaluator {
    /// Computar métricas directamente desde una trayectoria (sin análisis completo).
    ///
    /// Útil para evaluar candidatos de reparación (M8.2).
    /// No produce análisis de manipulabilidad, singularidad ni colisiones.
    pub fn compute_metrics_from_joints(trajectory: &Trajectory) -> PlanMetrics {
        let wps = trajectory.waypoints();
        if wps.is_empty() {
            return PlanMetrics {
                length: 0.0,
                waypoint_count: 0,
                manipulability: crate::evaluation::metrics::ManipulabilityMetrics::new(
                    0.0, 0.0, 0, 0,
                ),
                joint_safety: crate::evaluation::metrics::JointSafetyMetrics::new(1.0, 0.0, 0),
                collision: crate::evaluation::metrics::CollisionMetrics::new(f64::MAX, 0, 0),
                smoothness: 0.0,
                orientation_change: 0.0,
            };
        }

        // Length
        let length: f64 = wps
            .windows(2)
            .map(|w| {
                w[1].joints()
                    .iter()
                    .zip(w[0].joints())
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt()
            })
            .sum();

        // Joint safety (no manipulability data without analysis)
        let min_margin = wps
            .iter()
            .flat_map(|wp| {
                wp.joints().iter().map(|&q| {
                    let range = std::f64::consts::PI;
                    1.0 - (q.abs() / range).clamp(0.0, 1.0)
                })
            })
            .fold(f64::MAX, f64::min);

        let avg_util = {
            let total_max: f64 = wps
                .iter()
                .map(|wp| {
                    wp.joints()
                        .iter()
                        .map(|&q| (q.abs() / std::f64::consts::PI).clamp(0.0, 1.0))
                        .fold(0.0f64, f64::max)
                })
                .sum();
            total_max / wps.len() as f64
        };

        let violation_count = wps
            .iter()
            .filter(|wp| {
                wp.joints()
                    .iter()
                    .any(|&q| q.abs() > std::f64::consts::PI - 0.01)
            })
            .count();

        // Smoothness
        let smoothness = if wps.len() >= 3 {
            wps.windows(3)
                .map(|w| {
                    let dt = (w[2].timestamp() - w[0].timestamp()).max(1e-6);
                    let jerk: f64 = w[2]
                        .joints()
                        .iter()
                        .zip(w[1].joints())
                        .zip(w[0].joints())
                        .map(|((c, b), a)| ((c - 2.0 * b + a) / dt).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    jerk
                })
                .sum::<f64>()
                / (wps.len() - 2) as f64
        } else {
            0.0
        };

        // Orientation change (estimated from joint deltas)
        let orientation_change: f64 = wps
            .windows(2)
            .map(|w| {
                w[1].joints()
                    .iter()
                    .zip(w[0].joints())
                    .map(|(a, b)| (a - b).abs())
                    .sum::<f64>()
                    * 0.1
            })
            .sum();

        PlanMetrics {
            length,
            waypoint_count: wps.len(),
            manipulability: crate::evaluation::metrics::ManipulabilityMetrics::new(0.0, 0.0, 0, 0),
            joint_safety: crate::evaluation::metrics::JointSafetyMetrics::new(
                min_margin,
                avg_util,
                violation_count,
            ),
            collision: crate::evaluation::metrics::CollisionMetrics::new(f64::MAX, 0, 0),
            smoothness,
            orientation_change,
        }
    }
}
