//! Acceptance Policy — evalua si un candidato del operador es mejor
//! que la trayectoria actual antes de aceptarlo.
//!
//! El pipeline produce un **candidato** via `op.apply()`. Este módulo
//! compara métricas clave (segment error, joint margin) entre el estado
//! actual y el candidato para decidir si se acepta o rechaza.
//!
//! Todas las métricas se computan en JOINT SPACE — no requieren
//! ForwardKinematics. El costo de evaluación es O(n·dof) por región.
//!
//! # Uso
//!
//! ```ignore
//! let evaluation = AcceptancePolicy::evaluate(&current, &candidate, region, ctx);
//! if evaluation.accepted {
//!     current = candidate;
//! } else {
//!     log!("rejected: {}", evaluation.reason);
//! }
//! ```

use crate::domain::context::OptimizationContext;
use thalos_core::trajectory::Trajectory;

/// Resultado de evaluar un candidato contra el estado actual.
#[derive(Debug, Clone)]
pub struct AcceptanceEvaluation {
    /// `true` si el candidato es aceptable.
    pub accepted: bool,
    /// Descripción legible de por qué fue aceptado o rechazado.
    pub reason: String,
    /// Degradación de segment error (1.0 = igual, >1.0 = peor).
    pub segment_error_ratio: f64,
    /// Degradación de joint margin (1.0 = igual, <1.0 = peor).
    pub joint_margin_ratio: f64,
}

impl AcceptanceEvaluation {
    fn accepted(reason: &str, seg_ratio: f64, margin_ratio: f64) -> Self {
        Self {
            accepted: true,
            reason: reason.to_string(),
            segment_error_ratio: seg_ratio,
            joint_margin_ratio: margin_ratio,
        }
    }

    fn rejected(reason: &str, seg_ratio: f64, margin_ratio: f64) -> Self {
        Self {
            accepted: false,
            reason: reason.to_string(),
            segment_error_ratio: seg_ratio,
            joint_margin_ratio: margin_ratio,
        }
    }
}

/// Política de aceptación configurable.
///
/// Los thresholds definen cuánta degradación se tolera antes de rechazar.
pub struct AcceptancePolicy {
    /// Razón máxima permitida de empeoramiento de segment error.
    /// 1.20 = se tolera hasta 20% peor.
    pub max_segment_error_ratio: f64,
    /// Razón mínima permitida de empeoramiento de joint margin.
    /// 0.95 = se tolera hasta 5% de reducción.
    pub min_joint_margin_ratio: f64,
    /// Si se excede este ratio, se rechaza aunque el segment error
    /// y joint margin estén bien (por si se agregaron métricas).
    pub max_waypoint_count_ratio: f64,
}

impl Default for AcceptancePolicy {
    fn default() -> Self {
        Self {
            // Segment error puede empeorar hasta 20% — más que eso
            // indica que el operador está desestabilizando la trayectoria.
            max_segment_error_ratio: 1.20,
            // Joint margin puede bajar hasta 5% — más que eso es riesgoso.
            min_joint_margin_ratio: 0.95,
            // Waypoint count no debería aumentar más de 5× por región.
            max_waypoint_count_ratio: 5.0,
        }
    }
}

impl AcceptancePolicy {
    /// Evalúa un candidato contra la trayectoria actual.
    ///
    /// # Argumentos
    /// * `current` — trayectoria actual (pre-aplicación)
    /// * `candidate` — trayectoria candidato (post-blend)
    /// * `ctx` — contexto (para joint limits)
    ///
    /// # Returns
    /// `AcceptanceEvaluation` con decisión y métricas.
    pub fn evaluate(
        &self,
        current: &Trajectory,
        candidate: &Trajectory,
        ctx: &OptimizationContext,
    ) -> AcceptanceEvaluation {
        // 1. Compute segment error (antes vs después)
        let cur_seg_err = max_segment_error(current);
        let cand_seg_err = max_segment_error(candidate);

        // 2. Compute joint margin
        let cur_margin =
            min_joint_margin(current, &ctx.joint_limits.lower, &ctx.joint_limits.upper);
        let cand_margin =
            min_joint_margin(candidate, &ctx.joint_limits.lower, &ctx.joint_limits.upper);

        // 3. Compute waypoint count ratio
        let cur_len = current.len().max(1);
        let cand_len = candidate.len();
        let count_ratio = cand_len as f64 / cur_len as f64;

        // Ratios: >1.0 significa que el candidato empeoró
        let seg_ratio = if cur_seg_err > 1e-12 {
            cand_seg_err / cur_seg_err
        } else {
            1.0
        };

        let margin_ratio = if cur_margin > 1e-12 {
            cand_margin / cur_margin
        } else {
            // Si el margen actual es ~0, no podemos empeorarlo
            // pero si el candidato también es 0, no penalizamos.
            if cand_margin < 1e-12 { 1.0 } else { 0.5 }
        };

        // 4. Decision
        let mut reasons: Vec<String> = Vec::new();

        if seg_ratio > self.max_segment_error_ratio {
            reasons.push(format!(
                "segment error +{:.1}% (limit +{}%)",
                (seg_ratio - 1.0) * 100.0,
                (self.max_segment_error_ratio - 1.0) * 100.0,
            ));
        }

        if margin_ratio < self.min_joint_margin_ratio {
            reasons.push(format!(
                "joint margin {:.1}% (limit {:.1}%)",
                margin_ratio * 100.0,
                self.min_joint_margin_ratio * 100.0,
            ));
        }

        if count_ratio > self.max_waypoint_count_ratio {
            reasons.push(format!(
                "waypoints {:.0}× (limit {:.0}×)",
                count_ratio, self.max_waypoint_count_ratio,
            ));
        }

        if reasons.is_empty() {
            let mut details = vec![];
            if seg_ratio > 1.0 {
                details.push(format!("segment error +{:.1}%", (seg_ratio - 1.0) * 100.0));
            } else {
                details.push(format!("segment error -{:.1}%", (1.0 - seg_ratio) * 100.0));
            }
            if margin_ratio > 1.0 {
                details.push(format!("margin +{:.1}%", (margin_ratio - 1.0) * 100.0));
            } else if margin_ratio < 1.0 {
                details.push(format!("margin -{:.1}%", (1.0 - margin_ratio) * 100.0));
            } else {
                details.push("margin unchanged".to_string());
            }
            AcceptanceEvaluation::accepted(&details.join(", "), seg_ratio, margin_ratio)
        } else {
            AcceptanceEvaluation::rejected(&reasons.join("; "), seg_ratio, margin_ratio)
        }
    }
}

// ── Metric helpers (solo joint space, sin FK) ────────────────

/// Máxima distancia L2 en joint space entre waypoints consecutivos.
fn max_segment_error(traj: &Trajectory) -> f64 {
    let wps = traj.waypoints();
    if wps.len() < 2 {
        return 0.0;
    }
    let mut max_err = 0.0;
    for i in 0..wps.len() - 1 {
        let err: f64 = wps[i]
            .joints()
            .iter()
            .zip(wps[i + 1].joints().iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        if err > max_err {
            max_err = err;
        }
    }
    max_err
}

/// Mínima distancia de cualquier joint a su límite mecánico.
fn min_joint_margin(traj: &Trajectory, lower: &[f64], upper: &[f64]) -> f64 {
    if lower.is_empty() || upper.is_empty() {
        return f64::INFINITY;
    }
    traj.waypoints()
        .iter()
        .flat_map(|wp| {
            wp.joints()
                .iter()
                .zip(lower.iter().zip(upper.iter()))
                .map(|(q, (lo, hi))| (q - lo).abs().min((hi - q).abs()))
        })
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::context::JointLimits;
    use thalos_core::trajectory::{Trajectory, TrajectoryPoint};

    fn identity_traj() -> Trajectory {
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.5], 1.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 2.0),
        ])
    }

    fn bad_traj() -> Trajectory {
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![5.0, 5.0], 1.0), // huge jump
            TrajectoryPoint::new(vec![1.0, 1.0], 2.0),
        ])
    }

    fn test_ctx() -> OptimizationContext {
        OptimizationContext {
            joint_limits: JointLimits {
                lower: vec![-2.0, -2.0],
                upper: vec![2.0, 2.0],
                velocity: None,
                acceleration: None,
            },
            ..OptimizationContext::default()
        }
    }

    #[test]
    fn identical_trajectory_is_accepted() {
        let traj = identity_traj();
        let policy = AcceptancePolicy::default();
        let eval = policy.evaluate(&traj, &traj, &test_ctx());
        assert!(eval.accepted, "identical traj should be accepted");
        assert!((eval.segment_error_ratio - 1.0).abs() < 1e-10);
    }

    #[test]
    fn worse_segment_error_is_rejected() {
        let current = identity_traj();
        let candidate = bad_traj();
        let policy = AcceptancePolicy::default();
        let eval = policy.evaluate(&current, &candidate, &test_ctx());
        assert!(!eval.accepted, "worse segment error should be rejected");
        assert!(eval.segment_error_ratio > 1.20, "expected >1.20");
    }

    #[test]
    fn slightly_worse_segment_error_is_tolerated() {
        let current = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
        ]);
        // 5% worse → should pass (tolerance is 20%)
        let candidate = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.05, 1.05], 1.0),
        ]);
        let policy = AcceptancePolicy::default();
        let eval = policy.evaluate(&current, &candidate, &test_ctx());
        assert!(eval.accepted, "5% worse should be tolerated");
    }

    #[test]
    fn degraded_joint_margin_is_rejected() {
        let current = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.5], 1.0),
        ]);
        // Candidate pushes one joint near limit
        let candidate = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.9, 0.5], 1.0), // joint 0 at 1.9, limit=2.0 → margin=0.1
        ]);
        let ctx = OptimizationContext {
            joint_limits: JointLimits {
                lower: vec![-2.0, -2.0],
                upper: vec![2.0, 2.0],
                velocity: None,
                acceleration: None,
            },
            ..OptimizationContext::default()
        };
        let policy = AcceptancePolicy::default();
        let eval = policy.evaluate(&current, &candidate, &ctx);
        assert!(!eval.accepted, "degraded margin should be rejected");
    }

    #[test]
    fn zero_margin_current_does_not_panic() {
        // Both current and candidate have zero margin → should accept
        let traj = Trajectory::new(vec![TrajectoryPoint::new(vec![-2.0, 2.0], 0.0)]);
        let ctx = OptimizationContext {
            joint_limits: JointLimits {
                lower: vec![-2.0, -2.0],
                upper: vec![2.0, 2.0],
                velocity: None,
                acceleration: None,
            },
            ..OptimizationContext::default()
        };
        let policy = AcceptancePolicy::default();
        let eval = policy.evaluate(&traj, &traj, &ctx);
        // margin_ratio should be 1.0 even though margin itself is 0
        assert!(eval.accepted);
        assert!((eval.joint_margin_ratio - 1.0).abs() < 1e-10);
    }

    #[test]
    fn waypoint_explosion_is_caught() {
        let current = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
        ]);
        // 100× waypoint count → should be rejected
        let candidate = Trajectory::new(
            (0..=100)
                .map(|i| {
                    let t = i as f64 * 0.01;
                    TrajectoryPoint::new(vec![t, t], t)
                })
                .collect(),
        );
        let policy = AcceptancePolicy::default();
        let eval = policy.evaluate(&current, &candidate, &test_ctx());
        assert!(!eval.accepted, "waypoint explosion should be rejected");
    }
}
