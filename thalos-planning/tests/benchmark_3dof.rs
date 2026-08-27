//! Benchmark: 3DOF manipulator — qué operadores realmente mejoran la trayectoria.
//!
//! Crea trayectorias con problemas conocidos y ejecuta cada operador,
//! reportando métricas antes/después y mejora porcentual.

use thalos_core::{
    models::manipulator_3dof::Manipulator3DOFSpec,
    robot::serial_chain::SerialChain,
    trajectory::{Trajectory, TrajectoryPoint},
};
use thalos_optimization::{
    TrajectoryOperator,
    domain::context::{JointLimits, OptimizationContext, PipelineConfig},
    operators::JointCenteringOperator,
    pipeline::OptimizationPipeline,
};
use thalos_planning::{evaluation::evaluator::PlanEvaluator, evaluation::metrics::PlanMetrics};

fn build_chain() -> SerialChain {
    Manipulator3DOFSpec::ideal().build()
}

fn compute_metrics(traj: &Trajectory) -> PlanMetrics {
    PlanEvaluator::compute_metrics_from_joints(traj)
}

fn pct(before: f64, after: f64) -> String {
    if before.abs() < 1e-12 {
        return "N/A".into();
    }
    format!("{:+.1}%", (after - before) / before * 100.0)
}

fn make_ctx(chain: &SerialChain) -> OptimizationContext {
    let lower: Vec<f64> = chain
        .segments
        .iter()
        .map(|s| s.joint.limits().min)
        .collect();
    let upper: Vec<f64> = chain
        .segments
        .iter()
        .map(|s| s.joint.limits().max)
        .collect();
    OptimizationContext {
        joint_limits: JointLimits {
            lower,
            upper,
            velocity: None,
            acceleration: None,
        },
        config: PipelineConfig::default(),
        tool_frame: None,
    }
}

fn run_operator(
    name: &str,
    op: &dyn TrajectoryOperator,
    chain: &SerialChain,
    traj: &Trajectory,
    region: &thalos_core::analysis::region::ProblemRegion,
    before: &PlanMetrics,
    ctx: &OptimizationContext,
) {
    let result = op.apply(chain, traj, region, ctx, None);
    match result {
        Ok(new_traj) => {
            let m = compute_metrics(&new_traj);
            println!(
                "  ◉ {:<22} smooth={:.4} ({}), manip_avg={:.4} ({}), joint_margin={:.4} ({})",
                name,
                m.smoothness,
                pct(before.smoothness, m.smoothness),
                m.manipulability.average,
                pct(before.manipulability.average, m.manipulability.average),
                m.joint_safety.min_margin,
                pct(before.joint_safety.min_margin, m.joint_safety.min_margin),
            );
        }
        Err(e) => println!("  ✗ {:<22} {:?}", name, e),
    }
}

fn trajectory_near_limits() -> Trajectory {
    let mut pts = Vec::new();
    for i in 0..30 {
        let t = i as f64 * 0.1;
        let q2 = 2.8 + (i as f64 * 0.01);
        pts.push(TrajectoryPoint::new(
            vec![0.5 + i as f64 * 0.02, q2.min(3.0), 0.3 + i as f64 * 0.01],
            t,
        ));
    }
    Trajectory::new(pts)
}

fn regions_near_limits() -> Vec<thalos_core::analysis::region::ProblemRegion> {
    use thalos_core::analysis::region::*;
    vec![ProblemRegion::new(
        RegionId(0),
        RegionKind::Constraint,
        RegionSeverity::Warning,
        10..25,
    )]
}

fn trajectory_near_singularity() -> Trajectory {
    let mut pts = Vec::new();
    for i in 0..30 {
        let t = i as f64 * 0.1;
        let q = if i >= 10 && i < 20 {
            vec![0.02 * (i - 10) as f64, 0.01 * (i - 10) as f64, 0.5]
        } else {
            vec![0.5 + i as f64 * 0.05, 0.3 + i as f64 * 0.05, 0.5]
        };
        pts.push(TrajectoryPoint::new(q, t));
    }
    Trajectory::new(pts)
}

fn regions_singularity() -> Vec<thalos_core::analysis::region::ProblemRegion> {
    use thalos_core::analysis::region::*;
    vec![ProblemRegion::new(
        RegionId(0),
        RegionKind::Singularity,
        RegionSeverity::Critical,
        10..20,
    )]
}

// ═══════════════════════════════════════════════════════════════════
//  ADAPTIVE SAMPLING — BASELINE BENCHMARK (antes de implementar)
// ═══════════════════════════════════════════════════════════════════
//
// Objetivo: medir el estado actual de trayectorias con cambios bruscos
// para poder comparar después de implementar AdaptiveSampling.

/// Trayectoria 3D con cambios bruscos (sharp bends) que un
/// AdaptiveSampling debería detectar y resolver mejor que
/// un SplitSegment uniforme.
fn trajectory_sharp_bends() -> Trajectory {
    let mut pts = Vec::new();
    for i in 0..50 {
        let t = i as f64 * 0.1;
        // Segmento 1: suave (0-15)
        // Segmento 2: cambio brusco (15-25) — simula un sharp bend
        // Segmento 3: suave (25-50)
        let (j1, j2, j3) = if i < 15 {
            (0.1 * i as f64 * 0.05, 0.1 + i as f64 * 0.02, 0.0)
        } else if i < 25 {
            // Cambio brusco: joint 1 salta, joint 2 cambia dirección
            let p = (i - 15) as f64 / 10.0;
            (1.0 + p * 1.5, 0.4 + p * (-0.8), 0.0 + p * 0.5)
        } else {
            let p = (i - 25) as f64;
            (2.5 + p * 0.01, -0.4 + p * 0.01, 0.5)
        };
        pts.push(TrajectoryPoint::new(vec![j1, j2, j3], t));
    }
    Trajectory::new(pts)
}

/// Calcula la máxima diferencia entre waypoints consecutivos
/// como proxy de "cambio brusco" (max joint velocity).
fn max_joint_delta(traj: &Trajectory) -> f64 {
    let wps = traj.waypoints();
    if wps.len() < 2 {
        return 0.0;
    }
    (0..wps.len() - 1)
        .map(|i| {
            let a = wps[i].joints();
            let b = wps[i + 1].joints();
            a.iter()
                .zip(b.iter())
                .map(|(ai, bi)| (bi - ai).abs())
                .sum::<f64>()
        })
        .fold(0.0, f64::max)
}

#[test]
fn benchmark_adaptive_sampling_baseline() {
    println!("\n═══════════════════════════════════════════════════");
    println!("  BASELINE: AdaptiveSampling (ANTES de implementar)");
    println!("═══════════════════════════════════════════════════\n");

    let traj = trajectory_sharp_bends();
    let m = compute_metrics(&traj);

    println!("  Trayectoria: 50 waypoints, cambio brusco en idx 15-25");
    println!("  Waypoints:   {}", traj.waypoints().len());
    println!("  Smoothness:  {:.4}", m.smoothness);
    println!(
        "  Max delta:   {:.4} rad (proxy de cambio brusco)",
        max_joint_delta(&traj)
    );

    println!("\n═══════════════════════════════════════════════════");
    println!("  FIN BASELINE — implementar AdaptiveSampling y re-ejecutar");
    println!("═══════════════════════════════════════════════════\n");
}

#[test]
fn benchmark_all_operators_on_3dof() {
    let chain = build_chain();

    println!("\n═══════════════════════════════════════════════════");
    println!("  BENCHMARK: Operadores en manipulador 3DOF");
    println!("═══════════════════════════════════════════════════\n");

    // Trayectoria 1: Joints cerca del límite
    println!("─── TRAYECTORIA 1: Joints cerca del límite ───");
    let traj1 = trajectory_near_limits();
    let m1 = compute_metrics(&traj1);
    println!(
        "  Antes: smooth={:.4}, manip_avg={:.4}, joint_margin={:.4}",
        m1.smoothness, m1.manipulability.average, m1.joint_safety.min_margin
    );
    let reg1 = &regions_near_limits()[0];
    let ctx = make_ctx(&chain);
    println!(
        "  Joint limits: {:?} to {:?}",
        ctx.joint_limits.lower, ctx.joint_limits.upper
    );

    run_operator(
        "JointCentering(0.3)",
        &JointCenteringOperator::new(0.3),
        &chain,
        &traj1,
        reg1,
        &m1,
        &ctx,
    );

    // Trayectoria 2: Singularidad
    println!("\n─── TRAYECTORIA 2: Cerca de singularidad ───");
    let traj2 = trajectory_near_singularity();
    let m2 = compute_metrics(&traj2);
    println!(
        "  Antes: smooth={:.4}, manip_avg={:.4}",
        m2.smoothness, m2.manipulability.average
    );
    let reg2 = &regions_singularity()[0];

    run_operator(
        "JointCentering(0.3)",
        &JointCenteringOperator::new(0.3),
        &chain,
        &traj2,
        reg2,
        &m2,
        &ctx,
    );

    // ═══ VALIDACIÓN: Pipeline + blending ═══
    println!("\n─── VALIDACIÓN: Pipeline (con blending) vs traj completa ───");

    // Helper to compute one pipeline result
    let pipeline_jc =
        |label: &str,
         traj: &Trajectory,
         before: &PlanMetrics,
         regions: Vec<thalos_core::analysis::region::ProblemRegion>| {
            let jc = JointCenteringOperator::new(0.3);
            let ops: Vec<&dyn TrajectoryOperator> = vec![&jc];
            let pipeline = OptimizationPipeline::new(PipelineConfig::default());
            match pipeline.optimize_regions(&ops, &chain, traj, &regions, before, &ctx, None) {
                Ok(r) => {
                    let m = compute_metrics(&r.trajectory);
                    println!(
                        "  Pipeline {}: smooth={:.4} ({}), joint_margin={:.4} ({})",
                        label,
                        m.smoothness,
                        pct(before.smoothness, m.smoothness),
                        m.joint_safety.min_margin,
                        pct(before.joint_safety.min_margin, m.joint_safety.min_margin),
                    );
                }
                Err(e) => println!("  Pipeline {}: {:?}", label, e),
            }
        };
    pipeline_jc("traj1", &traj1, &m1, regions_near_limits());
    pipeline_jc("traj2", &traj2, &m2, regions_singularity());

    // ─── EXPERIMENTO: JointCentering sobre trayectorias COMPLETAS ───
    println!("\n─── EXPERIMENTO: JC sobre trayectoria COMPLETA (sin bordes) ───");
    let full1 = thalos_core::analysis::region::ProblemRegion::new(
        thalos_core::analysis::region::RegionId(0),
        thalos_core::analysis::region::RegionKind::Constraint,
        thalos_core::analysis::region::RegionSeverity::Warning,
        0..traj1.waypoints().len(),
    );
    run_operator(
        "JC full traj1",
        &JointCenteringOperator::new(0.3),
        &chain,
        &traj1,
        &full1,
        &m1,
        &ctx,
    );
    let full2 = thalos_core::analysis::region::ProblemRegion::new(
        thalos_core::analysis::region::RegionId(0),
        thalos_core::analysis::region::RegionKind::Singularity,
        thalos_core::analysis::region::RegionSeverity::Critical,
        0..traj2.waypoints().len(),
    );
    run_operator(
        "JC full traj2",
        &JointCenteringOperator::new(0.3),
        &chain,
        &traj2,
        &full2,
        &m2,
        &ctx,
    );

    println!("\n─── TABLA RESUMEN ───");
    println!(
        "  {:<30}  {:>12}  {:>16}",
        "Caso", "Smooth Δ", "Joint Margin Δ"
    );
    println!("  {:-<30}  {:->12}  {:->16}", "", "", "");

    println!("\n═══════════════════════════════════════════════════");
    println!("  FIN DEL BENCHMARK");
    println!("═══════════════════════════════════════════════════\n");
}
