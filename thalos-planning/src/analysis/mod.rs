//! Análisis de trayectorias planificadas.
//!
//! Evalúa cada waypoint de una trayectoria para detectar:
//! - Singularidades
//! - Manipulabilidad
//! - Colisiones y distancia a obstáculos
//! - Violaciones de constraints
//!
//! El camino canónico es [`TrajectoryAnalyzer::analyze`], que emite
//! [`Observation`](thalos_core::analysis::observation::Observation)s ancladas al
//! artefacto analizado (I3); el `DefaultAggregator` las agrega a un
//! `AnalysisReport`. El camino técnico
//! [`TrajectoryAnalyzer::analyze_plan`] produce un [`PlanAnalysis`] con datos
//! por waypoint y métricas agregadas (consumido por el pipeline de
//! optimización). Ambos comparten UN solo pasa de evaluación
//! (`analyze_with_observations`), nunca evalúan la trayectoria dos veces.

pub mod domain;

use std::collections::BTreeMap;

use thalos_collision::distance::geometries_distance;
use thalos_core::{
    analysis::{
        attribute_value::AttributeValue,
        constraints::{Constraint, ConstraintEvaluator, ConstraintViolation},
        location::Location,
        observation::{ArtifactRef, Observation, ObservationId, ObservationKind, Severity},
    },
    collision::{CollisionBodyBuilder, CollisionChecker, CollisionMatrix, EntityId},
    kinematics::{
        forward::ForwardKinematics,
        jacobian::{
            GeometricJacobian, JacobianSolver, manipulability::ManipulabilityReport,
            singularity::SingularityReport,
        },
    },
    robot::{serial_chain::SerialChain, tool_frame::ToolFrame},
    spatial::frame::FrameId,
    trajectory::Trajectory,
};

use crate::error::PlanningError;

// ─── Data types ───────────────────────────────────────────────────

/// Análisis completo de una trayectoria.
#[derive(Debug, Clone)]
pub struct PlanAnalysis {
    /// Datos por waypoint.
    pub waypoints: Vec<WaypointAnalysis>,
    /// Métricas agregadas de toda la trayectoria.
    pub metrics: AnalysisMetrics,
    /// Violaciones de constraints, si se evaluaron.
    pub constraint_violations: Vec<ConstraintViolation>,
}

/// Análisis de un waypoint individual.
#[derive(Debug, Clone)]
pub struct WaypointAnalysis {
    /// Índice del waypoint.
    pub index: usize,
    /// Tiempo del waypoint (segundos).
    pub timestamp: f64,
    /// Configuración articular.
    pub joints: Vec<f64>,
    /// Reporte de singularidad.
    pub singularity: Option<SingularityReport>,
    /// Reporte de manipulabilidad.
    pub manipulability: Option<ManipulabilityReport>,
    /// Distancia mínima a obstáculos (negativo = colisión).
    pub min_collision_distance: Option<f64>,
}

/// Métricas agregadas de la trayectoria completa.
#[derive(Debug, Clone)]
pub struct AnalysisMetrics {
    /// Cantidad de waypoints analizados.
    pub waypoint_count: usize,
    /// Duración total estimada (segundos).
    pub trajectory_duration: f64,
    /// Manipulabilidad promedio (Yoshikawa) sobre todos los waypoints.
    pub avg_manipulability: Option<f64>,
    /// Manipulabilidad mínima (Yoshikawa).
    pub min_manipulability: Option<f64>,
    /// Cantidad de waypoints near-singular.
    pub near_singular_count: usize,
    /// Cantidad de waypoints singulares.
    pub singular_count: usize,
    /// Distancia mínima a obstáculos en toda la trayectoria.
    pub min_collision_distance: Option<f64>,
    /// Índice del waypoint con distancia mínima.
    pub min_collision_waypoint: Option<usize>,
    /// Si la trayectoria tiene colisiones.
    pub has_collisions: bool,
    /// Primera colisión detectada (waypoint).
    pub first_collision_waypoint: Option<usize>,
}

impl AnalysisMetrics {
    /// Proyección a `BTreeMap<String, f64>` para `AnalysisReport.metrics`
    /// (design P3: el servicio puebla `report.metrics` desde el análisis
    /// técnico; claves estables del dominio).
    ///
    /// Los optionals ausentes (`avg_manipulability`, `min_manipulability`,
    /// `min_collision_distance`) se OMITEN — nunca se proyecta `null`/`NaN` al
    /// wire. Los agregados escalares (`waypoint_count`, `trajectory_duration`,
    /// `near_singular_count`, `singular_count`, `has_collisions` como 1.0/0.0)
    /// siempre están presentes.
    pub fn to_btree_map(&self) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        map.insert("waypoint_count".to_string(), self.waypoint_count as f64);
        map.insert("trajectory_duration".to_string(), self.trajectory_duration);
        map.insert(
            "near_singular_count".to_string(),
            self.near_singular_count as f64,
        );
        map.insert("singular_count".to_string(), self.singular_count as f64);
        map.insert(
            "has_collisions".to_string(),
            if self.has_collisions { 1.0 } else { 0.0 },
        );
        if let Some(avg) = self.avg_manipulability {
            map.insert("avg_manipulability".to_string(), avg);
        }
        if let Some(min) = self.min_manipulability {
            map.insert("min_manipulability".to_string(), min);
        }
        if let Some(d) = self.min_collision_distance {
            map.insert("min_collision_distance".to_string(), d);
        }
        if let Some(wp) = self.min_collision_waypoint {
            map.insert("min_collision_waypoint".to_string(), wp as f64);
        }
        map
    }
}

// ─── TrajectoryAnalyzer ───────────────────────────────────────────

/// Analizador de trayectorias planificadas.
///
/// Evalúa cada waypoint contra criterios de calidad y seguridad.
/// No requiere estado mutable — todas las dependencias se inyectan.
pub struct TrajectoryAnalyzer<'a> {
    pub chain: &'a SerialChain,
    pub fk: ForwardKinematics,
    pub end_effector: FrameId,
    pub tcp: Option<&'a ToolFrame>,
    pub collision_checker: Option<&'a dyn CollisionChecker>,
    pub collision_matrix: Option<&'a CollisionMatrix>,
    pub constraints: Option<&'a [Constraint]>,
    pub constraint_evaluator: Option<&'a dyn ConstraintEvaluator>,
    pub ik_solver: Option<&'a dyn thalos_core::kinematics::inverse::IKSolver>,
}

impl<'a> TrajectoryAnalyzer<'a> {
    pub fn new(chain: &'a SerialChain, tcp: Option<&'a ToolFrame>) -> Self {
        let end_effector = tcp
            .map(|t| t.base_frame.clone())
            .unwrap_or_else(|| *chain.end_effector());
        let fk = ForwardKinematics::new(chain.clone());
        Self {
            chain,
            fk,
            end_effector,
            tcp,
            collision_checker: None,
            collision_matrix: None,
            constraints: None,
            constraint_evaluator: None,
            ik_solver: None,
        }
    }

    pub fn with_collision_checker(
        mut self,
        checker: &'a dyn CollisionChecker,
        matrix: &'a CollisionMatrix,
    ) -> Self {
        self.collision_checker = Some(checker);
        self.collision_matrix = Some(matrix);
        self
    }

    pub fn with_constraints(
        mut self,
        constraints: &'a [Constraint],
        evaluator: &'a dyn ConstraintEvaluator,
    ) -> Self {
        self.constraints = Some(constraints);
        self.constraint_evaluator = Some(evaluator);
        self
    }

    /// Análisis técnico completo por waypoint (camino de métricas).
    ///
    /// Produce un [`PlanAnalysis`] (waypoints + métricas) para los consumidores
    /// del análisis técnico: pipeline de optimización y harness pbm. El camino
    /// canónico de observaciones es [`Self::analyze`]; ambos comparten un solo
    /// pasa de evaluación vía [`Self::analyze_with_observations`].
    pub fn analyze_plan(&self, trajectory: &Trajectory) -> Result<PlanAnalysis, PlanningError> {
        self.analyze_internal(None, trajectory)
            .map(|(plan, _)| plan)
    }

    /// Emite las observaciones canónicas del análisis (PR 3, design D2).
    ///
    /// Cada [`Observation`](thalos_core::analysis::observation::Observation)
    /// queda anclada al `artifact` recibido (I3) y describe el fenómeno por
    /// `kind` + `location` (I2). La agregación a un
    /// [`AnalysisReport`](thalos_core::analysis::report::AnalysisReport) es
    /// responsabilidad del `DefaultAggregator`, nunca del analyzer (D2).
    ///
    /// El `message` de los hallazgos legacy se descarta (I1) — los renderers
    /// reconstruyen la presentación (cambio A). El `artifact` (I3) no vive en
    /// la observación sin ancla, por eso se recibe como parámetro.
    pub fn analyze(&self, artifact: ArtifactRef, trajectory: &Trajectory) -> Vec<Observation> {
        self.analyze_with_observations(artifact, trajectory)
            .expect("TrajectoryAnalyzer::analyze only fails on programmer error")
            .1
    }

    /// Pasa ÚNICO del análisis: análisis técnico por waypoint + observaciones
    /// canónicas en la misma evaluación (PR 7a).
    ///
    /// `analyze_plan` (consumidores de waypoints/métricas) y `analyze`
    /// (observaciones) comparten esta implementación. Los consumidores que
    /// necesitan ambos — el servicio de análisis y el harness pbm — la usan
    /// para NO evaluar la trayectoria dos veces.
    pub fn analyze_with_observations(
        &self,
        artifact: ArtifactRef,
        trajectory: &Trajectory,
    ) -> Result<(PlanAnalysis, Vec<Observation>), PlanningError> {
        self.analyze_internal(Some(&artifact), trajectory)
    }

    /// Evaluación única de la trayectoria: waypoints + métricas + violaciones
    /// de constraints + observaciones canónicas (solo si `artifact` es `Some`).
    fn analyze_internal(
        &self,
        artifact: Option<&ArtifactRef>,
        trajectory: &Trajectory,
    ) -> Result<(PlanAnalysis, Vec<Observation>), PlanningError> {
        let mut waypoints = Vec::with_capacity(trajectory.len());
        let mut total_yoshikawa = 0.0;
        let mut min_yoshikawa = f64::MAX;
        let mut yoshikawa_count = 0;
        let mut near_singular = 0;
        let mut singular = 0;
        let mut abs_min_collision = f64::MAX;
        let mut min_coll_wp = None;

        for (idx, wp) in trajectory.waypoints().iter().enumerate() {
            let q = wp.joints().to_vec();
            let fk_result = self.fk.evaluate(&q);

            // Jacobiano + singularidad
            let jacobian_solver =
                GeometricJacobian::new(self.fk.clone(), self.end_effector.clone());
            let jacobian = jacobian_solver.evaluate(&q);
            let singularity = SingularityReport::analyze(&jacobian);
            let manipulability = ManipulabilityReport::compute(&singularity);

            if singularity.condition_number < 100.0 {
                // Normal
            } else if singularity.condition_number < 1000.0 {
                near_singular += 1;
            } else {
                singular += 1;
            }

            total_yoshikawa += manipulability.yoshikawa;
            if manipulability.yoshikawa < min_yoshikawa {
                min_yoshikawa = manipulability.yoshikawa;
            }
            yoshikawa_count += 1;

            // Colisiones
            let min_collision = if let Some(checker) = self.collision_checker {
                let bodies = CollisionBodyBuilder::build(self.chain, &fk_result);
                let default_matrix = CollisionMatrix::new();
                let matrix = self.collision_matrix.unwrap_or(&default_matrix);
                let result = checker.check(&bodies, matrix);

                // Compute minimum distance between all body pairs
                let mut min_dist = f64::MAX;
                for i in 0..bodies.len() {
                    for j in (i + 1)..bodies.len() {
                        if let (EntityId::Link(la), EntityId::Link(lb)) =
                            (&bodies[i].entity, &bodies[j].entity)
                        {
                            if matrix.is_ignored(*la, *lb) {
                                continue;
                            }
                        }
                        let d = geometries_distance(
                            &bodies[i].geometry,
                            &bodies[i].pose,
                            &bodies[j].geometry,
                            &bodies[j].pose,
                        );
                        if d < min_dist {
                            min_dist = d;
                        }
                    }
                }

                if !result.is_empty() {
                    // Collision detected — penetration
                    if min_dist > 0.0 {
                        min_dist = -0.001; // signal collision
                    }
                }

                if min_dist < abs_min_collision {
                    abs_min_collision = min_dist;
                    min_coll_wp = Some(idx);
                }

                Some(min_dist)
            } else {
                None
            };

            waypoints.push(WaypointAnalysis {
                index: idx,
                timestamp: wp.timestamp(),
                joints: q,
                singularity: Some(singularity),
                manipulability: Some(manipulability),
                min_collision_distance: min_collision,
            });
        }

        // Constraints
        let constraint_violations = if let Some(constraints) = self.constraints {
            if let Some(evaluator) = self.constraint_evaluator {
                evaluator.evaluate_trajectory(
                    constraints,
                    trajectory,
                    self.chain,
                    &self.fk,
                    self.tcp,
                )
            } else {
                vec![]
            }
        } else {
            vec![]
        };

        let metrics = AnalysisMetrics {
            waypoint_count: waypoints.len(),
            trajectory_duration: trajectory.duration(),
            avg_manipulability: if yoshikawa_count > 0 {
                Some(total_yoshikawa / yoshikawa_count as f64)
            } else {
                None
            },
            min_manipulability: if min_yoshikawa < f64::MAX {
                Some(min_yoshikawa)
            } else {
                None
            },
            near_singular_count: near_singular,
            singular_count: singular,
            min_collision_distance: if abs_min_collision < f64::MAX {
                Some(abs_min_collision)
            } else {
                None
            },
            min_collision_waypoint: min_coll_wp,
            has_collisions: abs_min_collision < 0.0 || abs_min_collision < 1e-9,
            first_collision_waypoint: if abs_min_collision < 0.0 {
                min_coll_wp
            } else {
                None
            },
        };

        // Observaciones canónicas — hechos objetivos derivados del análisis
        // (PR 3/7a). El artifact es requerido (I3): solo se emiten cuando el
        // caller pide observaciones (el camino técnico `analyze_plan` no).
        let mut observations: Vec<Observation> = Vec::new();
        if let Some(artifact) = artifact {
            let mut push = |kind: ObservationKind,
                            severity: Severity,
                            waypoint: Option<usize>,
                            value: Option<f64>,
                            threshold: Option<f64>| {
                let mut attributes = BTreeMap::new();
                if let Some(v) = value {
                    attributes.insert("value".to_string(), AttributeValue::Number(v));
                }
                if let Some(t) = threshold {
                    attributes.insert("threshold".to_string(), AttributeValue::Number(t));
                }
                observations.push(Observation {
                    id: ObservationId(0), // el aggregator reasigna 1..=n (I8)
                    kind,
                    severity,
                    artifact: artifact.clone(),
                    location: waypoint
                        .map(Location::Waypoint)
                        .unwrap_or(Location::Timestamp(0)),
                    attributes,
                    causes: Vec::new(),
                    related: Vec::new(),
                });
            };

            // Manipulabilidad baja (fenómeno de plan: promedio < umbral)
            if let Some(avg) = metrics.avg_manipulability {
                let manip_threshold = 0.3;
                if avg < manip_threshold {
                    // El waypoint con manipulabilidad mínima es el ancla
                    if let Some(worst) = waypoints
                        .iter()
                        .filter_map(|w| w.manipulability.as_ref().map(|m| (w.index, m.yoshikawa)))
                        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                    {
                        push(
                            ObservationKind::LowManipulability,
                            Severity::Warning,
                            Some(worst.0),
                            Some(worst.1),
                            Some(manip_threshold),
                        );
                    }
                }
            }

            // Singularidades
            for wp in &waypoints {
                if let Some(sr) = &wp.singularity {
                    if sr.condition_number >= 1000.0 {
                        push(
                            ObservationKind::Singularity,
                            Severity::Error,
                            Some(wp.index),
                            Some(sr.condition_number),
                            Some(1000.0),
                        );
                    } else if sr.condition_number >= 100.0 {
                        push(
                            ObservationKind::NearSingularity,
                            Severity::Warning,
                            Some(wp.index),
                            Some(sr.condition_number),
                            Some(100.0),
                        );
                    }
                }
            }

            // Colisiones
            if metrics.has_collisions {
                push(
                    ObservationKind::CollisionRisk,
                    Severity::Error,
                    metrics.first_collision_waypoint,
                    metrics.min_collision_distance,
                    Some(0.0),
                );
            } else if let Some(min_dist) = metrics.min_collision_distance {
                if min_dist < 0.05 {
                    push(
                        ObservationKind::CollisionNear,
                        Severity::Warning,
                        metrics.min_collision_waypoint,
                        Some(min_dist),
                        Some(0.05),
                    );
                }
            }

            // Violaciones de constraints
            for v in &constraint_violations {
                push(
                    ObservationKind::ConstraintViolation,
                    Severity::Error,
                    Some(v.waypoint),
                    Some(v.magnitude),
                    None,
                );
            }
        }

        Ok((
            PlanAnalysis {
                waypoints,
                metrics,
                constraint_violations,
            },
            observations,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_collision::NaiveCollisionChecker;
    use thalos_core::{
        analysis::observation::ArtifactRef,
        ids::MotionPlanId,
        models::{RobotModel, RobotRegistry},
        trajectory::TrajectoryPoint,
    };

    fn make_simple_trajectory() -> Trajectory {
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.3], 0.5),
            TrajectoryPoint::new(vec![1.0, 0.5], 1.0),
        ])
    }

    #[test]
    fn analyzes_all_waypoints() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = make_simple_trajectory();
        let analyzer = TrajectoryAnalyzer::new(&chain, None);

        let analysis = analyzer.analyze_plan(&traj).expect("analysis failed");
        assert_eq!(analysis.waypoints.len(), 3);
        assert!(analysis.metrics.waypoint_count == 3);
    }

    #[test]
    fn produces_manipulability_metrics() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = make_simple_trajectory();
        let analyzer = TrajectoryAnalyzer::new(&chain, None);

        let analysis = analyzer.analyze_plan(&traj).expect("analysis failed");
        assert!(analysis.metrics.avg_manipulability.is_some());
        assert!(analysis.metrics.avg_manipulability.unwrap() > 0.0);
    }

    #[test]
    fn detects_collisions_with_checker() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let checker = NaiveCollisionChecker;
        let matrix = CollisionMatrix::new();
        let traj = make_simple_trajectory();
        let analyzer =
            TrajectoryAnalyzer::new(&chain, None).with_collision_checker(&checker, &matrix);

        let analysis = analyzer.analyze_plan(&traj).expect("analysis failed");
        // Planar2R links are separated — should be no collisions
        assert!(!analysis.metrics.has_collisions);
    }

    // ─── Scenario 1: Perfect plan ────────────────────────────────

    #[test]
    fn scenario_perfect_plan_emits_no_observations() {
        // Trajectory where q2 ≈ π/2 (maximum manipulability)
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 1.57], 0.0),
            TrajectoryPoint::new(vec![0.3, 1.57], 0.5),
            TrajectoryPoint::new(vec![0.6, 1.57], 1.0),
        ]);
        let analyzer = TrajectoryAnalyzer::new(&chain, None);
        let artifact = ArtifactRef::MotionPlan(MotionPlanId("mp-perfect".to_string()));

        let observations = analyzer.analyze(artifact, &traj);
        assert!(
            observations.is_empty(),
            "Expected no observations for perfect plan, got {}: {:?}",
            observations.len(),
            observations.iter().map(|o| o.kind).collect::<Vec<_>>()
        );
    }

    // ─── Scenario 2: Low manipulability ──────────────────────────

    #[test]
    fn scenario_low_manipulability_emits_observations() {
        // Trajectory with q2 close to 0 (near-extended arm → low manipulability)
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.05], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.05], 0.5),
        ]);
        let analyzer = TrajectoryAnalyzer::new(&chain, None);
        let artifact = ArtifactRef::MotionPlan(MotionPlanId("mp-low-manip".to_string()));

        let observations = analyzer.analyze(artifact, &traj);
        assert!(
            !observations.is_empty(),
            "Expected observations for low-manipulability plan"
        );

        let has_low_manip = observations
            .iter()
            .any(|o| matches!(o.kind, ObservationKind::LowManipulability));
        let has_near_sing = observations
            .iter()
            .any(|o| matches!(o.kind, ObservationKind::NearSingularity));

        // Either observation is valid here — both indicate a problem
        assert!(
            has_low_manip || has_near_sing,
            "Expected LowManipulability or NearSingularity observation, got: {:?}",
            observations.iter().map(|o| o.kind).collect::<Vec<_>>()
        );
    }

    // ─── Scenario 3: Singularity ─────────────────────────────────

    #[test]
    fn scenario_singularity_emits_error_observation() {
        // Arm fully extended along X → singular configuration
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![TrajectoryPoint::new(vec![0.0, 0.0], 0.0)]);
        let analyzer = TrajectoryAnalyzer::new(&chain, None);
        let artifact = ArtifactRef::MotionPlan(MotionPlanId("mp-singular".to_string()));

        let observations = analyzer.analyze(artifact, &traj);
        // At q = [0, 0] the arm is fully extended → singular
        // Should have at least a NearSingularity or Singularity observation
        let has_singularity = observations.iter().any(|o| {
            matches!(
                o.kind,
                ObservationKind::Singularity | ObservationKind::NearSingularity
            )
        });

        assert!(
            has_singularity,
            "Expected Singularity or NearSingularity observation for fully-extended arm, got: {:?}",
            observations.iter().map(|o| o.kind).collect::<Vec<_>>()
        );
    }

    // ─── Scenario 5: Multiple problems ────────────────────────────

    #[test]
    fn scenario_multiple_problems_aggregate_correctly() {
        // Mix of good and bad waypoints
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),  // singular
            TrajectoryPoint::new(vec![0.5, 0.05], 0.5), // low manipulability
            TrajectoryPoint::new(vec![0.5, 1.57], 1.0), // good
        ]);
        let analyzer = TrajectoryAnalyzer::new(&chain, None);
        let artifact = ArtifactRef::MotionPlan(MotionPlanId("mp-mixed".to_string()));

        let observations = analyzer.analyze(artifact, &traj);
        // Should have observations from waypoints 0 and 1
        assert!(!observations.is_empty());
        // At least one observation from the singular waypoint
        let sing_observations = observations.iter().filter(|o| {
            matches!(
                o.kind,
                ObservationKind::Singularity
                    | ObservationKind::NearSingularity
                    | ObservationKind::LowManipulability
            )
        });
        assert!(sing_observations.count() >= 1);
    }

    // ─── PR 3: canonical pipeline (task 3.1) ───────────────────────
    //
    // TrajectoryAnalyzer → Vec<Observation> → DefaultAggregator → AnalysisReport.
    // I3: every observation (and the report) is anchored to the analyzed
    // MotionPlan. I2: NearSingularity/Singularity are identified by kind +
    // location, never by text.

    #[test]
    fn plan_pipeline_emits_observations_and_report() {
        use thalos_core::analysis::aggregator::{Aggregator, DefaultAggregator};
        use thalos_core::analysis::location::Location;
        use thalos_core::analysis::observation::{ArtifactRef, ObservationKind, Severity};
        use thalos_core::analysis::scoring::DefaultScoringPolicy;
        use thalos_core::ids::MotionPlanId;

        // Fully extended arm → singular configuration (same trajectory as the
        // legacy `scenario_singularity_generates_error` — fidelity anchor).
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![TrajectoryPoint::new(vec![0.0, 0.0], 0.0)]);
        let analyzer = TrajectoryAnalyzer::new(&chain, None);
        let artifact = ArtifactRef::MotionPlan(MotionPlanId("mp-pipeline".to_string()));

        // Canonical producer contract (D2): analyze emits observations, NOT a report.
        let observations = analyzer.analyze(artifact.clone(), &traj);
        assert!(
            !observations.is_empty(),
            "a singular trajectory must produce observations"
        );

        let report = DefaultAggregator::new(DefaultScoringPolicy).aggregate(artifact, observations);

        // I3: report + observations anchored to the analyzed MotionPlan.
        assert_eq!(
            report.artifact,
            ArtifactRef::MotionPlan(MotionPlanId("mp-pipeline".to_string()))
        );
        assert!(
            report
                .observations
                .iter()
                .all(|o| matches!(o.artifact, ArtifactRef::MotionPlan(_)))
        );

        // Fidelity: the fully-extended arm yields a Singularity or
        // NearSingularity observation (PR 3/7a vocabulary).
        assert!(
            report.observations.iter().any(|o| matches!(
                o.kind,
                ObservationKind::Singularity | ObservationKind::NearSingularity
            )),
            "expected Singularity/NearSingularity observation, got: {:?}",
            report
                .observations
                .iter()
                .map(|o| o.kind)
                .collect::<Vec<_>>()
        );

        // I2: the phenomenon is anchored at a waypoint, machine-readable.
        assert!(
            report
                .observations
                .iter()
                .any(|o| matches!(o.location, Location::Waypoint(_)))
        );
        // The full singularity is an Error (severity preserved from the
        // legacy detection — the observation keeps the same semantics).
        assert!(
            report
                .observations
                .iter()
                .any(|o| o.severity == Severity::Error)
        );

        // Report structurally valid; quality reflects the observed problems.
        assert_eq!(report.validate(), Ok(()));
        assert!(
            report.summary.quality_index < 1.0,
            "quality must drop when problems are observed"
        );
    }

    // ─── C4: conceptual regression over the full chain (PR 3) ──────
    //
    // Plan → TrajectoryAnalyzer → Observations → PlanAdvisor → Actions
    //
    // Criteria (user contract C4):
    // 1. EVERY action references an observation id that exists (I5 — explicit,
    //    not just rely on report.validate()).
    // 2. No observation is orphaned by an adaptation error: every observation
    //    whose phenomenon has a plan-level remediation rule produces at least
    //    one action. Phenomena outside the rule table (execution/semantic, or
    //    anything new via `#[non_exhaustive]`) legitimately produce none — the
    //    advisor never invents knowledge (C2).
    #[test]
    fn full_chain_actions_reference_observations_without_orphans() {
        use thalos_core::analysis::aggregator::{Aggregator, DefaultAggregator};
        use thalos_core::analysis::observation::{ArtifactRef, ObservationKind};
        use thalos_core::analysis::scoring::DefaultScoringPolicy;
        use thalos_core::ids::MotionPlanId;

        // Mix of problem waypoints (same setup as the legacy
        // `scenario_multiple_problems_aggregate_correctly` — fidelity anchor).
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),  // singular
            TrajectoryPoint::new(vec![0.5, 0.05], 0.5), // low manipulability
            TrajectoryPoint::new(vec![0.5, 1.57], 1.0), // good
        ]);
        let artifact = ArtifactRef::MotionPlan(MotionPlanId("mp-chain".to_string()));

        // 1. Plan → TrajectoryAnalyzer → Observations
        let analyzer = TrajectoryAnalyzer::new(&chain, None);
        let observations = analyzer.analyze(artifact.clone(), &traj);
        assert!(
            !observations.is_empty(),
            "problem trajectory must produce observations"
        );

        // 2. Observations → PlanAdvisor → Actions
        let actions = crate::advisor::PlanAdvisor.advise(&observations);
        assert!(
            !actions.is_empty(),
            "plan-level problems must produce actions"
        );

        // C3/I5 (explicit): every action targets an observation id that exists.
        let ids: Vec<_> = observations.iter().map(|o| o.id).collect();
        for action in &actions {
            assert!(
                ids.contains(&action.target_observation),
                "action {:?} must target an existing observation id",
                action.kind
            );
        }

        // C4: no orphan by adaptation error — every observation of a
        // remediated kind produces at least one action.
        let remediated: [ObservationKind; 6] = [
            ObservationKind::LowManipulability,
            ObservationKind::NearSingularity,
            ObservationKind::Singularity,
            ObservationKind::CollisionRisk,
            ObservationKind::CollisionNear,
            ObservationKind::ConstraintViolation,
        ];
        for obs in &observations {
            if remediated.contains(&obs.kind) {
                assert!(
                    actions.iter().any(|a| a.target_observation == obs.id),
                    "observation {:?} (kind {:?}) must produce at least one action — \
                     orphaned by adaptation error",
                    obs.id,
                    obs.kind
                );
            }
        }

        // The trajectory must exercise the singular phenomenon (fidelity).
        assert!(
            observations.iter().any(|o| matches!(
                o.kind,
                ObservationKind::Singularity | ObservationKind::NearSingularity
            )),
            "fully-extended arm must yield a singular observation"
        );

        // 3. Observations → DefaultAggregator → AnalysisReport (validate-safe).
        let report = DefaultAggregator::new(DefaultScoringPolicy).aggregate(artifact, observations);
        assert_eq!(report.validate(), Ok(()));
        assert!(report.summary.quality_index < 1.0);
    }

    // PR 7a: the remediation chain is fully observation-based — the advisor
    // produces actions over observations (I5) — see `advisor::tests`
    // `advise_produces_actions_over_observations` and the C4 chain test
    // `full_chain_actions_reference_observations_without_orphans` above.

    // ─── S1: AnalysisMetrics → BTreeMap<String, f64> projection (P3) ─────
    //
    // The runtime service populates `report.metrics` from the technical
    // analysis (`AnalysisMetrics`) so `/plan/analyze` stops shipping an empty
    // `{}`. The projection uses STABLE domain keys and omits absent
    // optionals — additive, never changes the semantics of existing fields.

    #[test]
    fn metrics_projection_carries_all_aggregates() {
        let metrics = AnalysisMetrics {
            waypoint_count: 5,
            trajectory_duration: 12.5,
            avg_manipulability: Some(0.42),
            min_manipulability: Some(0.21),
            near_singular_count: 3,
            singular_count: 1,
            min_collision_distance: Some(0.035),
            min_collision_waypoint: Some(2),
            has_collisions: true,
            first_collision_waypoint: Some(2),
        };

        let map = metrics.to_btree_map();
        assert_eq!(map["waypoint_count"], 5.0);
        assert_eq!(map["trajectory_duration"], 12.5);
        assert!((map["avg_manipulability"] - 0.42).abs() < 1e-12);
        assert!((map["min_manipulability"] - 0.21).abs() < 1e-12);
        assert_eq!(map["near_singular_count"], 3.0);
        assert_eq!(map["singular_count"], 1.0);
        assert!((map["min_collision_distance"] - 0.035).abs() < 1e-12);
        assert_eq!(map["min_collision_waypoint"], 2.0);
        assert_eq!(map["has_collisions"], 1.0);
        assert_eq!(
            map.len(),
            9,
            "all non-optional aggregates + present optionals"
        );
    }

    #[test]
    fn metrics_projection_omits_absent_optionals() {
        let metrics = AnalysisMetrics {
            waypoint_count: 2,
            trajectory_duration: 0.0,
            avg_manipulability: None,
            min_manipulability: None,
            near_singular_count: 0,
            singular_count: 0,
            min_collision_distance: None,
            min_collision_waypoint: None,
            has_collisions: false,
            first_collision_waypoint: None,
        };

        let map = metrics.to_btree_map();
        assert_eq!(map["waypoint_count"], 2.0);
        assert_eq!(map["has_collisions"], 0.0);
        for absent in [
            "avg_manipulability",
            "min_manipulability",
            "min_collision_distance",
            "min_collision_waypoint",
        ] {
            assert!(
                !map.contains_key(absent),
                "`{absent}` must be omitted when the value is None"
            );
        }
        assert_eq!(map.len(), 5, "scalar aggregates are always present");
    }
}
