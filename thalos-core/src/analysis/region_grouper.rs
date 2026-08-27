//! RegionGrouper — the SINGLE owner of contiguous problem-region grouping.
//!
//! # Why a dedicated owner (PR 7a decision, documented with evidence)
//!
//! The contiguous-region capability (group observations of the same
//! phenomenon with gap → `waypoint_range`) MUST live in exactly one place:
//! handlers, optimization and repair all consume `&[ProblemRegion]` but none
//! of them may own the grouping algorithm — that would scatter the debt.
//!
//! It does NOT live in
//! [`DefaultAggregator`](crate::analysis::aggregator::DefaultAggregator):
//! the analysis model is frozen (I7 single quality measure; the report
//! contract is observations/actions/metrics/summary — no region field), so the
//! aggregator cannot surface regions without expanding the model. It also
//! preserves the aggregator's single responsibility (obs → report, design D3).
//!
//! `RegionGrouper` is the explicit, pure, unit-testable owner: it consumes the
//! canonical [`Observation`] language (`Location::Waypoint` anchors) and
//! produces core [`ProblemRegion`]s. Every consumer — the API
//! `ProblemRegionsDtoAdapter` projection, the optimization pipeline, the repair
//! planners and the session preview — calls this component and never
//! re-implements grouping.
//!
//! The region phenomenon itself is part of the domain vocabulary:
//! [`Location::Region`](Location::Region) anchors an observation to a
//! [`RegionId`]; `ProblemRegion` is its formal representation. The grouper
//! assigns those ids in deterministic group order.

use crate::analysis::attribute_value::AttributeValue;
use crate::analysis::location::Location;
use crate::analysis::observation::{Observation, ObservationKind, Severity};
use crate::analysis::region::{
    ProblemRegion, RegionExplanation, RegionId, RegionKind, RegionMetrics, RegionSeverity,
};

/// Configuración de la agrupación de regiones.
#[derive(Debug, Clone)]
pub struct RegionGrouperConfig {
    /// Distancia máxima entre waypoints para considerarlos parte de la misma
    /// región. Default: 2 (un waypoint sano entre dos problemáticos no separa).
    pub gap_threshold: usize,
    /// Cantidad mínima de waypoints para formar una región. Default: 1.
    pub minimum_region_size: usize,
    /// Si es true, los hallazgos aislados (singletons) no forman región.
    /// Default: false.
    pub ignore_singletons: bool,
}

impl Default for RegionGrouperConfig {
    fn default() -> Self {
        Self {
            gap_threshold: 2,
            minimum_region_size: 1,
            ignore_singletons: false,
        }
    }
}

/// Único dueño de la detección de regiones contiguas de observaciones.
///
/// Agrupa observaciones del mismo fenómeno (`ObservationKind` → `RegionKind`)
/// ancladas a `Location::Waypoint` cuyo gap es ≤ `gap_threshold`, y produce
/// [`ProblemRegion`]s con rango, métricas y explicación. Es una función pura:
/// misma entrada → mismas regiones (determinista).
pub struct RegionGrouper {
    config: RegionGrouperConfig,
}

impl Default for RegionGrouper {
    fn default() -> Self {
        Self::new(RegionGrouperConfig::default())
    }
}

impl RegionGrouper {
    /// Crea un agrupador con la configuración dada.
    pub fn new(config: RegionGrouperConfig) -> Self {
        Self { config }
    }
    /// Agrupa observaciones de una trayectoria en regiones contiguas.
    pub fn group(&self, observations: &[Observation]) -> Vec<ProblemRegion> {
        let normalized = self.normalize(observations);
        self.detect_regions(&normalized)
    }

    // ─── Etapa 1: Normalize ─────────────────────────────────────────

    /// Ordena observaciones por waypoint y proyecta el fenómeno a `RegionKind`.
    /// Observaciones sin ancla de waypoint o sin región (kind sin mapeo) se
    /// descartan — una región es un rango de waypoints, no un fenómeno suelto.
    fn normalize(&self, observations: &[Observation]) -> Vec<NormalizedObservation> {
        let mut sorted: Vec<NormalizedObservation> = observations
            .iter()
            .filter_map(|o| {
                let waypoint = match o.location {
                    Location::Waypoint(wp) => wp,
                    _ => return None,
                };
                let kind = Self::region_kind(o.kind)?;
                Some(NormalizedObservation {
                    waypoint,
                    kind,
                    severity: Self::region_severity(o.severity),
                    value: Self::value_attribute(o),
                })
            })
            .collect();

        sorted.sort_by(|a, b| a.waypoint.cmp(&b.waypoint));
        sorted
    }

    // ─── Etapa 2: Detect Regions ────────────────────────────────────

    /// Agrupa observaciones normalizadas en regiones.
    /// Dos observaciones pertenecen a la misma región si:
    /// - Mismo `RegionKind`
    /// - Distancia entre waypoints ≤ `gap_threshold`
    fn detect_regions(&self, normalized: &[NormalizedObservation]) -> Vec<ProblemRegion> {
        if normalized.is_empty() {
            return vec![];
        }

        let mut regions: Vec<ProblemRegion> = Vec::new();
        let mut current_start = normalized[0].waypoint;
        let mut current_end = normalized[0].waypoint + 1;
        let mut current_kind = normalized[0].kind;
        let mut current: Vec<&NormalizedObservation> = Vec::new();
        current.push(&normalized[0]);

        for obs in &normalized[1..] {
            let same_kind = obs.kind == current_kind;
            let distance = obs.waypoint.saturating_sub(current_end.saturating_sub(1));

            if same_kind && distance <= self.config.gap_threshold {
                current_end = current_end.max(obs.waypoint + 1);
                current.push(obs);
            } else {
                if self.should_keep_region(&current) {
                    regions.push(self.build_region(
                        regions.len(),
                        current_kind,
                        current_start..current_end,
                        &current,
                    ));
                }
                current_start = obs.waypoint;
                current_end = obs.waypoint + 1;
                current_kind = obs.kind;
                current.clear();
                current.push(obs);
            }
        }

        if self.should_keep_region(&current) {
            regions.push(self.build_region(
                regions.len(),
                current_kind,
                current_start..current_end,
                &current,
            ));
        }

        regions
    }

    fn should_keep_region(&self, observations: &[&NormalizedObservation]) -> bool {
        if self.config.ignore_singletons && observations.len() < self.config.minimum_region_size {
            return false;
        }
        observations.len() >= self.config.minimum_region_size
    }

    fn build_region(
        &self,
        id: usize,
        kind: RegionKind,
        range: std::ops::Range<usize>,
        observations: &[&NormalizedObservation],
    ) -> ProblemRegion {
        let severity = Self::compute_severity(observations);

        let mut metrics = RegionMetrics {
            waypoint_count: range.len(),
            average_value: None,
            min_value: None,
            max_value: None,
            error_count: 0,
            warning_count: 0,
        };

        let mut sum = 0.0_f64;
        let mut value_count = 0_usize;

        for o in observations {
            match o.severity {
                RegionSeverity::Critical => metrics.error_count += 1,
                RegionSeverity::Warning => metrics.warning_count += 1,
                RegionSeverity::Info => {}
            }
            if let Some(v) = o.value {
                sum += v;
                value_count += 1;
                metrics.min_value = Some(metrics.min_value.map_or(v, |m| m.min(v)));
                metrics.max_value = Some(metrics.max_value.map_or(v, |m| m.max(v)));
            }
        }

        if value_count > 0 {
            metrics.average_value = Some(sum / value_count as f64);
        }

        let strategies = Self::strategies_for(kind);

        let explanation = RegionExplanation {
            cause: format!(
                "{} region detected at waypoints {}–{}",
                kind.name(),
                range.start,
                range.end.saturating_sub(1)
            ),
            consequence: format!(
                "{} observations, {} errors, {} warnings",
                metrics.waypoint_count, metrics.error_count, metrics.warning_count
            ),
            recommended_strategies: strategies,
            confidence: 1.0,
        };

        ProblemRegion {
            id: RegionId(id),
            kind,
            severity,
            waypoint_range: range,
            metrics: Some(metrics),
            boundary: None,
            explanation: Some(explanation),
            confidence: 1.0,
            evidence: vec![],
        }
    }

    /// Estrategias de remediación sugeridas por tipo de región (presentación
    /// del wire legacy — `recommended_strategies` del DTO).
    fn strategies_for(kind: RegionKind) -> Vec<String> {
        match kind {
            RegionKind::Collision => vec![
                "Lift TCP".into(),
                "Insert waypoint".into(),
                "Adjust approach angle".into(),
            ],
            RegionKind::Singularity => vec![
                "Switch IK solver".into(),
                "Lift TCP".into(),
                "Adjust path".into(),
            ],
            RegionKind::LowManipulability => vec![
                "Switch IK solver".into(),
                "Lift TCP".into(),
                "Insert waypoint".into(),
            ],
            RegionKind::Constraint => vec![
                "Adjust joint range".into(),
                "Insert intermediate waypoint".into(),
            ],
            RegionKind::Velocity => {
                vec!["Reduce speed".into(), "Adjust acceleration profile".into()]
            }
            RegionKind::Tracking => vec![
                "Increase sample rate".into(),
                "Adjust tracking parameters".into(),
            ],
        }
    }

    /// Mapeo `ObservationKind → RegionKind` (mismo fenómeno, representación
    /// de región). Los fenómenos sin región (semántica, ejecución suelta, …)
    /// retornan `None` — no forman regiones de waypoints.
    fn region_kind(kind: ObservationKind) -> Option<RegionKind> {
        match kind {
            ObservationKind::Singularity | ObservationKind::NearSingularity => {
                Some(RegionKind::Singularity)
            }
            ObservationKind::LowManipulability => Some(RegionKind::LowManipulability),
            ObservationKind::CollisionRisk | ObservationKind::CollisionNear => {
                Some(RegionKind::Collision)
            }
            ObservationKind::ConstraintViolation => Some(RegionKind::Constraint),
            ObservationKind::TrackingError
            | ObservationKind::TrackingSpike
            | ObservationKind::JointDeviation => Some(RegionKind::Tracking),
            ObservationKind::VelocityDeviation => Some(RegionKind::Velocity),
            _ => None,
        }
    }

    fn region_severity(severity: Severity) -> RegionSeverity {
        match severity {
            Severity::Error => RegionSeverity::Critical,
            Severity::Warning => RegionSeverity::Warning,
            Severity::Info => RegionSeverity::Info,
        }
    }

    fn value_attribute(observation: &Observation) -> Option<f64> {
        match observation.attributes.get("value") {
            Some(AttributeValue::Number(v)) => Some(*v),
            _ => None,
        }
    }

    fn compute_severity(observations: &[&NormalizedObservation]) -> RegionSeverity {
        let mut max = RegionSeverity::Info;
        for o in observations {
            if o.severity > max {
                max = o.severity;
            }
        }
        max
    }
}

// ─── Internal types ──────────────────────────────────────────────────

/// Observación normalizada: anclada a waypoint y proyectada a región.
#[derive(Debug, Clone)]
struct NormalizedObservation {
    waypoint: usize,
    kind: RegionKind,
    severity: RegionSeverity,
    value: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::{RegionGrouper, RegionGrouperConfig};
    use crate::analysis::attribute_value::AttributeValue;
    use crate::analysis::location::Location;
    use crate::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use crate::analysis::region::{RegionId, RegionKind, RegionSeverity};
    use crate::ids::MotionPlanId;
    use std::collections::BTreeMap;

    fn observation(
        id: u32,
        kind: ObservationKind,
        severity: Severity,
        waypoint: usize,
    ) -> Observation {
        Observation {
            id: ObservationId(id),
            kind,
            severity,
            artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
            location: Location::Waypoint(waypoint),
            attributes: BTreeMap::new(),
            causes: Vec::new(),
            related: Vec::new(),
        }
    }

    fn singular(id: u32, waypoint: usize, severity: Severity) -> Observation {
        observation(id, ObservationKind::Singularity, severity, waypoint)
    }

    fn with_value(mut o: Observation, value: f64) -> Observation {
        o.attributes
            .insert("value".to_string(), AttributeValue::Number(value));
        o
    }

    #[test]
    fn contiguous_same_kind_form_one_region() {
        // Spec/design: observaciones contiguas del mismo fenómeno → 1 región.
        let observations = vec![
            singular(1, 5, Severity::Error),
            singular(2, 6, Severity::Error),
            singular(3, 7, Severity::Warning),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].waypoint_range, 5..8);
        assert_eq!(regions[0].kind, RegionKind::Singularity);
    }

    #[test]
    fn gap_beyond_threshold_splits_regions() {
        // gap_threshold=2: waypoints 6→10 distance 4 > 2 → split.
        let observations = vec![
            singular(1, 5, Severity::Error),
            singular(2, 6, Severity::Error),
            singular(3, 10, Severity::Warning),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions.len(), 2);
        assert_eq!(regions[0].waypoint_range, 5..7);
        assert_eq!(regions[1].waypoint_range, 10..11);
    }

    #[test]
    fn different_kinds_never_merge() {
        // Singularity at 5 + LowManipulability at 6 → 2 regiones.
        let observations = vec![
            singular(1, 5, Severity::Error),
            observation(2, ObservationKind::LowManipulability, Severity::Warning, 6),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions.len(), 2);
        assert_eq!(regions[0].kind, RegionKind::Singularity);
        assert_eq!(regions[1].kind, RegionKind::LowManipulability);
    }

    #[test]
    fn gap_within_threshold_keeps_region() {
        // waypoints 5→7 distance 2 ≤ 2 → misma región (gap_threshold=2).
        let observations = vec![
            singular(1, 5, Severity::Error),
            singular(2, 7, Severity::Error),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].waypoint_range, 5..8);
    }

    #[test]
    fn grouping_is_deterministic() {
        let observations = vec![
            singular(3, 3, Severity::Warning),
            singular(1, 1, Severity::Error),
            observation(2, ObservationKind::LowManipulability, Severity::Info, 2),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions1 = grouper.group(&observations);
        let regions2 = grouper.group(&observations);
        assert_eq!(regions1.len(), regions2.len());
        assert_eq!(regions1[0].waypoint_range, regions2[0].waypoint_range);
        assert_eq!(regions1[1].waypoint_range, regions2[1].waypoint_range);
    }

    #[test]
    fn empty_input_produces_no_regions() {
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        assert!(grouper.group(&[]).is_empty());
    }

    #[test]
    fn ignore_singletons_drops_lone_observation() {
        let observations = vec![singular(1, 5, Severity::Error)];
        let mut config = RegionGrouperConfig::default();
        config.ignore_singletons = true;
        config.minimum_region_size = 2;
        let grouper = RegionGrouper::new(config);
        assert!(grouper.group(&observations).is_empty());
    }

    #[test]
    fn non_waypoint_observations_are_skipped() {
        // Observaciones sin ancla de waypoint (Timestamp/Region) no forman
        // regiones de waypoints.
        let mut obs = singular(1, 5, Severity::Error);
        obs.location = Location::Timestamp(42);
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        assert!(grouper.group(&[obs]).is_empty());
    }

    #[test]
    fn region_metrics_aggregate_severity_counts_and_values() {
        // Metrics del wire legacy: error/warning counts + avg/min/max value
        // derivados de la severidad y del atributo "value" de las observaciones.
        let observations = vec![
            with_value(singular(1, 5, Severity::Error), 1500.0),
            with_value(singular(2, 6, Severity::Warning), 500.0),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        let metrics = regions[0].metrics.as_ref().expect("metrics");
        assert_eq!(metrics.error_count, 1);
        assert_eq!(metrics.warning_count, 1);
        assert_eq!(metrics.average_value, Some(1000.0));
        assert_eq!(metrics.min_value, Some(500.0));
        assert_eq!(metrics.max_value, Some(1500.0));
        assert_eq!(metrics.waypoint_count, 2);
    }

    #[test]
    fn region_severity_is_max_of_observations() {
        let observations = vec![
            singular(1, 5, Severity::Warning),
            singular(2, 6, Severity::Error),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions[0].severity, RegionSeverity::Critical);
    }

    #[test]
    fn region_ids_are_sequential_in_group_order() {
        let observations = vec![
            singular(1, 5, Severity::Error),
            singular(2, 6, Severity::Error),
            observation(3, ObservationKind::CollisionRisk, Severity::Error, 10),
        ];
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions[0].id, RegionId(0));
        assert_eq!(regions[1].id, RegionId(1));
        assert_eq!(regions[1].kind, RegionKind::Collision);
    }

    // ─── Real-world fidelity (ported from the legacy RegionDetector) ───

    #[test]
    fn eighty_contiguous_singularities_form_one_region() {
        let observations: Vec<Observation> = (147..=226)
            .map(|wp| singular(wp as u32, wp, Severity::Error))
            .collect();
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions.len(), 1, "80 singularities → 1 region");
        assert_eq!(regions[0].waypoint_count(), 80);
        assert_eq!(regions[0].kind, RegionKind::Singularity);
        assert_eq!(regions[0].waypoint_range, 147..227);
    }

    #[test]
    fn mixed_trajectory_separates_by_kind() {
        let mut observations = Vec::new();
        for wp in 147..=226 {
            observations.push(singular(wp as u32, wp, Severity::Error));
        }
        for wp in 401..=405 {
            observations.push(observation(
                wp as u32,
                ObservationKind::CollisionRisk,
                Severity::Error,
                wp,
            ));
        }
        let grouper = RegionGrouper::new(RegionGrouperConfig::default());
        let regions = grouper.group(&observations);
        assert_eq!(regions.len(), 2, "1 singular + 1 collision");
        assert_eq!(regions[0].kind, RegionKind::Singularity);
        assert_eq!(regions[0].waypoint_count(), 80);
        assert_eq!(regions[1].kind, RegionKind::Collision);
        assert_eq!(regions[1].waypoint_count(), 5);
    }
}
