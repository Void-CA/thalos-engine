//! Assessment output contract (design "Assessment output contract").
//!
//! `Assessment` is the single output of `Assessor::assess`: a categorical
//! risk verdict, a quality score (the normalized complement of the crisp risk
//! value), the triggered rules, the evidence, the PlanAdvisor-grounded
//! recommendation references and the full inference trace.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use thalos_core::analysis::action::ActionKind;

use crate::kb::RuleCategory;

/// Categorical risk verdict, derived from the Mamdani crisp value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Risk {
    Low,
    Medium,
    High,
    Critical,
}

impl Risk {
    /// Map a crisp risk value in [0, 1] onto the categorical verdict:
    /// [0, 0.25) → Low, [0.25, 0.5) → Medium, [0.5, 0.75) → High,
    /// [0.75, 1.0] → Critical.
    pub fn from_crisp(value: f64) -> Self {
        if value.is_nan() {
            return Risk::Low;
        }
        let clamped = value.clamp(0.0, 1.0);
        match clamped {
            v if v < 0.25 => Risk::Low,
            v if v < 0.5 => Risk::Medium,
            v if v < 0.75 => Risk::High,
            _ => Risk::Critical,
        }
    }
}

/// A fired rule, summarized for the output.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TriggeredRule {
    /// Rule id, e.g. `"R07_low_manipulability"`.
    pub id: String,
    /// Reasoning category.
    pub category: RuleCategory,
    /// Agenda priority.
    pub priority: u8,
}

/// A single trace entry — one fired rule in exact execution order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TraceEntry {
    /// Fired rule id.
    pub rule_id: String,
    /// Agenda priority of the fired rule.
    pub priority: u8,
    /// Antecedent → matched value (fuzzy degree or boolean fact).
    pub bindings: BTreeMap<String, String>,
    /// Derived facts produced by this firing.
    pub derived_output: BTreeMap<String, bool>,
}

/// A reference to an existing `PlanAdvisor` action kind, associated with the
/// diagnosis after inference (design: no parallel recommendation mechanism).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecommendationRef {
    /// The `ActionKind` the diagnosis associates with (matches report actions).
    pub action_kind: ActionKind,
    /// Problem region the recommendation addresses, when resolvable.
    pub region_id: Option<usize>,
    /// Human-readable rationale (English).
    pub rationale: String,
}

/// The complete intelligent assessment of an `AnalysisReport`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Assessment {
    /// Categorical risk verdict (primary output of fuzzy inference).
    pub risk: Risk,
    /// Quality score in [0, 1] — the normalized complement of the crisp risk
    /// value (no second fuzzy system for quality).
    pub quality: f64,
    /// Rules that fired during inference.
    pub triggered_rules: Vec<TriggeredRule>,
    /// Key-value evidence (derived inputs + MarkEvidence entries).
    pub evidence: BTreeMap<String, f64>,
    /// References to existing PlanAdvisor actions by kind.
    pub recommendations: Vec<RecommendationRef>,
    /// Full inference trace in firing order.
    pub trace: Vec<TraceEntry>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn risk_mapping_boundaries() {
        // Crisp [0, .25) = Low, [.25, .5) = Medium, [.5, .75) = High,
        // [.75, 1] = Critical.
        assert_eq!(Risk::from_crisp(0.0), Risk::Low);
        assert_eq!(Risk::from_crisp(0.24999), Risk::Low);
        assert_eq!(Risk::from_crisp(0.25), Risk::Medium);
        assert_eq!(Risk::from_crisp(0.49999), Risk::Medium);
        assert_eq!(Risk::from_crisp(0.5), Risk::High);
        assert_eq!(Risk::from_crisp(0.74999), Risk::High);
        assert_eq!(Risk::from_crisp(0.75), Risk::Critical);
        assert_eq!(Risk::from_crisp(1.0), Risk::Critical);
    }

    #[test]
    fn risk_mapping_clamps_out_of_range() {
        assert_eq!(Risk::from_crisp(-1.0), Risk::Low);
        assert_eq!(Risk::from_crisp(1.5), Risk::Critical);
        assert_eq!(Risk::from_crisp(f64::NAN), Risk::Low);
    }

    #[test]
    fn risk_serializes_snake_case() {
        assert_eq!(
            serde_json::to_string(&Risk::Critical).expect("serialize"),
            "\"critical\""
        );
        assert_eq!(
            serde_json::to_string(&Risk::Low).expect("serialize"),
            "\"low\""
        );
    }

    #[test]
    fn assessment_round_trips_preserving_trace_order() {
        let assessment = Assessment {
            risk: Risk::High,
            quality: 0.3,
            triggered_rules: vec![TriggeredRule {
                id: "R07_low_manipulability".into(),
                category: RuleCategory::Manipulability,
                priority: 3,
            }],
            evidence: BTreeMap::from([("manipulability".to_string(), 0.2)]),
            recommendations: vec![RecommendationRef {
                action_kind: ActionKind::Manipulability,
                region_id: Some(3),
                rationale: "Improve manipulability near the flagged region.".to_string(),
            }],
            trace: vec![
                TraceEntry {
                    rule_id: "R07_low_manipulability".into(),
                    priority: 3,
                    bindings: BTreeMap::from([(
                        "Manipulability IS low".to_string(),
                        "0.67".into(),
                    )]),
                    derived_output: BTreeMap::from([("danger_zone".to_string(), true)]),
                },
                TraceEntry {
                    rule_id: "R11_danger_zone".into(),
                    priority: 10,
                    bindings: BTreeMap::from([("danger_zone".to_string(), "true".into())]),
                    derived_output: BTreeMap::new(),
                },
            ],
        };

        let json = serde_json::to_string(&assessment).expect("serialize");
        let back: Assessment = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, assessment);
        let ids: Vec<&str> = back.trace.iter().map(|t| t.rule_id.as_str()).collect();
        assert_eq!(ids, vec!["R07_low_manipulability", "R11_danger_zone"]);
    }
}
