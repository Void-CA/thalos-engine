//! Knowledge base for the trajectory assessor (design "Rule base — 10–15
//! rules as data").
//!
//! The KB is data-declared: every rule is a [`Rule`] struct with a category,
//! a priority and typed antecedents/consequents. There is **no**
//! recommendation-producing consequent — the expert engine only derives
//! facts, marks evidence and maps rule activation onto the Mamdani risk
//! output variable. `PlanAdvisor` stays the sole recommendation producer.

use serde::{Deserialize, Serialize};

use crate::fuzzy::{FuzzySet, LinguisticVariable, MembershipShape};

/// Thresholds replicated from the trajectory analyzer (design "Threshold
/// anchoring contract"). These are documented copies of analyzer-local
/// literals in `thalos-planning/src/analysis/mod.rs` and are each pinned by a
/// behavioral anchoring test in `tests/golden.rs` — if the analyzer's local
/// constant ever changes, the anchoring test fails loudly.
///
/// The one threshold that exists as public config
/// (`near_singular_condition_threshold` on
/// `thalos_core::analysis::singularity::config::SingularityConfig`) is read
/// directly — see [`near_singular_threshold`] — never duplicated here.
pub const MANIPULABILITY_LOW_THRESHOLD: f64 = 0.3;
pub const SINGULAR_CONDITION_THRESHOLD: f64 = 1000.0;
pub const COLLISION_DISTANCE: f64 = 0.0;
pub const NEAR_COLLISION_DISTANCE: f64 = 0.05;

/// The shared near-singular threshold, read from `SingularityConfig` (design:
/// shared directly, no duplication).
pub fn near_singular_threshold() -> f64 {
    thalos_core::analysis::singularity::config::SingularityConfig::default()
        .near_singular_condition_threshold
}

/// Category of a rule — grounds the rule in one reasoning dimension.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RuleCategory {
    Collision,
    Singularity,
    Manipulability,
    Trajectory,
}

/// A linguistic (fuzzy) input variable the rules reason over.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LinguisticVar {
    Manipulability,
    SingularityProximity,
    CollisionClearance,
}

/// Scale contract for a crisp input entering the fuzzy KB (ADR-004 "Derived
/// Feature Scale Contract for the Intelligence KB"). Every derived feature
/// that crosses the derived-features → KB frontier MUST declare all five
/// elements — definition, unit, domain, normalization, semantic evidence —
/// plus the per-feature missing-data default. Contracts are `const` data:
/// they document and are test-visible, they do not gate at runtime (MVP).
#[derive(Debug, Clone, Copy)]
pub struct FeatureContract {
    /// The linguistic variable this contract describes.
    pub variable: LinguisticVar,
    /// One-sentence semantic description of the feature.
    pub definition: &'static str,
    /// Measurement unit, or "dimensionless".
    pub unit: &'static str,
    /// Numeric bounds of the fuzzy domain, `[min, max]`.
    pub domain: [f64; 2],
    /// How raw values map into the fuzzy domain.
    pub normalization: &'static str,
    /// Citation of the analyzer threshold or empirical basis anchoring the scale.
    pub semantic_evidence: &'static str,
    /// Value used when the metric is absent from the report.
    pub missing_default: f64,
    /// What the missing-data default represents semantically.
    pub missing_default_semantics: &'static str,
}

/// The three KB input variables, each with its declared scale contract (spec
/// "Three contracts declared for the three KB inputs"). The web-side
/// `evidence.ts` semantics table mirrors this declaration.
pub const FEATURE_CONTRACTS: &[FeatureContract] = &[
    FeatureContract {
        variable: LinguisticVar::Manipulability,
        definition: "Average (or minimum) manipulability over the trajectory",
        unit: "dimensionless",
        domain: [0.0, 1.0],
        normalization: "raw value; < MANIPULABILITY_LOW_THRESHOLD → `low` fuzzy set",
        semantic_evidence: "MANIPULABILITY_LOW_THRESHOLD (0.3) from the trajectory analyzer",
        missing_default: 0.0,
        missing_default_semantics: "worst case (no dexterity)",
    },
    FeatureContract {
        variable: LinguisticVar::SingularityProximity,
        definition: "Presence/severity of singularity events on the trajectory",
        unit: "dimensionless",
        domain: [0.0, 1.0],
        normalization: "discrete presence/severity: 0.0 absent, 0.15 near-singular only, 0.5 singular event — both paths (observations + fallback) share this scale; never events / waypoint_count",
        semantic_evidence: "SINGULAR_CONDITION_THRESHOLD (1000.0) and the shared near-singular threshold (100.0) from SingularityConfig; observations path maps Singularity → 0.5, NearSingularity → 0.15",
        missing_default: 0.0,
        missing_default_semantics: "no singularity event observed",
    },
    FeatureContract {
        variable: LinguisticVar::CollisionClearance,
        definition: "Minimum collision clearance along the trajectory",
        unit: "m",
        domain: [-1.0, 1.0],
        normalization: "raw value in meters; ≤ COLLISION_DISTANCE → `danger`, ≤ NEAR_COLLISION_DISTANCE → `near`, else `safe`",
        semantic_evidence: "COLLISION_DISTANCE (0.0) and NEAR_COLLISION_DISTANCE (0.05) from the trajectory analyzer",
        missing_default: 1.0,
        missing_default_semantics: "no obstacle seen (safe)",
    },
];

/// A rule antecedent. `MetricIs` matches a fuzzy membership degree; a
/// `FactEquals` antecedent matches a derived working-memory fact (an absent
/// fact counts as `false`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Antecedent {
    MetricIs {
        variable: LinguisticVar,
        set: &'static str,
    },
    FactEquals {
        fact: &'static str,
        value: bool,
    },
}

/// A fuzzy set of the output (risk) variable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RiskSet {
    Low,
    Medium,
    High,
    Critical,
}

/// A rule consequent. **No recommendation variant exists** — the engine never
/// emits semantic recommendations; `PlanAdvisor` is the sole producer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Consequent {
    /// Derive a working-memory fact consumed by later rules (forward chaining).
    DeriveFact { fact: &'static str, value: bool },
    /// Record a key-value entry in the assessment evidence.
    MarkEvidence { key: &'static str, value: f64 },
    /// Map the rule's activation onto the Mamdani risk output set.
    RiskIs { set: RiskSet },
}

/// A single data-declared rule.
#[derive(Debug, Clone, PartialEq)]
pub struct Rule {
    /// Stable id, e.g. `"R07_low_manipulability"`.
    pub id: &'static str,
    /// Reasoning category.
    pub category: RuleCategory,
    /// Agenda priority, 1 (low) – 10 (high).
    pub priority: u8,
    /// All antecedents must match for the rule to fire (AND = min).
    pub antecedents: Vec<Antecedent>,
    /// Consequents applied when the rule fires.
    pub consequents: Vec<Consequent>,
}

/// KB validation failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum KbError {
    #[error("rule base must hold between 10 and 15 rules, got {actual}")]
    CountOutOfRange { actual: usize },
    #[error("duplicate rule id {0}")]
    DuplicateId(&'static str),
    #[error("rule {0} has priority outside 1..=10")]
    PriorityOutOfRange(&'static str),
}

/// The default knowledge base (12 rules, frozen).
pub fn default_kb() -> Vec<Rule> {
    use Antecedent::{FactEquals, MetricIs};
    use Consequent::{DeriveFact, MarkEvidence, RiskIs};
    use LinguisticVar::*;
    use RiskSet::*;
    use RuleCategory as Category;

    vec![
        Rule {
            id: "R01_collision_danger",
            category: Category::Collision,
            priority: 10,
            antecedents: vec![MetricIs {
                variable: CollisionClearance,
                set: "danger",
            }],
            consequents: vec![RiskIs { set: Critical }],
        },
        Rule {
            id: "R02_collision_near",
            category: Category::Collision,
            priority: 8,
            antecedents: vec![MetricIs {
                variable: CollisionClearance,
                set: "near",
            }],
            consequents: vec![RiskIs { set: High }],
        },
        Rule {
            id: "R03_collision_danger_evidence",
            category: Category::Collision,
            priority: 6,
            antecedents: vec![MetricIs {
                variable: CollisionClearance,
                set: "danger",
            }],
            consequents: vec![
                DeriveFact {
                    fact: "danger_zone",
                    value: true,
                },
                MarkEvidence {
                    key: "collision_danger",
                    value: 1.0,
                },
            ],
        },
        Rule {
            id: "R04_singularity_medium",
            category: Category::Singularity,
            priority: 5,
            antecedents: vec![MetricIs {
                variable: SingularityProximity,
                set: "medium",
            }],
            consequents: vec![RiskIs { set: Medium }],
        },
        Rule {
            id: "R05_manipulability_medium",
            category: Category::Manipulability,
            priority: 5,
            antecedents: vec![MetricIs {
                variable: Manipulability,
                set: "medium",
            }],
            consequents: vec![RiskIs { set: Medium }],
        },
        Rule {
            id: "R07_low_manipulability",
            category: Category::Manipulability,
            priority: 3,
            antecedents: vec![MetricIs {
                variable: Manipulability,
                set: "low",
            }],
            consequents: vec![
                RiskIs { set: High },
                DeriveFact {
                    fact: "low_manipulability",
                    value: true,
                },
                MarkEvidence {
                    key: "manipulability_low",
                    value: 1.0,
                },
            ],
        },
        Rule {
            id: "R08_safe_clearance",
            category: Category::Collision,
            priority: 2,
            antecedents: vec![MetricIs {
                variable: CollisionClearance,
                set: "safe",
            }],
            consequents: vec![DeriveFact {
                fact: "safe_clearance",
                value: true,
            }],
        },
        Rule {
            id: "R09_near_singularity",
            category: Category::Singularity,
            priority: 3,
            antecedents: vec![MetricIs {
                variable: SingularityProximity,
                set: "high",
            }],
            consequents: vec![
                RiskIs { set: High },
                DeriveFact {
                    fact: "near_singularity",
                    value: true,
                },
                MarkEvidence {
                    key: "singularity_high",
                    value: 1.0,
                },
            ],
        },
        Rule {
            id: "R10_manipulability_high",
            category: Category::Manipulability,
            priority: 2,
            antecedents: vec![MetricIs {
                variable: Manipulability,
                set: "high",
            }],
            consequents: vec![DeriveFact {
                fact: "good_manipulability",
                value: true,
            }],
        },
        Rule {
            id: "R11_compromised_manipulability",
            category: Category::Manipulability,
            priority: 10,
            antecedents: vec![
                FactEquals {
                    fact: "low_manipulability",
                    value: true,
                },
                FactEquals {
                    fact: "near_singularity",
                    value: true,
                },
            ],
            consequents: vec![
                RiskIs { set: Critical },
                MarkEvidence {
                    key: "critical_combination",
                    value: 1.0,
                },
            ],
        },
        Rule {
            id: "R12_safe_plan",
            category: Category::Trajectory,
            priority: 1,
            antecedents: vec![
                FactEquals {
                    fact: "safe_clearance",
                    value: true,
                },
                MetricIs {
                    variable: SingularityProximity,
                    set: "low",
                },
                MetricIs {
                    variable: Manipulability,
                    set: "high",
                },
            ],
            consequents: vec![RiskIs { set: Low }],
        },
    ]
}

/// Validate the rule base: count within [10, 15], unique ids, priorities in
/// 1..=10. The absence of a recommendation-producing consequent is enforced
/// by the `Consequent` type itself (there is no `Recommend` variant).
pub fn validate(kb: &[Rule]) -> Result<(), KbError> {
    if !(10..=15).contains(&kb.len()) {
        return Err(KbError::CountOutOfRange { actual: kb.len() });
    }
    let mut seen = std::collections::HashSet::new();
    for rule in kb {
        if !seen.insert(rule.id) {
            return Err(KbError::DuplicateId(rule.id));
        }
        if !(1..=10).contains(&rule.priority) {
            return Err(KbError::PriorityOutOfRange(rule.id));
        }
    }
    Ok(())
}

/// The three linguistic input variables (design "Three linguistic variables"),
/// each anchored to analyzer thresholds (see module docs and `golden.rs`).
pub fn input_variables() -> Vec<LinguisticVariable> {
    vec![
        LinguisticVariable {
            name: "manipulability",
            sets: vec![
                FuzzySet {
                    name: "low",
                    // Monotonic "clearly low": 1.0 at manipulability 0, falling
                    // linearly to 0 at the analyzer threshold (0.3). A value
                    // just below the threshold (0.29) is only MARGINALLY low,
                    // so it belongs to `medium` — this stops marginal plans
                    // from over-escalating to Critical.
                    shape: MembershipShape::LeftShoulder {
                        plateau: 0.0,
                        zero: MANIPULABILITY_LOW_THRESHOLD,
                    },
                },
                FuzzySet {
                    name: "medium",
                    // Marginal zone: peaks at the analyzer threshold (0.3) and
                    // falls off by 0.5, so a value just below 0.3 reads as
                    // "medium" (recoverable) rather than "low" (clearly bad).
                    shape: MembershipShape::Triangular {
                        a: 0.15,
                        b: MANIPULABILITY_LOW_THRESHOLD,
                        c: 0.5,
                    },
                },
                FuzzySet {
                    name: "high",
                    // Monotonic "clearly good": rises from 0 at 0.5 to 1.0 at
                    // 0.7 and beyond. No support in the medium band (0.458 is
                    // NOT "high") and never decreases at the top — a clearly
                    // healthy 0.9 is fully "high", unlike the old bump that
                    // peaked at 0.6 and fell off.
                    shape: MembershipShape::RightShoulder {
                        start: 0.5,
                        plateau: 0.7,
                    },
                },
            ],
        },
        LinguisticVariable {
            name: "singularity_proximity",
            sets: vec![
                FuzzySet {
                    name: "low",
                    shape: MembershipShape::LeftShoulder {
                        plateau: 0.05,
                        zero: 0.1,
                    },
                },
                FuzzySet {
                    name: "medium",
                    shape: MembershipShape::Triangular {
                        a: 0.05,
                        b: 0.1,
                        c: 0.3,
                    },
                },
                FuzzySet {
                    name: "high",
                    shape: MembershipShape::Trapezoidal {
                        a: 0.2,
                        b: 0.3,
                        c: 1.0,
                        d: 1.0,
                    },
                },
            ],
        },
        LinguisticVariable {
            name: "collision_clearance",
            sets: vec![
                FuzzySet {
                    name: "danger",
                    shape: MembershipShape::LeftShoulder {
                        plateau: COLLISION_DISTANCE,
                        zero: 0.01,
                    },
                },
                FuzzySet {
                    name: "near",
                    shape: MembershipShape::Triangular {
                        a: 0.0,
                        b: 0.01,
                        c: NEAR_COLLISION_DISTANCE,
                    },
                },
                FuzzySet {
                    name: "safe",
                    shape: MembershipShape::RightShoulder {
                        start: NEAR_COLLISION_DISTANCE,
                        plateau: 0.1,
                    },
                },
            ],
        },
    ]
}

/// The risk output variable (Mamdani consequent variable over [0, 1]).
pub fn risk_variable() -> LinguisticVariable {
    LinguisticVariable {
        name: "risk",
        sets: vec![
            FuzzySet {
                name: "low",
                shape: MembershipShape::LeftShoulder {
                    plateau: 0.15,
                    zero: 0.4,
                },
            },
            FuzzySet {
                name: "medium",
                // Centroid ~0.375 — inside the Medium bucket [0.25, 0.5). A
                // lone medium signal must land Medium, not on the 0.5 boundary.
                shape: MembershipShape::Triangular {
                    a: 0.25,
                    b: 0.375,
                    c: 0.5,
                },
            },
            FuzzySet {
                name: "high",
                // Centroid ~0.625 — inside the High bucket [0.5, 0.75). A lone
                // high signal must land High, not on the 0.75 Critical edge.
                shape: MembershipShape::Triangular {
                    a: 0.5,
                    b: 0.625,
                    c: 0.75,
                },
            },
            FuzzySet {
                name: "critical",
                shape: MembershipShape::RightShoulder {
                    start: 0.75,
                    plateau: 1.0,
                },
            },
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_kb_has_between_10_and_15_rules() {
        let kb = default_kb();
        assert!(
            (10..=15).contains(&kb.len()),
            "rule count must be in [10, 15], got {}",
            kb.len()
        );
    }

    #[test]
    fn default_kb_ids_are_unique() {
        let kb = default_kb();
        let mut seen = std::collections::HashSet::new();
        for rule in &kb {
            assert!(
                seen.insert(rule.id),
                "duplicate rule id `{}` in default KB",
                rule.id
            );
        }
    }

    #[test]
    fn default_kb_covers_all_four_categories() {
        let kb = default_kb();
        let categories: std::collections::HashSet<RuleCategory> =
            kb.iter().map(|r| r.category).collect();
        for category in [
            RuleCategory::Collision,
            RuleCategory::Singularity,
            RuleCategory::Manipulability,
            RuleCategory::Trajectory,
        ] {
            assert!(
                categories.contains(&category),
                "default KB must contain a rule for {category:?}"
            );
        }
    }

    #[test]
    fn default_kb_has_no_recommend_consequent() {
        // The design forbids a recommendation-producing consequent: the expert
        // engine never emits semantic recommendations — `PlanAdvisor` is the
        // sole producer. The `Consequent` type has no `Recommend` variant; this
        // test pins that every fired consequent stays in the allowed set.
        let kb = default_kb();
        for rule in &kb {
            for consequent in &rule.consequents {
                assert!(
                    matches!(
                        consequent,
                        Consequent::DeriveFact { .. }
                            | Consequent::MarkEvidence { .. }
                            | Consequent::RiskIs { .. }
                    ),
                    "rule `{}` must not carry a recommendation consequent",
                    rule.id
                );
            }
        }
    }

    #[test]
    fn default_kb_priorities_are_in_1_to_10() {
        for rule in &default_kb() {
            assert!(
                (1..=10).contains(&rule.priority),
                "rule `{}` priority {} outside 1..=10",
                rule.id,
                rule.priority
            );
        }
    }

    #[test]
    fn validate_accepts_default_kb() {
        assert_eq!(validate(&default_kb()), Ok(()));
    }

    #[test]
    fn validate_rejects_duplicate_ids() {
        let mut kb = default_kb();
        kb.push(kb[0].clone());
        assert!(matches!(validate(&kb), Err(KbError::DuplicateId(_))));
    }

    #[test]
    fn validate_rejects_too_few_rules() {
        let kb = default_kb()[..5].to_vec();
        assert!(matches!(
            validate(&kb),
            Err(KbError::CountOutOfRange { actual: 5 })
        ));
    }

    #[test]
    fn validate_rejects_too_many_rules() {
        let mut kb = default_kb();
        for i in 0..5 {
            kb.push(Rule {
                id: Box::leak(format!("R99_extra_{i}").into_boxed_str()),
                category: RuleCategory::Trajectory,
                priority: 1,
                antecedents: vec![],
                consequents: vec![],
            });
        }
        assert!(matches!(
            validate(&kb),
            Err(KbError::CountOutOfRange { .. })
        ));
    }

    #[test]
    fn anchoring_constants_match_the_analyzer() {
        // Values replicated from thalos-planning/src/analysis/mod.rs
        // (manip_threshold=0.3, singular condition 1000.0, collision 0.0,
        // near-collision 0.05). Behavioral anchoring lives in golden.rs.
        assert_eq!(MANIPULABILITY_LOW_THRESHOLD, 0.3);
        assert_eq!(SINGULAR_CONDITION_THRESHOLD, 1000.0);
        assert_eq!(COLLISION_DISTANCE, 0.0);
        assert_eq!(NEAR_COLLISION_DISTANCE, 0.05);
    }

    #[test]
    fn near_singular_threshold_is_read_from_shared_config() {
        // Design: `near_singular_condition_threshold = 100.0` lives on the
        // public `SingularityConfig`; the intelligence crate reads it directly
        // (no duplication) and pins it against the analyzer's documented
        // literal 100.0.
        let shared = thalos_core::analysis::singularity::config::SingularityConfig::default()
            .near_singular_condition_threshold;
        assert_eq!(near_singular_threshold(), 100.0);
        assert_eq!(near_singular_threshold(), shared);
    }

    #[test]
    fn input_variables_are_exactly_three() {
        let variables = input_variables();
        assert_eq!(variables.len(), 3);
        let names: Vec<&str> = variables.iter().map(|v| v.name).collect();
        assert_eq!(
            names,
            vec![
                "manipulability",
                "singularity_proximity",
                "collision_clearance",
            ]
        );
    }

    #[test]
    fn risk_variable_has_the_four_risk_sets() {
        let risk = risk_variable();
        let names: Vec<&str> = risk.sets.iter().map(|s| s.name).collect();
        assert_eq!(names, vec!["low", "medium", "high", "critical"]);
    }

    #[test]
    fn manipulability_membership_is_monotonic_around_threshold() {
        // Behavioral anchor (recalibrated): the analyzer emits
        // `LowManipulability` when avg_manipulability < 0.3. The fuzzy layer
        // refines that crisp threshold into a gradient:
        //   - far below 0.3 (0.1) → clearly `low` (degree > 0.5);
        //   - just below 0.3 (0.29) → marginal → `medium` dominates (degree >
        //     0.5), NOT `low` — so marginal plans are Medium, not Critical.
        let variable = &input_variables()[0];

        let low_at_01 = variable
            .fuzzify(0.1)
            .into_iter()
            .find(|(name, _)| *name == "low")
            .expect("low set present");
        assert!(
            low_at_01.1 > 0.5,
            "low(0.1) must exceed 0.5 (clearly low), got {}",
            low_at_01.1
        );

        let medium_at_029 = variable
            .fuzzify(0.29)
            .into_iter()
            .find(|(name, _)| *name == "medium")
            .expect("medium set present");
        assert!(
            medium_at_029.1 > 0.5,
            "medium(0.29) must exceed 0.5 (marginal), got {}",
            medium_at_029.1
        );

        // `low` must be monotonically DECREASING: higher at 0.1 than at 0.29.
        let low_at_029 = variable
            .fuzzify(0.29)
            .into_iter()
            .find(|(name, _)| *name == "low")
            .expect("low set present");
        assert!(
            low_at_01.1 > low_at_029.1,
            "low must decrease as manipulability rises: low(0.1)={} > low(0.29)={}",
            low_at_01.1,
            low_at_029.1
        );

        // `high` must be monotonic and ABSENT in the medium band: an
        // intermediate 0.458 is NOT "high", and a clearly healthy 0.9 is fully
        // "high" (the old bump peaked at 0.6 and then decreased).
        let high_at_0458 = variable
            .fuzzify(0.458)
            .into_iter()
            .find(|(name, _)| *name == "high")
            .expect("high set present");
        assert!(
            high_at_0458.1 == 0.0,
            "high(0.458) must be 0 (medium band), got {}",
            high_at_0458.1
        );
        let high_at_09 = variable
            .fuzzify(0.9)
            .into_iter()
            .find(|(name, _)| *name == "high")
            .expect("high set present");
        assert!(
            (high_at_09.1 - 1.0).abs() < 1e-9,
            "high(0.9) must be 1.0 (clearly healthy), got {}",
            high_at_09.1
        );
    }

    #[test]
    fn collision_danger_degree_at_collision_distance_is_one() {
        // Behavioral anchor (task 2.1): the analyzer emits `CollisionRisk`
        // when min_collision_distance <= 0.0; the IA `danger` set at 0.0 is 1.0.
        let variable = &input_variables()[2];
        let danger = variable
            .fuzzify(COLLISION_DISTANCE)
            .into_iter()
            .find(|(name, _)| *name == "danger")
            .expect("danger set present");
        assert!(
            danger.1 > 0.5,
            "IA danger MF at 0.0 must exceed 0.5, got {}",
            danger.1
        );
    }

    #[test]
    fn singularity_high_degree_at_proximity_boundary_exceeds_half() {
        // Behavioral anchor (task 2.1): a plan whose waypoints are singular
        // (condition >= 1000) produces a proximity whose `high` degree exceeds
        // 0.5 — the analyzer's Singularity observation and the IA agree.
        let variable = &input_variables()[1];
        let high = variable
            .fuzzify(0.3)
            .into_iter()
            .find(|(name, _)| *name == "high")
            .expect("high set present");
        assert!(
            high.1 > 0.5,
            "IA singularity high MF at 0.3 must exceed 0.5, got {}",
            high.1
        );
    }

    #[test]
    fn feature_contracts_are_declared() {
        // Spec "Three contracts declared for the three KB inputs": exactly one
        // `FEATURE_CONTRACT` per fuzzy input variable, each complete with the
        // 5-element scale contract (definition, unit, domain, normalization,
        // semantic_evidence) plus the per-feature missing-data default.
        assert_eq!(
            FEATURE_CONTRACTS.len(),
            3,
            "one contract per KB input variable"
        );

        let variable_name = |v: LinguisticVar| match v {
            LinguisticVar::Manipulability => "manipulability",
            LinguisticVar::SingularityProximity => "singularity_proximity",
            LinguisticVar::CollisionClearance => "collision_clearance",
        };

        // Every KB input variable must have a declared contract, and every
        // contract must describe a real KB input (no orphans either way).
        let declared: std::collections::HashSet<&str> = FEATURE_CONTRACTS
            .iter()
            .map(|c| variable_name(c.variable))
            .collect();
        let kb_variables: std::collections::HashSet<&str> =
            input_variables().iter().map(|v| v.name).collect();
        assert_eq!(
            declared, kb_variables,
            "FEATURE_CONTRACTS must cover exactly the input_variables() set"
        );

        // Spec "Contract anchors to analyzer threshold": the manipulability
        // contract cites the analyzer threshold and describes the < 0.3 → low
        // normalization.
        let manip = FEATURE_CONTRACTS
            .iter()
            .find(|c| c.variable == LinguisticVar::Manipulability)
            .expect("manipulability contract present");
        assert!(
            manip
                .semantic_evidence
                .contains("MANIPULABILITY_LOW_THRESHOLD"),
            "manipulability contract must cite MANIPULABILITY_LOW_THRESHOLD"
        );
        assert!(
            manip.normalization.contains("low"),
            "manipulability normalization must describe the low fuzzy set"
        );

        // Every field of every contract must be filled in — an empty element
        // means the feature has no scale contract and must not enter diagnosis.
        for contract in FEATURE_CONTRACTS {
            assert!(
                !contract.definition.is_empty(),
                "{:?} missing definition",
                contract.variable
            );
            assert!(
                !contract.unit.is_empty(),
                "{:?} missing unit",
                contract.variable
            );
            assert!(
                contract.domain[0] <= contract.domain[1],
                "{:?} has an inverted domain",
                contract.variable
            );
            assert!(
                !contract.normalization.is_empty(),
                "{:?} missing normalization",
                contract.variable
            );
            assert!(
                !contract.semantic_evidence.is_empty(),
                "{:?} missing semantic evidence",
                contract.variable
            );
            assert!(
                !contract.missing_default_semantics.is_empty(),
                "{:?} missing missing-default semantics",
                contract.variable
            );
        }
    }
}
