//! Forward-chaining inference engine (design "Forward chaining — working
//! memory with derived facts").
//!
//! The engine walks the rule base on a priority agenda. A rule fires when all
//! its antecedents match the current working memory (fuzzy membership degrees
//! plus derived facts); firing applies its consequents (deriving facts,
//! marking evidence, contributing to the Mamdani risk output) and records a
//! trace entry. The loop repeats until quiescence or a safety cap.

use std::collections::{BTreeMap, HashMap, HashSet};

use crate::kb::{Antecedent, Consequent, LinguisticVar, RiskSet, Rule};
use crate::output::TraceEntry;

/// Safety cap on inference passes (design: quiescence or max iterations = 50).
pub const MAX_ITERATIONS: usize = 50;

/// Fuzzy membership degrees of the linguistic input variables.
pub type Memberships = HashMap<(LinguisticVar, &'static str), f64>;

/// Result of a full inference run.
#[derive(Debug, Clone)]
pub struct EngineOutput {
    /// Fired rules in exact execution order (design: trace exposes firing order).
    pub trace: Vec<TraceEntry>,
    /// Final derived-fact working memory.
    pub facts: BTreeMap<&'static str, bool>,
    /// Evidence entries accumulated by `MarkEvidence` consequents.
    pub evidence: BTreeMap<String, f64>,
    /// Risk output contributions `(activation, set)` fed to Mamdani aggregation.
    pub risk_contributions: Vec<(f64, RiskSet)>,
}

/// Run forward chaining over `kb` until quiescence or `max_iterations` passes.
///
/// Rules are evaluated on a priority-descending agenda each pass; ties keep
/// declaration order. Each rule fires at most once.
pub fn run(kb: &[Rule], memberships: &Memberships, max_iterations: usize) -> EngineOutput {
    let mut facts: BTreeMap<&'static str, bool> = BTreeMap::new();
    let mut evidence: BTreeMap<String, f64> = BTreeMap::new();
    let mut trace: Vec<TraceEntry> = Vec::new();
    let mut risk_contributions: Vec<(f64, RiskSet)> = Vec::new();
    let mut fired: HashSet<&'static str> = HashSet::new();

    // Agenda: priority-descending, stable on ties (declaration order).
    let mut agenda: Vec<&Rule> = kb.iter().collect();
    agenda.sort_by(|a, b| b.priority.cmp(&a.priority));

    for _ in 0..max_iterations {
        let mut any_fired = false;
        for rule in &agenda {
            if fired.contains(rule.id) {
                continue;
            }
            let Some(activation) = evaluate_rule(rule, memberships, &facts) else {
                continue;
            };
            fired.insert(rule.id);
            any_fired = true;

            // Trace the fired rule: antecedent bindings + derived outputs.
            let mut bindings: BTreeMap<String, String> = BTreeMap::new();
            let mut derived_output: BTreeMap<String, bool> = BTreeMap::new();
            for antecedent in &rule.antecedents {
                match antecedent {
                    Antecedent::MetricIs { variable, set } => {
                        let degree = memberships.get(&(*variable, set)).copied().unwrap_or(0.0);
                        bindings.insert(format!("{:?} IS {set}", variable), format!("{degree:.3}"));
                    }
                    Antecedent::FactEquals { fact, value } => {
                        let matched = facts.get(fact).copied().unwrap_or(false);
                        bindings.insert(fact.to_string(), matched.to_string());
                        debug_assert_eq!(matched, *value, "fact antecedent must match");
                    }
                }
            }

            for consequent in &rule.consequents {
                match consequent {
                    Consequent::DeriveFact { fact, value } => {
                        facts.insert(fact, *value);
                        derived_output.insert(fact.to_string(), *value);
                    }
                    Consequent::MarkEvidence { key, value } => {
                        evidence.insert((*key).to_string(), *value);
                    }
                    Consequent::RiskIs { set } => {
                        risk_contributions.push((activation, *set));
                    }
                }
            }

            trace.push(TraceEntry {
                rule_id: rule.id.to_string(),
                priority: rule.priority,
                bindings,
                derived_output,
            });
        }
        if !any_fired {
            break;
        }
    }

    EngineOutput {
        trace,
        facts,
        evidence,
        risk_contributions,
    }
}

/// Evaluate a rule's antecedents against the current state.
///
/// Returns `Some(activation)` when every antecedent matches — activation is
/// the AND (= min) of the fuzzy degrees of `MetricIs` antecedents, with
/// `FactEquals` antecedents contributing 1.0. Returns `None` when any
/// antecedent fails.
fn evaluate_rule(
    rule: &Rule,
    memberships: &Memberships,
    facts: &BTreeMap<&'static str, bool>,
) -> Option<f64> {
    let mut activation = 1.0_f64;
    for antecedent in &rule.antecedents {
        match antecedent {
            Antecedent::MetricIs { variable, set } => {
                let degree = memberships.get(&(*variable, set)).copied().unwrap_or(0.0);
                if degree <= 0.0 {
                    return None;
                }
                activation = activation.min(degree);
            }
            Antecedent::FactEquals { fact, value } => {
                // An absent fact counts as `false`.
                if facts.get(fact).copied().unwrap_or(false) != *value {
                    return None;
                }
            }
        }
    }
    Some(activation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kb::RuleCategory;
    use crate::kb::default_kb;
    use Antecedent::{FactEquals, MetricIs};
    use Consequent::DeriveFact;
    use LinguisticVar::Manipulability as ManipVar;

    fn memberships_low(degree: f64) -> Memberships {
        let mut map = HashMap::new();
        map.insert((ManipVar, "low"), degree);
        map
    }

    #[test]
    fn two_rule_chain_fires_in_order_consuming_derived_fact() {
        // R1 derives `danger_zone`; R2 consumes it. Priority places R2 first,
        // but its antecedent is unsatisfied until R1 fires — so the trace is
        // [R1, R2] in exact firing order.
        let r1 = Rule {
            id: "R1_derive",
            category: RuleCategory::Manipulability,
            priority: 1,
            antecedents: vec![MetricIs {
                variable: ManipVar,
                set: "low",
            }],
            consequents: vec![DeriveFact {
                fact: "danger_zone",
                value: true,
            }],
        };
        let r2 = Rule {
            id: "R2_consume",
            category: RuleCategory::Manipulability,
            priority: 2,
            antecedents: vec![
                FactEquals {
                    fact: "danger_zone",
                    value: true,
                },
                MetricIs {
                    variable: ManipVar,
                    set: "low",
                },
            ],
            consequents: vec![DeriveFact {
                fact: "verdict",
                value: true,
            }],
        };
        let kb = vec![r1.clone(), r2.clone()];
        let output = run(&kb, &memberships_low(0.8), MAX_ITERATIONS);

        let ids: Vec<&str> = output.trace.iter().map(|t| t.rule_id.as_str()).collect();
        assert_eq!(ids, vec!["R1_derive", "R2_consume"]);
        assert_eq!(output.facts.get("danger_zone"), Some(&true));
        assert_eq!(output.facts.get("verdict"), Some(&true));
    }

    #[test]
    fn quiescence_terminates_after_no_new_fires() {
        let r1 = Rule {
            id: "R1",
            category: RuleCategory::Manipulability,
            priority: 1,
            antecedents: vec![MetricIs {
                variable: ManipVar,
                set: "low",
            }],
            consequents: vec![DeriveFact {
                fact: "f1",
                value: true,
            }],
        };
        let r2 = Rule {
            id: "R2",
            category: RuleCategory::Manipulability,
            priority: 2,
            antecedents: vec![FactEquals {
                fact: "f1",
                value: true,
            }],
            consequents: vec![DeriveFact {
                fact: "f2",
                value: true,
            }],
        };
        let output = run(&[r1, r2], &memberships_low(0.9), MAX_ITERATIONS);
        assert_eq!(output.trace.len(), 2);
        // A second run with no memberships must fire nothing.
        let empty = run(&default_kb(), &HashMap::new(), MAX_ITERATIONS);
        assert!(empty.trace.is_empty());
        assert!(empty.risk_contributions.is_empty());
    }

    #[test]
    fn rule_fires_at_most_once() {
        // A rule whose consequence would re-enable itself must not re-fire.
        let self_enabling = Rule {
            id: "R_self",
            category: RuleCategory::Manipulability,
            priority: 1,
            antecedents: vec![MetricIs {
                variable: ManipVar,
                set: "low",
            }],
            consequents: vec![DeriveFact {
                fact: "looped",
                value: true,
            }],
        };
        let output = run(&[self_enabling], &memberships_low(1.0), MAX_ITERATIONS);
        assert_eq!(output.trace.len(), 1);
        assert_eq!(output.facts.get("looped"), Some(&true));
    }

    #[test]
    fn iteration_cap_stops_long_chains() {
        // 60 chained rules require 60 passes — the 50-iteration cap must stop
        // the loop and yield exactly 50 fired rules (no infinite loop).
        let mut kb = Vec::new();
        for i in 0..60 {
            let fact = Box::leak(format!("f{i}").into_boxed_str());
            let mut antecedents = vec![MetricIs {
                variable: ManipVar,
                set: "low",
            }];
            if i > 0 {
                let prev = Box::leak(format!("f{}", i - 1).into_boxed_str());
                antecedents.push(FactEquals {
                    fact: prev,
                    value: true,
                });
            }
            kb.push(Rule {
                id: Box::leak(format!("R_chain_{i}").into_boxed_str()),
                // Strictly increasing priority so each pass fires exactly one
                // rule (rule i+1 is evaluated before rule i, so it can only
                // consume f_i in the next pass) — the cap is genuinely reached.
                category: RuleCategory::Manipulability,
                priority: (i + 1) as u8,
                antecedents,
                consequents: vec![DeriveFact { fact, value: true }],
            });
        }
        let output = run(&kb, &memberships_low(1.0), MAX_ITERATIONS);
        assert_eq!(output.trace.len(), MAX_ITERATIONS);
    }

    #[test]
    fn absent_derived_fact_counts_as_false() {
        // A FactEquals{value: false} antecedent matches while the fact is
        // absent — forward chaining treats missing facts as false.
        let rule = Rule {
            id: "R_false",
            category: RuleCategory::Manipulability,
            priority: 1,
            antecedents: vec![FactEquals {
                fact: "never_derived",
                value: false,
            }],
            consequents: vec![DeriveFact {
                fact: "fired_on_absence",
                value: true,
            }],
        };
        let output = run(&[rule], &HashMap::new(), MAX_ITERATIONS);
        assert_eq!(output.trace.len(), 1);
    }
}
