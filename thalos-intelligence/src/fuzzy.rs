//! Mamdani fuzzy inference primitives (design "Mamdani fuzzy — anchored to
//! analyzer constants").
//!
//! This module holds the pure fuzzy mathematics: membership shapes
//! (triangular, trapezoidal, left/right shoulders), fuzzification of a
//! linguistic variable, and centroid defuzzification. It is deliberately
//! domain-agnostic — the knowledge base (`kb`) maps domain metrics to these
//! shapes.

/// Number of samples used by centroid defuzzification over [0, 1]
/// (design: 100-sample discretization).
pub const DEFUZZ_SAMPLES: usize = 100;

use serde::{Deserialize, Serialize};

/// A membership shape. `LeftShoulder`/`RightShoulder` are the flat-plateau
/// variants used for threshold-anchored sets (e.g. "danger" clearance is 1.0
/// at and below the collision distance, "low" manipulability stays high up to
/// the plateau and then falls).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum MembershipShape {
    /// Triangle with vertices `(a, 0)`, `(b, 1)`, `(c, 0)`.
    Triangular { a: f64, b: f64, c: f64 },
    /// Trapezoid with vertices `(a, 0)`, `(b, 1)`, `(c, 1)`, `(d, 0)`.
    Trapezoidal { a: f64, b: f64, c: f64, d: f64 },
    /// Left shoulder: 1.0 for `x <= plateau`, falls linearly to 0 at `zero`.
    LeftShoulder { plateau: f64, zero: f64 },
    /// Right shoulder: 0 at `start`, rises linearly to 1.0 at `plateau`,
    /// stays 1.0 beyond.
    RightShoulder { start: f64, plateau: f64 },
}

impl MembershipShape {
    /// Membership degree of `x` in this shape, clamped to [0, 1].
    pub fn evaluate(&self, x: f64) -> f64 {
        match *self {
            MembershipShape::Triangular { a, b, c } => {
                if x <= a || x >= c {
                    0.0
                } else if x <= b {
                    (x - a) / (b - a)
                } else {
                    (c - x) / (c - b)
                }
            }
            MembershipShape::Trapezoidal { a, b, c, d } => {
                if x <= a {
                    0.0
                } else if x < b {
                    (x - a) / (b - a)
                } else if x <= c {
                    1.0
                } else if x < d {
                    (d - x) / (d - c)
                } else {
                    0.0
                }
            }
            MembershipShape::LeftShoulder { plateau, zero } => {
                if x <= plateau {
                    1.0
                } else if x >= zero {
                    0.0
                } else {
                    (zero - x) / (zero - plateau)
                }
            }
            MembershipShape::RightShoulder { start, plateau } => {
                if x <= start {
                    0.0
                } else if x >= plateau {
                    1.0
                } else {
                    (x - start) / (plateau - start)
                }
            }
        }
    }
}

/// A named fuzzy set inside a linguistic variable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FuzzySet {
    /// Stable set name, e.g. `"low"`, `"medium"`, `"high"`.
    pub name: &'static str,
    /// The membership shape of the set.
    pub shape: MembershipShape,
}

/// A linguistic variable: a name plus its fuzzy sets (2–4 sets each).
#[derive(Debug, Clone)]
pub struct LinguisticVariable {
    /// Stable variable name, e.g. `"manipulability"`.
    pub name: &'static str,
    /// The variable's fuzzy sets.
    pub sets: Vec<FuzzySet>,
}

impl LinguisticVariable {
    /// Fuzzify a crisp input `x`: returns `(set_name, degree)` for every set.
    pub fn fuzzify(&self, x: f64) -> Vec<(&'static str, f64)> {
        self.sets
            .iter()
            .map(|set| (set.name, set.shape.evaluate(x)))
            .collect()
    }
}

/// Centroid defuzzification of an aggregated output function over [0, 1].
///
/// `aggregated` is the max-aggregated output (design: aggregation = max);
/// the centroid is computed on `samples` equally-spaced points. An all-zero
/// aggregation (no rule fired) yields 0.0.
pub fn centroid(aggregated: impl Fn(f64) -> f64, samples: usize) -> f64 {
    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..samples {
        let x = (i as f64 + 0.5) / samples as f64;
        let y = aggregated(x);
        num += x * y;
        den += y;
    }
    if den <= 0.0 { 0.0 } else { num / den }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri(a: f64, b: f64, c: f64) -> MembershipShape {
        MembershipShape::Triangular { a, b, c }
    }

    fn trap(a: f64, b: f64, c: f64, d: f64) -> MembershipShape {
        MembershipShape::Trapezoidal { a, b, c, d }
    }

    #[test]
    fn triangular_peak_is_one() {
        // tri(0, 0.3, 0.6): the peak at 0.3 has degree 1.0.
        assert_eq!(tri(0.0, 0.3, 0.6).evaluate(0.3), 1.0);
    }

    #[test]
    fn triangular_feet_are_zero() {
        let shape = tri(0.0, 0.3, 0.6);
        assert_eq!(shape.evaluate(0.0), 0.0);
        assert_eq!(shape.evaluate(0.6), 0.0);
        assert_eq!(shape.evaluate(-0.1), 0.0);
        assert_eq!(shape.evaluate(0.7), 0.0);
    }

    #[test]
    fn triangular_left_slope_midpoint_is_half() {
        // Spec "Triangular Membership Evaluation": the left slope of a triangle
        // peaking at 0.3 evaluated at its midpoint (0.15) is 0.5.
        assert!((tri(0.0, 0.3, 0.6).evaluate(0.15) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn triangular_right_slope_is_symmetric() {
        let shape = tri(0.0, 0.3, 0.6);
        assert!((shape.evaluate(0.45) - 0.5).abs() < 1e-9);
        assert!((shape.evaluate(0.15) - shape.evaluate(0.45)).abs() < 1e-9);
    }

    #[test]
    fn trapezoid_plateau_is_one_and_ramps_are_linear() {
        let shape = trap(0.2, 0.3, 0.7, 0.9);
        assert_eq!(shape.evaluate(0.5), 1.0);
        assert_eq!(shape.evaluate(0.3), 1.0);
        assert_eq!(shape.evaluate(0.7), 1.0);
        // Left ramp midpoint (0.25) and right ramp midpoint (0.8).
        assert!((shape.evaluate(0.25) - 0.5).abs() < 1e-9);
        assert!((shape.evaluate(0.8) - 0.5).abs() < 1e-9);
        // Outside the feet the degree is zero.
        assert_eq!(shape.evaluate(0.1), 0.0);
        assert_eq!(shape.evaluate(1.0), 0.0);
    }

    #[test]
    fn left_shoulder_is_flat_until_plateau_then_falls() {
        let shape = MembershipShape::LeftShoulder {
            plateau: 0.05,
            zero: 0.1,
        };
        assert_eq!(shape.evaluate(0.0), 1.0);
        assert_eq!(shape.evaluate(0.05), 1.0);
        assert!((shape.evaluate(0.075) - 0.5).abs() < 1e-9);
        assert_eq!(shape.evaluate(0.1), 0.0);
        assert_eq!(shape.evaluate(0.5), 0.0);
    }

    #[test]
    fn right_shoulder_rises_from_start_to_plateau() {
        let shape = MembershipShape::RightShoulder {
            start: 0.05,
            plateau: 0.1,
        };
        assert_eq!(shape.evaluate(0.05), 0.0);
        assert!((shape.evaluate(0.075) - 0.5).abs() < 1e-9);
        assert_eq!(shape.evaluate(0.1), 1.0);
        assert_eq!(shape.evaluate(1.0), 1.0);
    }

    #[test]
    fn fuzzify_returns_every_set_degree() {
        let variable = LinguisticVariable {
            name: "manipulability",
            sets: vec![
                FuzzySet {
                    name: "low",
                    shape: tri(0.0, 0.3, 0.6),
                },
                FuzzySet {
                    name: "medium",
                    shape: tri(0.15, 0.3, 0.6),
                },
                FuzzySet {
                    name: "high",
                    shape: tri(0.3, 0.6, 1.0),
                },
            ],
        };
        let degrees = variable.fuzzify(0.15);
        assert_eq!(degrees.len(), 3);
        assert!((degrees[0].1 - 0.5).abs() < 1e-9, "low at 0.15 must be 0.5");
        assert!(
            (degrees[1].1 - 0.0).abs() < 1e-9,
            "medium at 0.15 must be 0"
        );
    }

    #[test]
    fn centroid_of_constant_unit_output_is_midpoint() {
        // A constant 1.0 aggregation over [0, 1] has its centroid at ~0.5.
        let c = centroid(|_| 1.0, DEFUZZ_SAMPLES);
        assert!((c - 0.5).abs() < 0.01, "centroid must be near 0.5, got {c}");
    }

    #[test]
    fn centroid_of_left_weighted_output_shifts_left() {
        // A left-shoulder aggregation pulls the centroid below 0.5.
        let shape = MembershipShape::LeftShoulder {
            plateau: 0.25,
            zero: 0.5,
        };
        let c = centroid(|x| shape.evaluate(x), DEFUZZ_SAMPLES);
        assert!(
            c > 0.15 && c < 0.5,
            "centroid of left-shoulder must sit in (0.15, 0.5), got {c}"
        );
    }

    #[test]
    fn centroid_of_right_weighted_output_shifts_right() {
        let shape = MembershipShape::RightShoulder {
            start: 0.75,
            plateau: 1.0,
        };
        let c = centroid(|x| shape.evaluate(x), DEFUZZ_SAMPLES);
        assert!(
            c > 0.85,
            "centroid of right-shoulder must sit above 0.85, got {c}"
        );
    }

    #[test]
    fn centroid_of_empty_output_is_zero() {
        // No rule fired → aggregation is all-zero → crisp risk 0.
        assert_eq!(centroid(|_| 0.0, DEFUZZ_SAMPLES), 0.0);
    }
}
