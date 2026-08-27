//! Seed generation for multi-start IK solving.
//!
//! A `SeedGenerator` produces initial joint configurations (`Vec<Vec<f64>>`)
//! that explore different branches of the solution space. Each seed is passed
//! to an IK solver independently; the solver decides which solutions converge.
//!
//! Seeds are pure data — no IK solving, no validation, no quality assessment.

use crate::motion::segment::MotionSegment;
use crate::ids::OperationId;

/// A policy for generating alternative IK seeds.
///
/// Different policies explore different branches:
/// - `ElbowAlternate`: flips elbow joints to find elbow-up/down alternatives
/// - `Perturbation`: adds small random offsets to explore nearby basins
pub trait SeedPolicy: Send + Sync {
    /// Generate alternative seeds from a base configuration.
    ///
    /// `base_joints`: the current joint configuration (from previous segment)
    /// `target_joints`: the target joint angles of the MoveJ segment (for reference)
    ///
    /// Returns a list of seeds to try. The first seed should be the baseline
    /// (original configuration) for comparison.
    fn generate_seeds(&self, base_joints: &[f64], target_joints: &[f64]) -> Vec<Vec<f64>>;
}

/// Elbow alternate policy: flips the sign of elbow-related joints to explore
/// the opposite elbow configuration (elbow-up ↔ elbow-down).
///
/// For a 6-DOF robot with joints [shoulder_yaw, shoulder_pitch, elbow_pitch,
/// wrist_roll, wrist_pitch, wrist_yaw], this flips joints 1 and 2 (the pitch
/// joints that control the elbow posture).
pub struct ElbowAlternate {
    /// Which joint indices to flip (default: [1, 2] for 6-DOF)
    pub flip_joints: Vec<usize>,
    /// Small perturbation to add after flipping (radians)
    pub perturbation: f64,
}

impl Default for ElbowAlternate {
    fn default() -> Self {
        Self {
            flip_joints: vec![1, 2],
            perturbation: 0.05,
        }
    }
}

impl SeedPolicy for ElbowAlternate {
    fn generate_seeds(&self, base_joints: &[f64], _target_joints: &[f64]) -> Vec<Vec<f64>> {
        let mut seeds = Vec::new();

        // Seed 0: baseline (original configuration)
        seeds.push(base_joints.to_vec());

        // Seed 1: elbow flipped (negate the flip joints)
        let mut flipped = base_joints.to_vec();
        for &idx in &self.flip_joints {
            if idx < flipped.len() {
                flipped[idx] = -flipped[idx];
            }
        }
        seeds.push(flipped);

        // Seed 2: elbow flipped + small perturbation
        let mut perturbed = seeds[1].clone();
        for idx in 0..perturbed.len() {
            if !self.flip_joints.contains(&idx) {
                perturbed[idx] += self.perturbation;
            }
        }
        seeds.push(perturbed);

        seeds
    }
}

/// Configuration for seed generation — controls which policy is used
/// and its parameters.
#[derive(Debug, Clone)]
pub struct SeedConfig {
    /// Joint indices to flip for elbow alternation
    pub flip_joints: Vec<usize>,
    /// Perturbation magnitude (radians)
    pub perturbation: f64,
}

impl Default for SeedConfig {
    fn default() -> Self {
        Self {
            flip_joints: vec![1, 2],
            perturbation: 0.05,
        }
    }
}

impl SeedConfig {
    /// Create a SeedConfig for a specific robot's joint layout.
    pub fn for_robot(n_joints: usize) -> Self {
        // For robots with >= 4 joints, assume joints 1,2 are the elbow pitch joints
        // For 3-DOF or simpler, flip joint 1 only
        let flip_joints = if n_joints >= 4 {
            vec![1, 2]
        } else if n_joints >= 2 {
            vec![1]
        } else {
            vec![]
        };
        Self {
            flip_joints,
            perturbation: 0.05,
        }
    }
}
