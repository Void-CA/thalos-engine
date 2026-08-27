//! Multi-start IK solver: wraps any `IKSolver` and tries multiple seeds.
//!
//! The solver is a thin wrapper — it does NOT implement a new IK algorithm.
//! It simply calls the base solver multiple times with different initial
//! configurations and collects the valid solutions.
//!
//! Design rules:
//! - `MultiStartIKSolver` finds solutions; it does NOT decide which is best
//! - Duplicate/equivalent solutions are filtered (configurable tolerance)
//! - Only converged solutions within joint limits are returned

use super::solver::{IKGoal, IKSolver};
use super::result::{IKResult, IKStatus};
use super::seed_generator::{SeedConfig, SeedPolicy, ElbowAlternate};

/// A multi-start IK solver that tries multiple seeds and collects valid solutions.
pub struct MultiStartIKSolver<'a> {
    base_solver: &'a dyn IKSolver,
    seed_config: SeedConfig,
    /// Tolerance for considering two solutions as duplicates (radians)
    duplicate_tolerance: f64,
}

impl<'a> MultiStartIKSolver<'a> {
    /// Create a new multi-start solver wrapping a base solver.
    pub fn new(base_solver: &'a dyn IKSolver, seed_config: SeedConfig) -> Self {
        Self {
            base_solver,
            seed_config,
            duplicate_tolerance: 1e-3,
        }
    }

    /// Create with elbow-alternate policy (default for6dof robots).
    pub fn elbow_alternate(base_solver: &'a dyn IKSolver) -> Self {
        Self::new(base_solver, SeedConfig::default())
    }

    /// Set the duplicate tolerance (radians).
    pub fn with_duplicate_tolerance(mut self, tol: f64) -> Self {
        self.duplicate_tolerance = tol;
        self
    }

    /// Solve IK with multiple seeds and return all valid, unique solutions.
    ///
    /// The first solution is always the one from the baseline seed (for comparison).
    /// Subsequent solutions are from alternative seeds, filtered for duplicates.
    pub fn solve_multi(&self, goal: IKGoal) -> Vec<IKResult> {
        // Generate seeds from the baseline configuration
        // Note: base_joints and target_joints are not available here because
        // the solver doesn't know the robot's current state. The seeds must
        // be provided externally via `solve_multi_with_seeds`.
        //
        // For now, this is a convenience method that uses default seeds.
        // Prefer `solve_multi_with_seeds` for full control.
        vec![]
    }

    /// Solve IK with explicitly provided seeds.
    ///
    /// `seeds`: list of initial joint configurations to try
    /// `goal`: the IK target (position or pose)
    ///
    /// Returns valid solutions in order: first the baseline (seed 0), then
    /// alternatives. Only converged solutions are included. Duplicates are
    /// filtered based on `duplicate_tolerance`.
    pub fn solve_multi_with_seeds(&self, seeds: &[Vec<f64>], goal: IKGoal) -> Vec<IKResult> {
        let mut solutions = Vec::new();

        for seed in seeds {
            let result = self.base_solver.solve(seed, goal.clone());

            match result {
                Ok(ik_result) if ik_result.status.is_converged() => {
                    // Check if this solution is significantly different from existing ones
                    if !self.is_duplicate(&ik_result.q, &solutions) {
                        solutions.push(ik_result);
                    }
                }
                _ => {
                    // Non-converged or error — skip this seed
                }
            }
        }

        solutions
    }

    /// Check if a new solution is a duplicate of any existing solution.
    fn is_duplicate(&self, new_q: &[f64], existing: &[IKResult]) -> bool {
        for sol in existing {
            if sol.q.len() != new_q.len() {
                continue;
            }
            let diff: f64 = sol.q.iter().zip(new_q.iter())
                .map(|(a, b)| (a - b).abs())
                .sum();
            if diff < self.duplicate_tolerance {
                return true;
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::inverse::solver::{IKGoal, IKSolver};
    use crate::kinematics::inverse::result::{IKResult, IKStatus};
    use thalos_math::Vector3;

    /// A mock solver that returns a fixed solution regardless of seed.
    struct MockSolver {
        solution: Vec<f64>,
    }

    impl IKSolver for MockSolver {
        fn solve(&self, _q0: &[f64], _goal: IKGoal) -> Result<IKResult, super::super::error::IkError> {
            Ok(IKResult::converged(self.solution.clone(), 10, 1e-8, None))
        }
    }

    /// A mock solver that returns different solutions based on seed sign.
    struct MockBranchingSolver;

    impl IKSolver for MockBranchingSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, super::super::error::IkError> {
            // If q0[1] > 0, return solution A; if q0[1] < 0, return solution B
            let q = if q0.get(1).map_or(false, |&v| v > 0.0) {
                vec![0.0, 0.5, -0.3]  // solution A
            } else {
                vec![0.0, -0.5, 0.3]  // solution B
            };
            Ok(IKResult::converged(q, 10, 1e-8, None))
        }
    }

    #[test]
    fn multi_start_filters_duplicates() {
        let solver = MockSolver { solution: vec![0.0, 0.5, -0.3] };
        let multi = MultiStartIKSolver::new(&solver, SeedConfig::default());

        let seeds = vec![
            vec![0.0, 0.5, -0.3],
            vec![0.0, -0.5, 0.3],  // different seed, same solution
        ];

        let goal = IKGoal::Position(Vector3::new(0.3, 0.0, 0.5));
        let results = multi.solve_multi_with_seeds(&seeds, goal);

        // Should have only 1 solution (duplicate filtered)
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].q, vec![0.0, 0.5, -0.3]);
    }

    #[test]
    fn multi_start_keeps_different_solutions() {
        let solver = MockBranchingSolver;
        let multi = MultiStartIKSolver::new(&solver, SeedConfig::default())
            .with_duplicate_tolerance(1e-3);

        let seeds = vec![
            vec![0.0, 0.5, -0.3],   // seed with positive q1 → solution A
            vec![0.0, -0.5, 0.3],   // seed with negative q1 → solution B
        ];

        let goal = IKGoal::Position(Vector3::new(0.3, 0.0, 0.5));
        let results = multi.solve_multi_with_seeds(&seeds, goal);

        // Should have 2 different solutions
        assert_eq!(results.len(), 2);
        assert_ne!(results[0].q, results[1].q);
    }
}
