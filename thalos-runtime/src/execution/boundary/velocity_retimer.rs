//! Pure per-joint velocity re-timer for the execution IR.
//!
//! `MoveLPlanner::plan` samples a CARTESIAN trapezoidal velocity profile and
//! does per-waypoint IK at a uniform cadence (`total_time / num_points`); it
//! never bounds per-JOINT velocity. A wrist's joint velocity that emerges from
//! IK can exceed the firmware channel ceiling, and the manifest builder rejects
//! the plan with `VELOCITY_EXCEEDED` (real-hardware repro; rejection is correct
//! — this is a planning bug, not a validator bug).
//!
//! The precise physical fix is re-timing: LOWERING the whole move would slow
//! every joint; re-timing ONLY the violating stretches of time bounds the
//! offending joint(s) while leaving adjacent valid gaps untouched. The spatial
//! joint trajectory is preserved exactly — only `dt` is stretched.

use thalos_engine::core::execution::plan::{ExecutionPlan, ExecutionWaypoint};

use super::safety_envelope::SafetyEnvelope;

/// Pure per-joint velocity re-timer. Deterministic: no RNG, no clock.
pub struct VelocityRetimer;

impl VelocityRetimer {
    /// Re-time `plan` so that every consecutive waypoint gap satisfies
    /// [`SafetyEnvelope::check_gap_velocity`].
    ///
    /// - Re-timing changes ONLY timestamps; joint values are untouched
    ///   (identical spatial path).
    /// - Only gaps that GENUINELY exceed a channel ceiling are stretched, by
    ///   exactly the minimum `dt` that bounds every joint (`min_gap_dt_us`).
    ///   Already-valid gaps (`≤ ceiling·(1+tolerance)`) keep their original
    ///   `dt`.
    /// - `dt_us == 0` gaps are PROTOCOL SEMANTICS (firmware-authoritative
    ///   velocity-bounding, ADR-3): they are preserved as-is and NEVER
    ///   synthesized a fake finite velocity.
    /// - Timestamps remain monotonic non-decreasing.
    pub fn retime(plan: &ExecutionPlan) -> ExecutionPlan {
        let waypoints = &plan.waypoints;
        if waypoints.is_empty() {
            return plan.clone();
        }

        // Stretch offending gaps only. Un-stretched waypoints keep their
        // ORIGINAL absolute timestamps bit-exact; each waypoint after a
        // stretched gap shifts forward by the accumulated stretch, so adjacent
        // valid gaps keep their original `dt`.
        let mut total_stretch_us: u64 = 0;
        let mut new_waypoints: Vec<ExecutionWaypoint> = Vec::with_capacity(waypoints.len());
        new_waypoints.push(ExecutionWaypoint {
            joints: waypoints[0].joints.clone(),
            timestamp: waypoints[0].timestamp,
        });

        for pair in waypoints.windows(2) {
            let prev = &pair[0];
            let next = &pair[1];
            let orig_dt_us = ((next.timestamp - prev.timestamp) * 1_000_000.0)
                .round()
                .max(0.0) as u64;

            // dt_us == 0 → physical velocity is UNDEFINED (Δt=0). This is
            // PROTOCOL SEMANTICS (ADR-3): preserve the zero-dt gap verbatim,
            // NEVER synthesize a fake finite velocity onto it.
            let stretch_us = if orig_dt_us == 0 {
                0
            } else {
                let delta_q: Vec<f64> = next
                    .joints
                    .iter()
                    .zip(&prev.joints)
                    .map(|(a, b)| a - b)
                    .collect();

                if SafetyEnvelope::check_gap_velocity(&delta_q, orig_dt_us as u32).is_ok() {
                    // Already at or under the ceiling — leave untouched.
                    0
                } else {
                    // Stretch to the minimum dt that satisfies EVERY ceiling.
                    let min = SafetyEnvelope::min_gap_dt_us(&delta_q) as u64;
                    min.max(1).saturating_sub(orig_dt_us)
                }
            };

            total_stretch_us += stretch_us;
            let shift = total_stretch_us as f64 / 1_000_000.0;
            new_waypoints.push(ExecutionWaypoint {
                joints: next.joints.clone(),
                timestamp: next.timestamp + shift,
            });
        }

        // Duration grows by exactly the stretched time (µs-exact increments);
        // when nothing stretches it is preserved bit-exact. The manifest
        // builder's TIMING_INVALID rule compares `sum(dt_us)` against this —
        // both advance by the same `total_stretch_us`, keeping a previously
        // valid plan within tolerance.
        let duration = plan.duration + total_stretch_us as f64 / 1_000_000.0;

        ExecutionPlan {
            waypoints: new_waypoints,
            segments: plan.segments.clone(),
            duration,
            repeat_count: plan.repeat_count,
            program_id: plan.program_id.clone(),
            program_revision: plan.program_revision,
            source_fingerprint: plan.source_fingerprint.clone(),
            robot_id: plan.robot_id.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::execution_boundary::manifest_builder::ExecutionManifestBuilder;
    use thalos_engine::core::execution::plan::{
        ExecutionSegment, PlanInstruction,
    };

    fn wp(joints: Vec<f64>, timestamp: f64) -> ExecutionWaypoint {
        ExecutionWaypoint { joints, timestamp }
    }

    fn one_segment_plan(waypoints: Vec<ExecutionWaypoint>) -> ExecutionPlan {
        let n = waypoints.len();
        let duration = waypoints
            .last()
            .map(|w| w.timestamp - waypoints[0].timestamp)
            .unwrap_or(0.0);
        ExecutionPlan {
            waypoints,
            segments: vec![ExecutionSegment {
                index: 0,
                planned_segment_index: 0,
                instruction: PlanInstruction::MoveJ,
                waypoint_range: 0..n,
            }],
            duration,
            repeat_count: 1,
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
            robot_id: None,
        }
    }

    fn dt_us(plan: &ExecutionPlan, idx: usize) -> u64 {
        ((plan.waypoints[idx].timestamp - plan.waypoints[idx - 1].timestamp) * 1_000_000.0)
            .round() as u64
    }

    /// The real-hardware violating gap: `axis_2` (wrist, ceiling 2.0 rad/s)
    /// Δq2 = 0.02038 rad in dt = 9877 µs → implied 2.0635 > 2.0. Re-timing must
    /// stretch dt so the implied velocity is at/below the ceiling, monotonicity
    /// holds, and joints are untouched.
    #[test]
    fn stretches_single_violating_wrist_gap() {
        let prev = vec![-0.44666246, 1.27985031, 0.13721055, 0.00119493];
        let next = vec![-0.44668399, 1.27990366, 0.15759187, 0.00137174];
        let plan = one_segment_plan(vec![wp(prev.clone(), 0.0), wp(next.clone(), 0.009877)]);

        // Sanity: the fixture genuinely violates before re-timing.
        let dq: Vec<f64> = next.iter().zip(&prev).map(|(a, b)| a - b).collect();
        assert!(SafetyEnvelope::check_gap_velocity(&dq, 9877).is_err());

        let retimed = VelocityRetimer::retime(&plan);

        assert_eq!(retimed.waypoints.len(), 2, "no waypoint added/removed");
        // Joints preserved exactly (spatial path identical).
        assert_eq!(retimed.waypoints[0].joints, prev);
        assert_eq!(retimed.waypoints[1].joints, next);
        // dt stretched above the original 9877 µs.
        let new_dt = dt_us(&retimed, 1);
        assert!(new_dt >= 9877, "dt must not shrink: {new_dt}");
        assert!(new_dt > 9877, "violating gap must be stretched: {new_dt}");
        // The stretched gap now passes the velocity check.
        let dq2: Vec<f64> = next.iter().zip(&prev).map(|(a, b)| a - b).collect();
        SafetyEnvelope::check_gap_velocity(&dq2, new_dt as u32)
            .unwrap_or_else(|v| panic!("stretched gap still violates: {v}"));
        // Monotonic non-decreasing.
        assert!(retimed.waypoints[1].timestamp >= retimed.waypoints[0].timestamp);
    }

    /// In a multi-gap trajectory only the offending stretch is slowed; an
    /// adjacent already-valid gap keeps its original dt.
    #[test]
    fn only_offending_gap_slowed_adjacent_valid_gap_unchanged() {
        let g0 = vec![-0.44666246, 1.27985031, 0.13721055, 0.00119493];
        let g1 = vec![-0.44668399, 1.27990366, 0.15759187, 0.00137174]; // slow wrist (violates)
        let g2 = vec![-0.44670000, 1.27990000, 0.15800000, 0.00140000]; // tiny, valid
        // Gap 0→1: Δq2 big over 9877 µs → violates. Gap 1→2: tiny joints valid.
        let valid_dt = 5000u64;
        let plan = one_segment_plan(vec![
            wp(g0.clone(), 0.0),
            wp(g1.clone(), 0.009877),
            wp(g2.clone(), 0.009877 + valid_dt as f64 / 1_000_000.0),
        ]);

        let retimed = VelocityRetimer::retime(&plan);

        // Offending gap stretched; valid adjacent gap keeps its original dt.
        assert!(dt_us(&retimed, 1) > 9877);
        assert_eq!(dt_us(&retimed, 2), valid_dt);
        // Joints untouched everywhere.
        for (i, wp) in retimed.waypoints.iter().enumerate() {
            assert_eq!(wp.joints, [g0.clone(), g1.clone(), g2.clone()][i]);
        }
        // Monotonic.
        assert!(retimed.waypoints[2].timestamp >= retimed.waypoints[1].timestamp);
    }

    /// Regression (real-hardware repro): the captured violating waypoints, once
    /// re-timed, produce a manifest whose EVERY gap passes `check_gap_velocity`
    /// and which the manifest builder's rule 8 accepts.
    #[test]
    fn retimed_plan_passes_manifest_rule_8_regression() {
        let prev = vec![-0.44666246, 1.27985031, 0.13721055, 0.00119493];
        let next = vec![-0.44668399, 1.27990366, 0.15759187, 0.00137174];
        // A believable surrounding trajectory so the manifest is well-formed.
        let start = vec![-0.44660000, 1.27980000, 0.13000000, 0.00100000];
        let end = vec![-0.44670000, 1.27991000, 0.16000000, 0.00140000];
        let plan = one_segment_plan(vec![
            wp(start, 0.0),
            wp(prev.clone(), 0.010000),
            wp(next.clone(), 0.019877),
            wp(end, 0.030000),
        ]);

        let retimed = VelocityRetimer::retime(&plan);
        let manifest = ExecutionManifestBuilder::build(&retimed).expect("retimed plan must build");

        // Every dt > 0 gap passes the velocity check; rule 8 accepts.
        for (i, pair) in manifest.samples.windows(2).enumerate() {
            let dt = pair[1].dt_us;
            if dt == 0 {
                continue;
            }
            let dq: Vec<f64> = pair[1]
                .joints
                .iter()
                .zip(&pair[0].joints)
                .map(|(a, b)| a - b)
                .collect();
            SafetyEnvelope::check_gap_velocity(&dq, dt)
                .unwrap_or_else(|v| panic!("retimed gap {i} still exceeds ceiling: {v}"));
        }
    }

    /// dt_us == 0 gaps are PROTOCOL SEMANTICS: the re-timer MUST NOT synthesize
    /// a finite velocity onto them. A zero-dt gap with a moving joint is
    /// preserved as zero-dt (dangerously-fast, but firmware velocity-bounds
    /// physical advancement — ADR-3), not silently stretched into fake finite
    /// time.
    #[test]
    fn preserves_zero_dt_gap_without_synthesizing_time() {
        let a = vec![0.0, 1.0, 0.0, 0.01];
        let b = vec![0.5, 1.5, 0.5, 0.02]; // large jump, but same timestamp → dt 0
        let plan = one_segment_plan(vec![wp(a.clone(), 0.0), wp(b.clone(), 0.0)]);

        let retimed = VelocityRetimer::retime(&plan);

        assert_eq!(dt_us(&retimed, 1), 0, "zero-dt gap must NOT be stretched");
        assert_eq!(retimed.waypoints[1].joints, b, "joints preserved");
        // Still passes check_gap_velocity (dt==0 → undefined, not rejected).
        let dq: Vec<f64> = b.iter().zip(&a).map(|(x, y)| x - y).collect();
        assert!(SafetyEnvelope::check_gap_velocity(&dq, 0).is_ok());
    }

    /// Monotonic non-decreasing timestamps hold even with repeated timestamps
    /// in the input (zero-dt subsequences).
    #[test]
    fn retimed_timestamps_remain_monotonic() {
        let plan = one_segment_plan(vec![
            wp(vec![0.0, 0.0, 0.0, 0.01], 0.0),
            wp(vec![0.5, 0.5, 0.5, 0.02], 1.0), // dt 1s, valid
            wp(vec![0.9, 0.9, 0.9, 0.03], 1.0), // dt 0 (same timestamp)
            wp(vec![1.0, 1.0, 1.0, 0.04], 2.0),
        ]);
        let retimed = VelocityRetimer::retime(&plan);
        for w in retimed.waypoints.windows(2) {
            assert!(w[1].timestamp >= w[0].timestamp);
        }
    }
}