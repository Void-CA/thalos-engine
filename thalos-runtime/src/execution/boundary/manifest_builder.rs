//! Pure builder: `ExecutionPlan` → [`ExecutionManifest`].
//!
//! Second hop of the pure chain
//! `CompiledPlan → ExecutionPlanBuilder → ExecutionPlan → ExecutionManifestBuilder → ExecutionManifest`.
//! Performs no I/O. Absolute timestamps (seconds) become delta `dt_us`
//! (microseconds, first sample `0`); consecutive bit-exact duplicate waypoints
//! are collapsed; segments map 1:1 from `ExecutionSegment` provenance
//! (`planned_segment_index`), never by re-inferring structure. Validation
//! mirrors `firmware/esp32/src/validator.cpp` and runs inside [`build`].

use thalos_engine::core::execution::plan::{BuilderError, ExecutionPlan, PlanInstruction};

use crate::execution_boundary::safety_envelope::SafetyEnvelope;
use crate::execution_boundary::{
    ExecutionManifest, ManifestInstruction, ManifestMetadata, ManifestSegment, TimedWaypoint,
};

/// Pure builder: `ExecutionPlan` → [`ExecutionManifest`].
///
/// Performs no I/O. The returned manifest is validated against the firmware
/// validator rules before being returned.
pub struct ExecutionManifestBuilder;

impl ExecutionManifestBuilder {
    pub fn build(plan: &ExecutionPlan) -> Result<ExecutionManifest, BuilderError> {
        // Single forward dedup scan. Consecutive waypoints are collapsed ONLY
        // when timestamp AND position are bit-exact equal (the earlier sample's
        // position is kept). `keep_map[i]` records the post-dedup sample index
        // of original waypoint `i`; segment ranges are derived from it later.
        let mut samples: Vec<TimedWaypoint> = Vec::new();
        let mut keep_map: Vec<usize> = Vec::with_capacity(plan.waypoints.len());
        for (i, wp) in plan.waypoints.iter().enumerate() {
            let dt_us = if i == 0 {
                0
            } else {
                let prev = &plan.waypoints[i - 1];
                if wp.timestamp == prev.timestamp {
                    if wp.joints == prev.joints {
                        // Bit-exact duplicate → collapse; keep the earlier sample.
                        keep_map.push(samples.len() - 1);
                        continue;
                    }
                    // Same timestamp, different position → never a silent collapse.
                    return Err(BuilderError::DedupConflict {
                        index: i,
                        t: wp.timestamp,
                    });
                }
                ((wp.timestamp - prev.timestamp) * 1_000_000.0).round() as u32
            };
            keep_map.push(samples.len());
            samples.push(TimedWaypoint {
                joints: wp.joints.clone(),
                dt_us,
            });
        }

        // Segment ranges against POST-dedup samples: `keep_map` translates each
        // segment's raw `waypoint_range` (indices into the ORIGINAL waypoints)
        // into post-dedup sample indices. `partition_end` keeps consecutive
        // segments non-overlapping when a segment-boundary waypoint collapsed
        // into the previous segment's last sample.
        let mut segments = Vec::with_capacity(plan.segments.len());
        let mut partition_end = 0usize;
        for seg in &plan.segments {
            let instruction = match seg.instruction {
                PlanInstruction::MoveJ => ManifestInstruction::MoveJ,
                PlanInstruction::MoveL => ManifestInstruction::MoveL,
            };
            let first = seg.waypoint_range.start.min(keep_map.len());
            let sample_start = partition_end.max(*keep_map.get(first).unwrap_or(&0));
            let last = seg.waypoint_range.end.saturating_sub(1).min(keep_map.len());
            let sample_end = sample_start.max(keep_map.get(last).map_or(sample_start, |&v| v + 1));
            partition_end = sample_end;
            segments.push(ManifestSegment {
                // Identity from provenance — never re-infer segment structure.
                index: seg.planned_segment_index,
                instruction,
                sample_start,
                sample_count: sample_end - sample_start,
            });
        }

        let metadata = ManifestMetadata {
            dof_count: plan.waypoints.first().map(|w| w.joints.len()).unwrap_or(0),
            total_samples: samples.len(),
            duration_us: (plan.duration * 1_000_000.0).round() as u64,
            repeat_count: plan.repeat_count,
        };

        let manifest = ExecutionManifest {
            metadata,
            segments,
            samples,
        };

        // Validation runs inside build(): it cannot be skipped by the caller.
        Self::validate(&manifest)?;
        Ok(manifest)
    }

    /// Validates a manifest against the firmware rules in
    /// `firmware/esp32/src/validator.cpp` order. Error strings match the
    /// firmware codes: `EMPTY_MANIFEST`, `DOF_MISMATCH`, `WAYPOINT_COUNT`,
    /// `SEGMENT_ORDER`, `SEGMENT_COVERAGE`, `TIMING_INVALID`.
    pub fn validate(manifest: &ExecutionManifest) -> Result<(), BuilderError> {
        // 1. EMPTY_MANIFEST — samples non-empty.
        if manifest.samples.is_empty() {
            return Err(BuilderError::Validation("EMPTY_MANIFEST".to_string()));
        }

        // 2. DOF_MISMATCH — all samples match `metadata.dof_count`.
        if manifest
            .samples
            .iter()
            .any(|s| s.joints.len() != manifest.metadata.dof_count)
        {
            return Err(BuilderError::Validation("DOF_MISMATCH".to_string()));
        }

        // 3. WAYPOINT_COUNT — `samples.len() == metadata.total_samples`.
        if manifest.samples.len() != manifest.metadata.total_samples {
            return Err(BuilderError::Validation("WAYPOINT_COUNT".to_string()));
        }

        // 4. SEGMENT_ORDER — strictly ascending `index`.
        if manifest
            .segments
            .windows(2)
            .any(|w| w[1].index <= w[0].index)
        {
            return Err(BuilderError::Validation("SEGMENT_ORDER".to_string()));
        }

        // 5. SEGMENT_COVERAGE — segments partition `[0, total_samples)` with
        //    no gaps and no overlap.
        let mut pos = 0usize;
        for seg in &manifest.segments {
            if seg.sample_start != pos {
                return Err(BuilderError::Validation("SEGMENT_COVERAGE".to_string()));
            }
            pos += seg.sample_count;
        }
        if pos != manifest.metadata.total_samples {
            return Err(BuilderError::Validation("SEGMENT_COVERAGE".to_string()));
        }

        // 6. TIMING_INVALID — `|sum(dt_us) - duration_us| <= max(duration_us/100, 1000)`.
        let accumulated: u64 = manifest.samples.iter().map(|s| s.dt_us as u64).sum();
        let declared = manifest.metadata.duration_us;
        let tolerance = (declared / 100).max(1000);
        if accumulated.abs_diff(declared) > tolerance {
            return Err(BuilderError::Validation("TIMING_INVALID".to_string()));
        }

        // 7. INVALID_JOINT — every waypoint joint inside the firmware
        //    SafetyEnvelope position limits (ADR-5 parity with the firmware
        //    validator's `check_physical_envelope`; spec test 11). The Rust
        //    mirror `SafetyEnvelope` holds the SAME values as
        //    `firmware/esp32/src/servo_config.h` — the backend rejects at the
        //    exact limits the firmware enforces.
        for sample in &manifest.samples {
            SafetyEnvelope::check_joints(&sample.joints)
                .map_err(|_| BuilderError::Validation("INVALID_JOINT".to_string()))?;
        }

        // 8. VELOCITY_EXCEEDED — implied velocity Δq/Δt ≤ the channel ceiling
        //    for dt > 0 gaps (ADR-5 / spec `backend_dt_us_zero_velocity_bounded`).
        //    dt_us == 0 makes physical velocity UNDEFINED (Δt = 0): the gap is
        //    skipped and the FIRMWARE executor velocity-bounds advancement
        //    (ADR-3 — dt_us==0 is PROTOCOL SEMANTICS, firmware-authoritative
        //    for velocity-bounding; the backend does NOT infer host velocity).
        //    Production plans are pre-emptively re-timed by `VelocityRetimer`
        //    (planned ExecutionPlans always pass here); this remains the hard
        //    rejection backstop for hand-built / un-re-timed manifests.
        for (i, pair) in manifest.samples.windows(2).enumerate() {
            let dt = pair[1].dt_us;
            if dt == 0 {
                continue;
            }
            let delta_q: Vec<f64> = pair[1]
                .joints
                .iter()
                .zip(&pair[0].joints)
                .map(|(a, b)| a - b)
                .collect();
            if let Err(v) = SafetyEnvelope::check_gap_velocity(&delta_q, dt) {
                let _ = i;
                let _ = v;
                return Err(BuilderError::Validation("VELOCITY_EXCEEDED".to_string()));
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::ops::Range;

    use thalos_engine::core::execution::plan::{ExecutionSegment, ExecutionWaypoint, PlanInstruction};

    use crate::execution_boundary::safety_envelope::SafetyEnvelope;

    use super::*;

    fn wp(joints: Vec<f64>, timestamp: f64) -> ExecutionWaypoint {
        ExecutionWaypoint { joints, timestamp }
    }

    fn seg(
        index: usize,
        planned_segment_index: usize,
        instruction: PlanInstruction,
        waypoint_range: Range<usize>,
    ) -> ExecutionSegment {
        ExecutionSegment {
            index,
            planned_segment_index,
            instruction,
            waypoint_range,
        }
    }

    fn plan(
        waypoints: Vec<ExecutionWaypoint>,
        segments: Vec<ExecutionSegment>,
        duration: f64,
    ) -> ExecutionPlan {
        ExecutionPlan {
            waypoints,
            segments,
            duration,
            repeat_count: 1,
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
            robot_id: None,
        }
    }

    /// A valid builder-produced manifest: 3 samples, MoveJ then MoveL.
    /// The validator rule tests corrupt ONE field of this real output.
    fn two_segment_manifest() -> ExecutionManifest {
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.5, 0.5], 0.5),
                wp(vec![1.0, 1.0], 1.0),
            ],
            vec![
                seg(0, 0, PlanInstruction::MoveJ, 0..2),
                seg(1, 1, PlanInstruction::MoveL, 2..3),
            ],
            1.0,
        );
        ExecutionManifestBuilder::build(&p).expect("valid fixture")
    }

    /// Absolute seconds MUST become delta microseconds: sample 0 has `dt_us = 0`,
    /// sample 1 `500_000` and sample 2 `500_000` for t = 0.0, 0.5, 1.0.
    #[test]
    fn absolute_to_delta_conversion() {
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.5, 0.5], 0.5),
                wp(vec![1.0, 1.0], 1.0),
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..3)],
            1.0,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("build should succeed");

        let dt: Vec<u32> = manifest.samples.iter().map(|s| s.dt_us).collect();
        assert_eq!(dt, vec![0, 500_000, 500_000]);
        assert_eq!(manifest.metadata.total_samples, 3);
    }

    /// `metadata.duration_us` MUST be within 1% of `plan.duration * 1e6`
    /// (here: within 1% of 2_000_000 for a 2.0 s plan). Joints are kept
    /// INSIDE the firmware SafetyEnvelope (M3 physical checks): the base
    /// ceiling is +1.5708 rad, so the final waypoint uses 1.4 rad instead of
    /// the pre-M3 2.0 rad — the TIMING rule under test is unchanged.
    #[test]
    fn manifest_duration_matches_plan() {
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.5, 0.5], 0.5),
                wp(vec![1.0, 1.0], 1.0),
                wp(vec![1.4, 1.4], 2.0),
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..4)],
            2.0,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("build should succeed");

        let declared = manifest.metadata.duration_us;
        let drift = declared.abs_diff(2_000_000);
        assert!(drift <= 2_000_000 / 100, "drift {drift} > 1% of 2_000_000");
        assert_eq!(declared, 2_000_000);
    }

    /// Two consecutive waypoints with bit-exact identical timestamp AND joints
    /// MUST collapse into one sample; the collapsed sample keeps the position.
    #[test]
    fn duplicate_timestamp_and_position_is_collapsed() {
        let p = plan(
            vec![
                wp(vec![0.1, 0.2], 0.0),
                wp(vec![0.1, 0.2], 0.0), // bit-exact duplicate
                wp(vec![0.3, 0.4], 1.0),
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..3)],
            1.0,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("build should succeed");

        assert_eq!(manifest.samples.len(), 2, "one fewer sample than the plan");
        assert_eq!(manifest.metadata.total_samples, 2);
        assert_eq!(manifest.samples[0].joints, vec![0.1, 0.2]);
        assert_eq!(manifest.samples[0].dt_us, 0);
        // Collapse keeps the ORIGINAL position; the delta to the next sample is
        // measured against the previous (identical) timestamp.
        assert_eq!(manifest.samples[1].joints, vec![0.3, 0.4]);
        assert_eq!(manifest.samples[1].dt_us, 1_000_000);
    }

    /// Each `ManifestSegment` MUST map 1:1 from the `ExecutionSegment` with the
    /// matching provenance (`planned_segment_index`), and `sample_start`/
    /// `sample_count` MUST be computed against POST-dedup samples — dedup
    /// shifts sample indices, so raw `waypoint_range` must NOT be used.
    #[test]
    fn manifest_segment_uses_planned_segment_provenance() {
        // 5 waypoints; w1 is a bit-exact duplicate of w0 → 4 post-dedup samples.
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.0, 0.0], 0.0), // duplicate → collapsed
                wp(vec![0.5, 0.5], 0.5),
                wp(vec![1.0, 1.0], 1.0),
                wp(vec![1.5, 1.5], 1.5),
            ],
            vec![
                seg(0, 0, PlanInstruction::MoveJ, 0..2),
                seg(1, 1, PlanInstruction::MoveL, 2..4),
                seg(2, 2, PlanInstruction::MoveJ, 4..5),
            ],
            1.5,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("build should succeed");

        // Provenance maps 1:1: three plan segments → three manifest segments.
        assert_eq!(manifest.segments.len(), 3);
        let provenance: Vec<usize> = manifest.segments.iter().map(|s| s.index).collect();
        assert_eq!(provenance, vec![0, 1, 2]);
        // Instruction mapping from provenance order.
        assert_eq!(manifest.segments[0].instruction, ManifestInstruction::MoveJ);
        assert_eq!(manifest.segments[1].instruction, ManifestInstruction::MoveL);
        assert_eq!(manifest.segments[2].instruction, ManifestInstruction::MoveJ);

        // Post-dedup sample ranges: seg0's raw waypoint_range was 2 waypoints,
        // but only 1 unique sample survives → count 1, not 2.
        assert_eq!(manifest.segments[0].sample_start, 0);
        assert_eq!(manifest.segments[0].sample_count, 1);
        assert_eq!(manifest.segments[1].sample_start, 1);
        assert_eq!(manifest.segments[1].sample_count, 2);
        assert_eq!(manifest.segments[2].sample_start, 3);
        assert_eq!(manifest.segments[2].sample_count, 1);
        assert_eq!(manifest.metadata.total_samples, 4);
    }

    /// Two consecutive waypoints with identical timestamp but DIFFERENT joints
    /// MUST produce an error, never a silent collapse; no manifest is returned.
    #[test]
    fn duplicate_timestamp_with_different_position_is_error() {
        let p = plan(
            vec![
                wp(vec![0.1, 0.2], 0.0),
                wp(vec![0.9, 0.9], 0.0), // same timestamp, different joints
                wp(vec![0.3, 0.4], 1.0),
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..3)],
            1.0,
        );

        let err = ExecutionManifestBuilder::build(&p).expect_err("build must fail");
        assert!(
            matches!(
                err,
                BuilderError::DedupConflict { index: 1, t } if (t - 0.0).abs() < f64::EPSILON
            ),
            "expected DedupConflict at index 1, got {err:?}"
        );
    }

    // ── Validator (firmware-parity) ────────────────────────────────────────

    fn assert_validation_code(manifest: &ExecutionManifest, expected: &str) {
        let err = ExecutionManifestBuilder::validate(manifest).expect_err("must fail");
        match err {
            BuilderError::Validation(code) => {
                assert_eq!(code, expected, "validation code mismatch")
            }
            other => panic!("expected Validation({expected}), got {other:?}"),
        }
    }

    /// Rule 1 — a plan with zero waypoints MUST be rejected (EMPTY_MANIFEST).
    #[test]
    fn empty_manifest_rejected() {
        let p = plan(vec![], vec![], 0.0);
        let err = ExecutionManifestBuilder::build(&p).expect_err("build must fail");
        assert!(matches!(
            err,
            BuilderError::Validation(code) if code == "EMPTY_MANIFEST"
        ));
    }

    /// Rule 2 — samples with different joint counts MUST be rejected (DOF_MISMATCH).
    #[test]
    fn dof_mismatch_rejected() {
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.5, 0.5, 0.5], 0.5), // 3 joints vs 2
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..2)],
            0.5,
        );
        let err = ExecutionManifestBuilder::build(&p).expect_err("build must fail");
        assert!(matches!(
            err,
            BuilderError::Validation(code) if code == "DOF_MISMATCH"
        ));
    }

    /// Rule 3 — `samples.len() != metadata.total_samples` MUST be rejected (WAYPOINT_COUNT).
    #[test]
    fn waypoint_count_mismatch_rejected() {
        let mut m = two_segment_manifest();
        m.metadata.total_samples = 5; // lies about the 3 samples
        assert_validation_code(&m, "WAYPOINT_COUNT");
    }

    /// Rule 4 — non-ascending segment indices MUST be rejected (SEGMENT_ORDER).
    #[test]
    fn segment_order_mismatch_rejected() {
        let mut m = two_segment_manifest();
        m.segments[1].index = 0; // duplicate index — not strictly ascending
        assert_validation_code(&m, "SEGMENT_ORDER");
    }

    /// Rule 5a — a gap in sample coverage MUST be rejected (SEGMENT_COVERAGE).
    #[test]
    fn segment_coverage_gap_rejected() {
        let mut m = two_segment_manifest();
        m.segments[1].sample_start = 3; // sample 1..2 left uncovered
        assert_validation_code(&m, "SEGMENT_COVERAGE");
    }

    /// Rule 5b — overlapping segment ranges MUST be rejected (SEGMENT_COVERAGE).
    #[test]
    fn segment_coverage_overlap_rejected() {
        let mut m = two_segment_manifest();
        m.segments[1].sample_start = 1; // sample 1 claimed by both segments
        assert_validation_code(&m, "SEGMENT_COVERAGE");
    }

    /// Rule 6a — accumulated `dt_us` may drift from `duration_us` by up to 1%
    /// (here 0.5% on a 2.0 s plan) and MUST still validate.
    #[test]
    fn timing_tolerance_within_one_percent() {
        // Declared duration 2.0 s → 2_000_000 µs; accumulated sum 1_990_000 µs
        // (0.5% drift) is within the 1% tolerance of 20_000 µs.
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.5, 0.5], 0.995),
                wp(vec![1.0, 1.0], 1.99),
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..3)],
            2.0,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("0.5% drift must pass");
        let accumulated: u64 = manifest.samples.iter().map(|s| s.dt_us as u64).sum();
        assert_eq!(accumulated, 1_990_000);
        assert_eq!(manifest.metadata.duration_us, 2_000_000);
        // The drift is exactly 0.5% of duration_us — within the 1% tolerance.
        assert_eq!(
            manifest.metadata.duration_us.abs_diff(accumulated),
            2_000_000 / 200
        );
    }

    /// Rule 6b — drift beyond 1% (here 2.5%) MUST be rejected (TIMING_INVALID).
    #[test]
    fn timing_outside_tolerance_rejected() {
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.5, 0.5], 0.975),
                wp(vec![1.0, 1.0], 1.95),
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..3)],
            2.0,
        );

        let err = ExecutionManifestBuilder::build(&p).expect_err("2.5% drift must fail");
        assert!(matches!(
            err,
            BuilderError::Validation(code) if code == "TIMING_INVALID"
        ));
    }

    /// Rule 6c — the timing tolerance has a 1000 µs floor: on a 0.05 s plan the
    /// 1% allowance is 500 µs, so a 600 µs drift passes ONLY via the floor.
    /// Joints are small (0.02 rad per 0.0247 s gap ≈ 0.81 rad/s) so the M3
    /// velocity check (rule 8) does not shadow the TIMING rule under test.
    #[test]
    fn timing_min_tolerance_floor_applied() {
        let p = plan(
            vec![
                wp(vec![0.0, 0.0], 0.0),
                wp(vec![0.02, 0.02], 0.0247),
                wp(vec![0.04, 0.04], 0.0494),
            ],
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..3)],
            0.05,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("floor tolerance must pass");
        let accumulated: u64 = manifest.samples.iter().map(|s| s.dt_us as u64).sum();
        let drift = manifest.metadata.duration_us.abs_diff(accumulated);
        assert_eq!(drift, 600);
        assert_eq!(
            manifest.metadata.duration_us / 100,
            500,
            "1% allowance < floor"
        );
    }

    // ── M3: physical-envelope checks (ADR-5 parity with the firmware validator) ──

    /// 4-DOF (RRPR icebot) waypoint — every joint inside its channel envelope.
    fn icebot_wp(joints: [f64; 4], timestamp: f64) -> ExecutionWaypoint {
        ExecutionWaypoint {
            joints: joints.to_vec(),
            timestamp,
        }
    }

    fn icebot_plan(waypoints: Vec<ExecutionWaypoint>, duration: f64) -> ExecutionPlan {
        let n = waypoints.len();
        plan(
            waypoints,
            vec![seg(0, 0, PlanInstruction::MoveJ, 0..n)],
            duration,
        )
    }

    /// Rule 7 (spec scenario `backend_manifest_out_of_envelope_rejected`,
    /// firmware test 11): a manifest whose waypoint joints are outside the
    /// firmware SafetyEnvelope (base at 2.5 rad > +1.5708) MUST be rejected
    /// with the firmware diagnostic code INVALID_JOINT — never clamped.
    #[test]
    fn physical_envelope_violation_rejected() {
        let p = icebot_plan(
            vec![
                icebot_wp([0.0, 0.0, 0.0, 0.01], 0.0),
                icebot_wp([2.5, 0.5, 0.5, 0.02], 0.5), // base 2.5 rad — out of ±1.5708
                icebot_wp([0.0, 0.0, 0.0, 0.01], 1.0),
            ],
            1.0,
        );

        let err = ExecutionManifestBuilder::build(&p).expect_err("build must fail");
        assert!(
            matches!(err, BuilderError::Validation(ref code) if code == "INVALID_JOINT"),
            "expected INVALID_JOINT, got {err:?}"
        );
    }

    /// Rule 7 — the elbow envelope is asymmetric (0..2.0944): a negative elbow
    /// joint is out-of-envelope even when |q| is small.
    #[test]
    fn physical_envelope_rejects_negative_elbow() {
        let p = icebot_plan(
            vec![
                icebot_wp([0.0, 0.0, 0.0, 0.01], 0.0),
                icebot_wp([0.5, -3.0, 0.0, 0.02], 0.5), // elbow −3.0 < −2.0944
                icebot_wp([0.0, 0.0, 0.0, 0.01], 1.0),
            ],
            1.0,
        );

        let err = ExecutionManifestBuilder::build(&p).expect_err("build must fail");
        assert!(
            matches!(err, BuilderError::Validation(ref code) if code == "INVALID_JOINT"),
            "expected INVALID_JOINT, got {err:?}"
        );
    }

    /// Rule 8 — implied velocity Δq/Δt must be ≤ the channel ceiling: base
    /// 1.0 rad over 0.5 s = 2.0 rad/s > 1.0 ceiling → VELOCITY_EXCEEDED.
    #[test]
    fn implied_velocity_exceeds_envelope_rejected() {
        let p = icebot_plan(
            vec![
                icebot_wp([0.0, 0.0, 0.0, 0.01], 0.0),
                icebot_wp([1.0, 0.1, 0.1, 0.02], 0.5), // base Δq = 1.0 over 0.5 s
                icebot_wp([0.0, 0.0, 0.0, 0.01], 1.0),
            ],
            1.0,
        );

        let err = ExecutionManifestBuilder::build(&p).expect_err("build must fail");
        assert!(
            matches!(err, BuilderError::Validation(ref code) if code == "VELOCITY_EXCEEDED"),
            "expected VELOCITY_EXCEEDED, got {err:?}"
        );
    }

    /// Rule 8 — dt_us == 0 makes physical velocity UNDEFINED (Δt = 0): the
    /// backend MUST NOT reject the manifest, the firmware executor
    /// velocity-bounds advancement (ADR-3, dt_us==0 PROTOCOL SEMANTICS —
    /// firmware-authoritative). A hand-built manifest with a huge joint jump
    /// and all-zero dt_us must still validate on timing/velocity (structure
    /// rules still apply).
    #[test]
    fn zero_dt_velocity_skipped_firmware_authoritative() {
        let mut m = two_segment_manifest();
        // Forge the degenerate all-zero-dt shape: same joints as the fixture
        // (all inside the envelope) but every dt_us = 0.
        for s in m.samples.iter_mut() {
            s.dt_us = 0;
        }
        m.metadata.duration_us = 0;

        // Position is fine; velocity is UNDEFINED (dt = 0) → must NOT reject.
        ExecutionManifestBuilder::validate(&m).expect("all-zero dt must not reject");
    }

    /// ADR-6 (spec `planner_valid_movej_accepted_by_firmware`,
    /// PlannerAccepted ⇒ FirmwareAcceptable): a movej the planner accepts
    /// (joints within envelope, requested velocity within ceilings) MUST
    /// produce a manifest every position waypoint AND every implied velocity
    /// the firmware SafetyEnvelope accepts. The converse is NOT required.
    #[test]
    fn planner_accepted_movej_is_firmware_acceptable() {
        // Planner-accepted movej: base/elbow/wrist 0→1.0 rad at 1.0 rad/s,
        // prismatic 0.01→0.03 at 0.04 m/s — all within the envelope.
        let p = icebot_plan(
            vec![
                icebot_wp([0.0, 0.0, 0.0, 0.01], 0.0),
                icebot_wp([1.0, 1.0, 1.0, 0.03], 1.0),
            ],
            1.0,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("planner-accepted movej builds");

        // Position: every waypoint inside the firmware SafetyEnvelope.
        for (i, sample) in manifest.samples.iter().enumerate() {
            SafetyEnvelope::check_joints(&sample.joints)
                .unwrap_or_else(|v| panic!("sample {i} out of firmware envelope: {v}"));
        }
        // Implied velocity: Δq/Δt ≤ ceiling for every dt > 0 gap.
        for (i, pair) in manifest.samples.windows(2).enumerate() {
            let dt = pair[1].dt_us;
            if dt == 0 {
                continue; // undefined velocity — firmware-authoritative
            }
            let delta_q: Vec<f64> = pair[1]
                .joints
                .iter()
                .zip(&pair[0].joints)
                .map(|(q1, q0)| q1 - q0)
                .collect();
            SafetyEnvelope::check_gap_velocity(&delta_q, dt)
                .unwrap_or_else(|v| panic!("gap {i} exceeds firmware velocity ceiling: {v}"));
        }
    }

    /// ADR-6 (spec `remediation_profile_accepted_by_firmware`): a remediation
    /// trajectory (PhysicalEnvelope-bounded, 1.0 rad/s / 600 rad/s² ceilings)
    /// must also produce a firmware-acceptable manifest.
    #[test]
    fn remediation_profile_is_firmware_acceptable() {
        // Clamped_departure_limits-style profile: base 0 → 1.0 rad in 1.0 s.
        let p = icebot_plan(
            vec![
                icebot_wp([0.0, 0.0, 0.0, 0.01], 0.0),
                icebot_wp([0.5, 0.5, 0.5, 0.02], 0.5),
                icebot_wp([1.0, 1.0, 1.0, 0.03], 1.0),
            ],
            1.0,
        );

        let manifest = ExecutionManifestBuilder::build(&p).expect("remediation profile builds");
        for sample in &manifest.samples {
            SafetyEnvelope::check_joints(&sample.joints).expect("remediation joint in envelope");
        }
        // The 0.5 s gaps imply 1.0 rad/s for revolute joints — at the ceiling.
        assert_eq!(manifest.samples[1].dt_us, 500_000);
    }

    /// ADR-2 / spec `No Silent Mutation` (Correction F): an out-of-envelope
    /// joint MUST be REJECTED, never silently clamped to the envelope. The
    /// builder returns an error — no manifest with modified joints exists.
    #[test]
    fn out_of_envelope_joint_rejected_never_clamped() {
        // base requested at 2.0 rad (would have been clamped to 1.5708 on
        // pre-change code) — must be an explicit rejection.
        let p = icebot_plan(
            vec![
                icebot_wp([0.0, 0.0, 0.0, 0.01], 0.0),
                icebot_wp([2.0, 0.5, 0.5, 0.02], 0.5),
                icebot_wp([0.0, 0.0, 0.0, 0.01], 1.0),
            ],
            1.0,
        );

        let err = ExecutionManifestBuilder::build(&p).expect_err("build must reject, never clamp");
        assert!(
            matches!(err, BuilderError::Validation(ref code) if code == "INVALID_JOINT"),
            "expected explicit INVALID_JOINT rejection, got {err:?}"
        );
    }

    /// ADR-2 no-silent-mutation (backend half): `validate()` on a hand-built
    /// manifest MUST return the error AND leave the input manifest bit-exact —
    /// the reject path never rewrites joints into the envelope.
    #[test]
    fn validate_rejects_out_of_envelope_without_mutating_input() {
        let mut m = two_segment_manifest();
        // Corrupt ONLY the middle sample's base joint (in place).
        m.samples[1].joints[0] = 2.0; // > +1.5708

        let err = ExecutionManifestBuilder::validate(&m).expect_err("must reject");
        assert!(
            matches!(err, BuilderError::Validation(ref code) if code == "INVALID_JOINT"),
            "expected INVALID_JOINT, got {err:?}"
        );

        // The input was NOT silently clamped to 1.5708 — it still holds 2.0.
        assert_eq!(
            m.samples[1].joints[0], 2.0,
            "validate() must never mutate the manifest joints"
        );
    }
}
