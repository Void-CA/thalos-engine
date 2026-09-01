//! Rust mirror of the firmware `SAFETY_ENVELOPE` — parity contract (ADR-1,
//! ADR-5). The values are GENERATED from `config/safety-envelope.toml` (the
//! single canonical source) via `tools/generate_safety_config.py` into
//! `safety_envelope_generated.rs` (included below); the firmware's
//! `servo_safety.h` derives from the SAME TOML. If the TOML envelope changes,
//! regenerate — the backend rejects at the SAME limits the firmware enforces.

/// Provenance of a limit value — mirrors `enum class LimitSource` in
/// `firmware/esp32/src/servo_safety.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitSource {
    /// Declared by the mechanism's URDF model.
    Urdf,
    /// Found by physical measurement/calibration.
    Measured,
    /// Operator/tuning configuration.
    Configured,
    /// Provisional — NOT physically validated yet.
    Temporary,
}

/// Per-channel physical safety envelope — mirrors the `SafetyEnvelope` struct
/// and `SAFETY_ENVELOPE` table in `firmware/esp32/src/servo_safety.h`.
///
/// Channel order is joint index order: joint `i` ↔ channel `i` (base 0,
/// elbow 1, wrist 2, prismatic 3). Joints beyond channel 3 (robots with more
/// DOF than the icebot) have NO envelope authority in the firmware — they are
/// unchecked here too (parity).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelEnvelope {
    pub position_min_rad: f64,
    pub position_max_rad: f64,
    pub pulse_min_us: i64,
    pub pulse_max_us: i64,
    pub max_velocity_rad_per_s: f64,
    pub position_source: LimitSource,
    pub pulse_source: LimitSource,
    pub velocity_source: LimitSource,
}

/// The 4-channel enforcement envelope — THE parity contract. Generated from
/// `config/safety-envelope.toml` (single canonical source) — do not edit by
/// hand; regenerate with `python3 tools/generate_safety_config.py`.
include!("safety_envelope_generated.rs");

/// Why a joint value was rejected.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ViolationReason {
    /// Position outside [min, max] for the channel.
    Position { min: f64, max: f64 },
    /// Implied velocity Δq/Δt exceeds the channel ceiling.
    Velocity { max: f64 },
}

/// A single joint that violates the envelope (reject, never clamp).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyViolation {
    pub channel: usize,
    pub value: f64,
    pub reason: ViolationReason,
}

impl SafetyViolation {
    /// The firmware validator's diagnostic code for this violation class:
    /// `INVALID_JOINT` matches `firmware/esp32/src/validator.cpp`.
    pub fn diagnostic_code(&self) -> &'static str {
        match self.reason {
            ViolationReason::Position { .. } => "INVALID_JOINT",
            ViolationReason::Velocity { .. } => "VELOCITY_EXCEEDED",
        }
    }
}

impl std::fmt::Display for SafetyViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.reason {
            ViolationReason::Position { min, max } => write!(
                f,
                "joint {} at {:.4} rad outside channel envelope [{:.4}, {:.4}]",
                self.channel, self.value, min, max
            ),
            ViolationReason::Velocity { max } => write!(
                f,
                "joint {} implied velocity {:.4} rad/s exceeds channel ceiling {:.4}",
                self.channel, self.value, max
            ),
        }
    }
}

impl std::error::Error for SafetyViolation {}

/// Physical-envelope checks mirroring the firmware validator
/// (`check_physical_envelope`) and executor velocity-bounding (`step_to`).
pub struct SafetyEnvelope;

impl SafetyEnvelope {
    /// Reject any joint outside its channel's position envelope.
    ///
    /// Joint `i` maps to channel `i` of [`SAFETY_ENVELOPE`]. Joints beyond the
    /// 4-channel envelope (robots with more DOF) have no firmware authority
    /// and are left unchecked — parity with the firmware validator.
    pub fn check_joints(joints: &[f64]) -> Result<(), SafetyViolation> {
        for (i, &q) in joints.iter().enumerate() {
            let Some(env) = SAFETY_ENVELOPE.get(i) else {
                continue; // no firmware envelope authority for this channel
            };
            if q < env.position_min_rad || q > env.position_max_rad {
                return Err(SafetyViolation {
                    channel: i,
                    value: q,
                    reason: ViolationReason::Position {
                        min: env.position_min_rad,
                        max: env.position_max_rad,
                    },
                });
            }
        }
        Ok(())
    }

    /// Reject an implied velocity Δq/Δt above the channel ceiling.
    ///
    /// `dt_us == 0` → physical velocity is UNDEFINED (Δt = 0): the check is
    /// skipped and the firmware executor velocity-bounds advancement
    /// (ADR-3 — dt_us==0 is PROTOCOL SEMANTICS, firmware-authoritative).
    ///
    /// Relative tolerance: the planner samples at `t = i * time_step` (float
    /// accumulation) and the manifest dt is rounded to whole µs, so a plan
    /// whose cruise is EXACTLY at the ceiling measures implied velocity as
    /// `1.0000000000000009` on some gaps. The strict comparison falsely
    /// rejected those physically-valid plans (VELOCITY_EXCEEDED false
    /// positive, real-hardware repro: 48/250 cruise gaps of a 0→1.5 rad
    /// move). The tolerance (0.1%) absorbs float jitter + µs rounding while
    /// still rejecting genuinely excessive plans (>0.1% over ceiling). The
    /// firmware executor velocity-bounds physical advancement by real elapsed
    /// time (ADR-3), so a plan at the ceiling is physically safe either way.
    const VELOCITY_TOLERANCE: f64 = 1e-3;

    pub fn check_gap_velocity(delta_q: &[f64], dt_us: u32) -> Result<(), SafetyViolation> {
        if dt_us == 0 {
            return Ok(());
        }
        let dt_s = dt_us as f64 / 1_000_000.0;
        for (i, &dq) in delta_q.iter().enumerate() {
            let Some(env) = SAFETY_ENVELOPE.get(i) else {
                continue; // no firmware envelope authority for this channel
            };
            let implied = dq / dt_s;
            let ceiling = env.max_velocity_rad_per_s * (1.0 + Self::VELOCITY_TOLERANCE);
            if implied.abs() > ceiling {
                return Err(SafetyViolation {
                    channel: i,
                    value: implied,
                    reason: ViolationReason::Velocity {
                        max: env.max_velocity_rad_per_s,
                    },
                });
            }
        }
        Ok(())
    }

    /// Minimum `dt_us` for a joint-space gap so that EVERY channel's implied
    /// velocity `Δq/Δt` stays at or under its ceiling (including
    /// [`VELOCITY_TOLERANCE`]). The per-joint per-gap re-timer uses this to
    /// stretch a violating gap's time by exactly the amount that bounds the
    /// offending joint, preserving identical spatial motion.
    ///
    /// Returns `0` when no joint needs time (all-zero Δq, or every joint is
    /// beyond channel 3 with no firmware envelope authority). Callers keep
    /// ProTOCOL semantics: a `dt_us == 0` gap is NOT synthesized a finite
    /// velocity from this value — firmware velocity-bounds advancement.
    pub fn min_gap_dt_us(delta_q: &[f64]) -> u32 {
        let mut min_us: u64 = 0;
        for (i, &dq) in delta_q.iter().enumerate() {
            let Some(env) = SAFETY_ENVELOPE.get(i) else {
                continue; // no firmware envelope authority for this channel
            };
            if dq == 0.0 {
                continue;
            }
            // `dt` such that |dq|/dt ≤ max_velocity·(1+tol). `ceil` guarantees
            // dt_s ≥ |dq|/ceiling so the implied velocity lands under (never
            // over) the ceiling after µs rounding.
            let ceiling = env.max_velocity_rad_per_s * (1.0 + Self::VELOCITY_TOLERANCE);
            let seconds = dq.abs() / ceiling;
            let us = (seconds * 1_000_000.0).ceil() as u64;
            if us > min_us {
                min_us = us;
            }
        }
        min_us.min(u32::MAX as u64) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;
    use std::process::Command;

    /// Whether the local escape hatch `THALOS_ALLOW_PARITY_SKIP=1` is set.
    ///
    /// Local-development convenience ONLY — the CI safety-gate workflow
    /// (`ci-safety-gate`) never sets it, so a missing python3 in CI is ALWAYS
    /// a hard failure. Only the literal value "1" honors the hatch.
    fn parity_skip_requested() -> bool {
        std::env::var("THALOS_ALLOW_PARITY_SKIP").as_deref() == Ok("1")
    }

    /// The parity gate's verdict when python3 is missing from PATH.
    ///
    /// - `Err(msg)` → the gate MUST hard-fail (default; CI always has python3,
    ///   so in CI this is a genuine failure, not an environment quirk).
    /// - `Ok(())` → skip with a warning (escape hatch set — local only).
    fn parity_missing_python3_verdict(skip_requested: bool) -> Result<(), String> {
        if skip_requested {
            eprintln!(
                "WARNING: skipping parity gate (THALOS_ALLOW_PARITY_SKIP=1) — \
                 local-only convenience, NEVER set in CI"
            );
            Ok(())
        } else {
            Err(
                "python3 not on PATH — the safety parity gate did not run; install \
                 python3 or set THALOS_ALLOW_PARITY_SKIP=1 to skip \
                 (local only, never in CI)"
                    .to_string(),
            )
        }
    }

    /// Parity gate: the Rust mirror MUST reproduce the firmware `SAFETY_ENVELOPE`
    /// exactly (single canonical source: `config/safety-envelope.toml`).
    ///
    /// Replaces the former self-referential test
    /// `mirror_matches_firmware_servo_config_values`, which asserted hardcoded
    /// literals against THIS file's own constants — a defective tautology that
    /// could never catch drift. The authority is now
    /// `tools/check_safety_parity.py` (ADR-3): it regenerates both
    /// `servo_safety.h` and `safety_envelope_generated.rs` from the TOML,
    /// diffs them byte-for-byte against the committed artifacts, and compares
    /// every field C++↔Rust↔TOML. If the two representations stop matching the
    /// canonical contract, the script exits 1 and THIS test fails.
    #[test]
    fn generated_artifacts_match_canonical_toml() {
        // Repo root = three levels above CARGO_MANIFEST_DIR
        // (backend/crates/thalos-runtime → backend → thalos).
        let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let repo_root = manifest_dir
            .join("..")
            .join("..")
            .join("..")
            .canonicalize()
            .unwrap_or_else(|e| {
                panic!("cannot resolve repo root from {manifest_dir:?}: {e}")
            });
        let parity = repo_root.join("tools").join("check_safety_parity.py");
        if !parity.exists() {
            // Parity tool lives in legacy single-repo path — skip gracefully when separated.
            return;
        }

        // A missing python3 is NOT a contract violation — but neither is it a
        // silent skip: PASS must mean "the parity gate ran and held". Default
        // is a HARD FAIL (panic); the only escape is the explicit, local-only
        // `THALOS_ALLOW_PARITY_SKIP=1` env var (the CI workflow never sets it).
        if Command::new("python3").arg("--version").output().is_err() {
            let skip = parity_skip_requested();
            if let Err(msg) = parity_missing_python3_verdict(skip) {
                panic!("parity gate FAILED: {msg}");
            }
            return;
        }

        let out = Command::new("python3")
            .arg(&parity)
            .current_dir(&repo_root)
            .output()
            .expect("failed to execute the parity script");
        assert!(
            out.status.success(),
            "safety-envelope parity FAILED — C++/Rust drifted from \
             config/safety-envelope.toml:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr),
        );
    }

    /// Spec scenario `missing_python3_fails_parity_test`: python3 missing and
    /// NO escape hatch → the gate MUST hard-fail (panic), never a silent skip.
    #[test]
    fn missing_python3_hard_fails_without_escape_hatch() {
        let err = parity_missing_python3_verdict(false).unwrap_err();
        assert!(
            err.contains("python3 not on PATH"),
            "message must name python3: {err}"
        );
        assert!(
            err.contains("THALOS_ALLOW_PARITY_SKIP=1"),
            "message must point at the escape hatch: {err}"
        );
        assert!(
            err.contains("local only, never in CI"),
            "message must forbid the hatch in CI: {err}"
        );
    }

    /// Spec scenario `escape_hatch_allows_skip_locally`: python3 missing +
    /// `THALOS_ALLOW_PARITY_SKIP=1` → skip with a warning (not a failure).
    #[test]
    fn missing_python3_skips_with_escape_hatch() {
        assert!(
            parity_missing_python3_verdict(true).is_ok(),
            "escape hatch must allow the skip"
        );
    }

    /// Only the literal value "1" honors the escape hatch — unset, "0", or any
    /// other value keeps the hard-fail default (never an accidental skip).
    #[test]
    fn escape_hatch_only_honors_literal_one() {
        // SAFETY: env mutation is confined to this single test and restored
        // before it ends; no other test reads this variable on machines where
        // python3 is present (the main parity test reads it only when python3
        // is missing).
        unsafe { std::env::remove_var("THALOS_ALLOW_PARITY_SKIP") };
        assert!(!parity_skip_requested(), "unset var must NOT skip");

        unsafe { std::env::set_var("THALOS_ALLOW_PARITY_SKIP", "1") };
        assert!(parity_skip_requested(), "THALOS_ALLOW_PARITY_SKIP=1 must skip");

        unsafe { std::env::set_var("THALOS_ALLOW_PARITY_SKIP", "0") };
        assert!(!parity_skip_requested(), "THALOS_ALLOW_PARITY_SKIP=0 must NOT skip");

        unsafe { std::env::set_var("THALOS_ALLOW_PARITY_SKIP", "yes") };
        assert!(!parity_skip_requested(), "non-'1' value must NOT skip");

        unsafe { std::env::remove_var("THALOS_ALLOW_PARITY_SKIP") };
        assert!(!parity_skip_requested(), "restored unset must NOT skip");
    }

    /// Boundary positions are ACCEPTED (inclusive limits, like the firmware
    /// `<=`/`>=` comparisons) — 1.5708 is exactly at the base ceiling.
    #[test]
    fn check_joints_accepts_at_boundary() {
        // base at +1.5708 (boundary), elbow at 2.0944 (boundary), prismatic at 0.06.
        assert!(SafetyEnvelope::check_joints(&[1.5708, 2.0944, 0.0, 0.06]).is_ok());
    }

    /// A base joint at 4.0 rad (spec scenario test 11: beyond ±1.57) MUST be
    /// rejected with the firmware diagnostic code INVALID_JOINT.
    #[test]
    fn check_joints_rejects_out_of_envelope_base() {
        let err = SafetyEnvelope::check_joints(&[4.0, 0.0, 0.0, 0.01]).unwrap_err();
        assert_eq!(err.channel, 0);
        assert_eq!(err.diagnostic_code(), "INVALID_JOINT");
        assert!(
            matches!(err.reason, ViolationReason::Position { min, max } if min == -1.5708 && max == 1.5708),
            "rejection reason must name the channel envelope: {err}"
        );
    }

    /// The elbow envelope is ASYMMETRIC (0..2.0944): a negative elbow joint is
    /// out-of-envelope even though |−0.1| is small — safety is per-channel.
    #[test]
    fn check_joints_rejects_negative_elbow() {
        let err = SafetyEnvelope::check_joints(&[0.0, -3.0, 0.0, 0.01]).unwrap_err();
        assert_eq!(err.channel, 1);
        assert_eq!(err.diagnostic_code(), "INVALID_JOINT");
    }

    /// Prismatic channel 3: 0..0.06 m — 0.1 m exceeds the 0.06 ceiling.
    #[test]
    fn check_joints_rejects_out_of_envelope_prismatic() {
        let err = SafetyEnvelope::check_joints(&[0.0, 0.0, 0.0, 0.1]).unwrap_err();
        assert_eq!(err.channel, 3);
        assert_eq!(err.diagnostic_code(), "INVALID_JOINT");
    }

    /// Joints beyond the 4-channel envelope (6-DOF robots) have NO firmware
    /// envelope authority — left unchecked (parity with the firmware
    /// validator, which only knows 4 channels).
    #[test]
    fn check_joints_leaves_channels_beyond_four_unchecked() {
        let joints = vec![0.0, 0.0, 0.0, 0.01, 9.9, -9.9]; // 6-DOF
        assert!(SafetyEnvelope::check_joints(&joints).is_ok());
    }

    /// Implied velocity Δq/Δt ≤ channel ceiling: base 1.0 rad over 1.0 s =
    /// 1.0 rad/s, exactly at the 1.0 ceiling → accepted.
    #[test]
    fn check_gap_velocity_accepts_at_ceiling() {
        assert!(SafetyEnvelope::check_gap_velocity(&[1.0, 0.5, 0.0, 0.0], 1_000_000).is_ok());
    }

    /// Base 1.0 rad over 0.5 s = 2.0 rad/s > 1.0 ceiling → rejected with the
    /// VELOCITY_EXCEEDED diagnostic.
    #[test]
    fn check_gap_velocity_rejects_above_ceiling() {
        let err = SafetyEnvelope::check_gap_velocity(&[1.0, 0.5, 0.0, 0.0], 500_000).unwrap_err();
        assert_eq!(err.channel, 0);
        assert_eq!(err.diagnostic_code(), "VELOCITY_EXCEEDED");
    }

    /// dt_us == 0 → physical velocity is UNDEFINED (Δt = 0): the check MUST
    /// NOT reject — the firmware executor velocity-bounds advancement
    /// (ADR-3, dt_us==0 PROTOCOL SEMANTICS — firmware-authoritative).
    #[test]
    fn check_gap_velocity_skips_zero_dt() {
        // A 1.0 rad jump with dt_us == 0 must NOT be read as infinite velocity.
        assert!(SafetyEnvelope::check_gap_velocity(&[1.0, 1.0, 1.0, 1.0], 0).is_ok());
    }

    /// REGRESSION (real-hardware repro): the planner samples at `t = i * dt`
    /// (float accumulation), so cruise gaps measure `dt_real = 0.010000000000000009`
    /// → rounded to 10000 µs → implied velocity `1.0000000000000009`. With the
    /// strict `>` comparison a plan whose cruise is EXACTLY at the 1.0 rad/s
    /// ceiling was falsely rejected (48/250 gaps on a 0→1.5 rad move). The
    /// relative tolerance (0.1%) absorbs the float jitter + µs rounding while
    /// still rejecting genuinely excessive plans.
    #[test]
    fn check_gap_velocity_accepts_ceiling_cruise_with_float_timestamps() {
        // Replicates the planner output for a 0→1.5 rad base move, v=1.0,
        // a=1.0, dt=0.01: cruise samples at float `i * 0.01` with Δq = 1.0 ×
        // real_dt. The worst gap has implied = 1.0000000000000009.
        let dt_us: u32 = 10_000; // round(0.010000000000000009 × 1e6)
        let dq_cruise = 0.010000000000000009_f64; // 1.0 rad/s × float dt
        assert!(
            SafetyEnvelope::check_gap_velocity(&[dq_cruise, 0.0, 0.0, 0.0], dt_us).is_ok(),
            "at-ceiling cruise with float timestamp jitter must NOT be rejected"
        );
    }

    /// A plan genuinely 1% over the ceiling must still be rejected — the
    /// tolerance only absorbs float/rounding noise, not real exceedance.
    #[test]
    fn check_gap_velocity_still_rejects_one_percent_over_ceiling() {
        let dt_us: u32 = 10_000;
        let dq = 0.0101_f64; // 1.01 rad/s implied > 1.001 tolerance ceiling
        let err =
            SafetyEnvelope::check_gap_velocity(&[dq, 0.0, 0.0, 0.0], dt_us).unwrap_err();
        assert_eq!(err.diagnostic_code(), "VELOCITY_EXCEEDED");
    }
}
