//! Trapezoidal velocity profile for cartesian moves (spec
//! `move-l-velocity-profile`).
//!
//! The math is ported from `interpolate/joint.rs` (which stays untouched):
//! `d_acc = v²/(2a)`; when `2·d_acc >= d` the profile is triangular
//! (`T = 2·sqrt(d/a)`, `v_peak = sqrt(d·a) <= v_max`), otherwise it has
//! accel + cruise + decel phases (`T = 2·(v/a) + (d − 2·d_acc)/v`). The
//! profile is distance-based and pure — `distance_at(t)` feeds MoveL's
//! `position(t) = start + (distance(t)/d)·(end − start)`.

/// Distance covered during the acceleration phase: `d_acc = v²/(2a)`.
fn accel_distance(max_velocity: f64, max_acceleration: f64) -> f64 {
    let v = max_velocity.abs();
    let a = max_acceleration.abs();
    if a > 1e-12 { (v * v) / (2.0 * a) } else { 0.0 }
}

/// Total duration of the profile over `distance` (spec
/// `move-l-velocity-profile`):
///
/// - `2·d_acc >= d` → triangular: `T = 2·sqrt(d/a)` (no cruise phase);
/// - otherwise → trapezoidal: `T = 2·(v/a) + (d − 2·d_acc)/v`.
///
/// Returns `0.0` for zero distance or a degenerate (zero) limit.
pub fn total_time(distance: f64, max_velocity: f64, max_acceleration: f64) -> f64 {
    let d = distance.max(0.0);
    let v = max_velocity.abs();
    let a = max_acceleration.abs();
    if d < 1e-12 || a < 1e-12 || v < 1e-12 {
        return 0.0;
    }
    let d_acc = accel_distance(v, a);
    if 2.0 * d_acc >= d {
        2.0 * (d / a).sqrt()
    } else {
        2.0 * (v / a) + (d - 2.0 * d_acc) / v
    }
}

/// Absolute distance travelled by time `t` along the profile, in
/// `[0, distance]` (spec `move-l-velocity-profile`: time → distance(t)).
///
/// Starts and ends at rest (v(0) = v(T) = 0) and is monotonic; the
/// trapezoidal phases are quadratic accel, linear cruise, quadratic decel.
pub fn distance_at(
    t: f64,
    distance: f64,
    max_velocity: f64,
    max_acceleration: f64,
    profile_total_time: f64,
) -> f64 {
    let d = distance.max(0.0);
    let v = max_velocity.abs();
    let a = max_acceleration.abs();
    if t <= 0.0 || profile_total_time <= 1e-12 {
        return 0.0;
    }
    if t >= profile_total_time {
        return d;
    }

    let d_acc = accel_distance(v, a);
    if 2.0 * d_acc >= d {
        // Triangular: accelerate to the halfway point, then decelerate.
        let t_half = profile_total_time / 2.0;
        if t <= t_half {
            0.5 * a * t * t
        } else {
            let t_remaining = profile_total_time - t;
            d - 0.5 * a * t_remaining * t_remaining
        }
    } else {
        // Trapezoidal: accel → cruise at v_max → decel.
        let t_acc = v / a;
        if t <= t_acc {
            0.5 * a * t * t
        } else if t <= profile_total_time - t_acc {
            d_acc + v * (t - t_acc)
        } else {
            let t_remaining = profile_total_time - t;
            d - 0.5 * a * t_remaining * t_remaining
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const V_MAX: f64 = 0.1;
    const A_MAX: f64 = 0.5;

    /// Spec "Boundary — 20mm @ 0.1 m/s, 0.5 m/s²": 2·d_acc == d (boundary
    /// triangular), so T = 2·sqrt(d/a) = 2·sqrt(0.04) = 0.4s and the peak
    /// velocity equals v_max (0.1).
    #[test]
    fn boundary_20mm_triangular_profile_matches_spec() {
        let t = total_time(0.02, V_MAX, A_MAX);
        assert!(
            (t - 0.4).abs() < 1e-9,
            "T must be 2·sqrt(d/a) = 0.4s, got {t}"
        );
        // Half the travel time → half the distance (symmetric triangular).
        let mid = distance_at(t / 2.0, 0.02, V_MAX, A_MAX, t);
        assert!(
            (mid - 0.01).abs() < 1e-9,
            "half-time must reach half-distance, got {mid}"
        );
        // Exact endpoints: distance(0) = 0, distance(T) = d.
        assert_eq!(distance_at(0.0, 0.02, V_MAX, A_MAX, t), 0.0);
        assert!(
            (distance_at(t, 0.02, V_MAX, A_MAX, t) - 0.02).abs() < 1e-9,
            "distance(T) must equal the full travel"
        );
    }

    /// Spec "Triangular fallback (short distance)": with 2·d_acc >= d there is
    /// no cruise phase — v_peak = sqrt(d·a) <= v_max and T = 2·sqrt(d/a).
    /// d = 5mm: 2·d_acc = 0.02 >= 0.005 → triangular, v_peak = 0.05, T = 0.2s.
    #[test]
    fn triangular_profile_for_short_distance() {
        let d = 0.005;
        let t = total_time(d, V_MAX, A_MAX);
        assert!(
            (t - 0.2).abs() < 1e-9,
            "triangular T must be 2·sqrt(d/a) = 0.2s, got {t}"
        );
        let v_peak = (d * A_MAX).sqrt();
        assert!(
            (v_peak - 0.05).abs() < 1e-12,
            "v_peak must be sqrt(d·a) = 0.05, got {v_peak}"
        );
        assert!(v_peak <= V_MAX, "v_peak must not exceed v_max");
    }

    /// Spec "Full trapezoidal (cruise reached)": with 2·d_acc < d the profile
    /// has accel + cruise + decel — T = 2·(v/a) + (d − 2·d_acc)/v. d = 0.1m:
    /// 2·d_acc = 0.02 < 0.1 → T = 0.4 + 0.8 = 1.2s.
    #[test]
    fn trapezoidal_profile_with_cruise() {
        let d = 0.1;
        let t = total_time(d, V_MAX, A_MAX);
        assert!(
            (t - 1.2).abs() < 1e-9,
            "trapezoidal T must be 1.2s, got {t}"
        );
        let d_acc = V_MAX * V_MAX / (2.0 * A_MAX); // 0.01
        // Cruise starts at t_acc = v/a = 0.2 with exactly d_acc travelled.
        let at_cruise = distance_at(0.2, d, V_MAX, A_MAX, t);
        assert!(
            (at_cruise - d_acc).abs() < 1e-9,
            "cruise start must be at d_acc, got {at_cruise}"
        );
        // Mid-cruise is linear: distance = d_acc + v·(t − t_acc).
        let at_mid = distance_at(0.5, d, V_MAX, A_MAX, t);
        let expected = d_acc + V_MAX * (0.5 - 0.2);
        assert!(
            (at_mid - expected).abs() < 1e-9,
            "cruise phase must be linear, got {at_mid}"
        );
        // Exact endpoints.
        assert_eq!(distance_at(0.0, d, V_MAX, A_MAX, t), 0.0);
        assert!(
            (distance_at(t, d, V_MAX, A_MAX, t) - d).abs() < 1e-9,
            "distance(T) must equal the full travel"
        );
    }

    /// Spec "move-l-profile-sampling-bounds": the discretized profile never
    /// exceeds v_max or a_max, and distance is monotonic.
    #[test]
    fn monotonic_and_within_bounds_across_the_profile() {
        let d = 0.1;
        let t = total_time(d, V_MAX, A_MAX);
        let dt = 0.01;
        let n = (t / dt).ceil() as usize;

        let mut prev_distance = 0.0_f64;
        let mut prev_velocity = 0.0_f64;
        let mut max_v: f64 = 0.0;
        let mut max_a: f64 = 0.0;

        for i in 0..=n {
            let time = (i as f64 * dt).min(t);
            let s = distance_at(time, d, V_MAX, A_MAX, t);
            assert!(
                s >= prev_distance - 1e-12,
                "distance must be monotonic at t={time}: {s} < {prev_distance}"
            );
            let v = if i == 0 {
                0.0
            } else {
                (s - prev_distance) / dt
            };
            max_v = max_v.max(v);
            if i > 0 {
                max_a = max_a.max((v - prev_velocity).abs() / dt);
            }
            prev_velocity = v;
            prev_distance = s;
        }

        assert!(
            max_v <= V_MAX + 1e-9,
            "implied velocity {max_v:.6} must not exceed v_max"
        );
        assert!(
            max_a <= A_MAX + 1e-6,
            "implied acceleration {max_a:.6} must not exceed a_max"
        );
    }
}
