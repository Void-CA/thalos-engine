use thalos_core::trajectory::TrajectoryPoint;

pub fn trapezoidal_profile(
    start: &[f64],
    end: &[f64],
    max_velocity: f64,
    max_acceleration: f64,
    dt: f64,
) -> Vec<TrajectoryPoint> {
    debug_assert_eq!(start.len(), end.len());

    let max_displacement: f64 = start
        .iter()
        .zip(end.iter())
        .map(|(a, b)| (b - a).abs())
        .fold(0.0_f64, f64::max);

    if max_displacement < 1e-12 {
        return vec![TrajectoryPoint::new(start.to_vec(), 0.0)];
    }

    let v = max_velocity.abs();
    let a = max_acceleration.abs();
    let t_acc = if a > 1e-12 { v / a } else { 0.0 };
    let d_acc = 0.5 * a * t_acc * t_acc;

    let total_time = if d_acc * 2.0 >= max_displacement {
        let t_peak = (max_displacement / a).sqrt();
        t_peak * 2.0
    } else {
        let cruise_distance = max_displacement - 2.0 * d_acc;
        let t_cruise = if v > 1e-12 { cruise_distance / v } else { 0.0 };
        2.0 * t_acc + t_cruise
    };

    if total_time < 1e-12 {
        return vec![TrajectoryPoint::new(start.to_vec(), 0.0)];
    }

    let num_points = (total_time / dt).ceil() as usize;
    let mut trajectory = Vec::with_capacity(num_points);

    for i in 0..=num_points {
        let t = (i as f64 * dt).min(total_time);
        let s = normalised_position(t, total_time, max_displacement, a, v, t_acc, d_acc);
        let joints: Vec<f64> = start
            .iter()
            .zip(end.iter())
            .map(|(a, b)| a + (b - a) * s)
            .collect();
        trajectory.push(TrajectoryPoint::new(joints, t));
    }

    trajectory
}

fn normalised_position(
    t: f64,
    total_time: f64,
    total_dist: f64,
    accel: f64,
    max_vel: f64,
    t_acc: f64,
    d_acc: f64,
) -> f64 {
    if t <= 1e-12 {
        return 0.0;
    }
    if t >= total_time - 1e-12 {
        return 1.0;
    }

    let is_triangular = d_acc * 2.0 >= total_dist;

    if is_triangular {
        let t_half = total_time / 2.0;
        if t <= t_half {
            0.5 * accel * t * t / total_dist
        } else {
            let t_dec = total_time - t;
            1.0 - 0.5 * accel * t_dec * t_dec / total_dist
        }
    } else {
        let t_cruise_start = t_acc;
        let t_cruise_end = total_time - t_acc;

        if t <= t_cruise_start {
            let d = 0.5 * accel * t * t;
            d / total_dist
        } else if t <= t_cruise_end {
            let d = d_acc + max_vel * (t - t_acc);
            d / total_dist
        } else {
            let t_rem = total_time - t;
            let d = total_dist - 0.5 * accel * t_rem * t_rem;
            d / total_dist
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trapezoidal_identical_start_end() {
        let q = vec![1.0, 2.0, 3.0];
        let traj = trapezoidal_profile(&q, &q, 1.0, 1.0, 0.01);
        assert_eq!(traj.len(), 1);
        assert!((traj[0].timestamp() - 0.0).abs() < 1e-12);
    }

    #[test]
    fn trapezoidal_starts_at_zero_and_ends_at_one() {
        let start = vec![0.0, 0.0];
        let end = vec![1.0, 2.0];
        let traj = trapezoidal_profile(&start, &end, 0.5, 0.2, 0.01);
        let first = &traj[0];
        let last = &traj[traj.len() - 1];
        for (j, s) in first.joints().iter().zip(start.iter()) {
            assert!((j - s).abs() < 1e-10);
        }
        for (j, e) in last.joints().iter().zip(end.iter()) {
            assert!((j - e).abs() < 1e-10);
        }
    }

    #[test]
    fn trapezoidal_monotonic_timestamps() {
        let start = vec![0.0];
        let end = vec![10.0];
        let traj = trapezoidal_profile(&start, &end, 1.0, 0.5, 0.01);
        for w in traj.windows(2) {
            assert!(w[0].timestamp() < w[1].timestamp());
        }
    }
}
