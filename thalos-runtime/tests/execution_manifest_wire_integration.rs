//! Integration test: the pure plan→wire chain.
//!
//! Exercises the FULL chain end-to-end, exactly as the spec scenario
//! `integration_compiled_plan_to_wire_output` requires:
//!
//! ```text
//! CompiledPlan
//!     ↓ ExecutionPlanBuilder        (thalos-planning)
//! ExecutionPlan
//!     ↓ ExecutionManifestBuilder    (thalos-runtime::execution_boundary)
//! ExecutionManifest
//!     ↓ Esp32Codec::encode_manifest_full  (thalos-transport::esp32::codec)
//! wire lines: MANIFEST / SEGMENT / SAMPLE / END_UPLOAD
//! ```
//!
//! What is verified:
//!
//! - Real (non-uniform) absolute timestamps survive to delta `dt_us` on the wire.
//! - Segment provenance maps 1:1 to `SEGMENT` lines with `movej`/`movel` tokens.
//! - The exact wire output (line for line) matches the documented protocol.
//!
//! What is NOT verified here: transport I/O (uses `encode_manifest` directly,
//! no `FakeTransport`), the internal builders' unit behaviors (covered by their
//! own test modules), and hardware (none).

use thalos_engine::core::ids::OperationId;
use thalos_engine::core::motion::segment::MotionSegment;
use thalos_engine::core::prelude::{Trajectory, TrajectoryPoint};
use thalos_engine::core::spatial::frame::FrameId;
use thalos_engine::core::spatial::pose::Pose;
use thalos_engine::math::Transform3D;
use thalos_engine::planning::execution_plan_builder::ExecutionPlanBuilder;
use thalos_engine::planning::motion::program::{CompiledPlan, PlannedSegment};
use thalos_runtime::execution_boundary::manifest_builder::ExecutionManifestBuilder;
use thalos_transport::esp32::codec::Esp32Codec;


// ---------------------------------------------------------------------------
// Fixtures — same style as `plan_with_movej_segment` (thalos-planning) and the
// PR1/PR2 builder tests: hand-built `CompiledPlan` values with known waypoints,
// segments, and timestamps.
// ---------------------------------------------------------------------------

fn movej(origin: &str, joints: Vec<f64>) -> MotionSegment {
    MotionSegment::MoveJ {
        origin: OperationId(origin.to_string()),
        target: joints,
        max_velocity: Some(500.0),
        max_acceleration: Some(1000.0),
    }
}

fn movel(origin: &str) -> MotionSegment {
    MotionSegment::MoveL {
        origin: OperationId(origin.to_string()),
        frame: FrameId::World,
        target_pose: Pose::new(FrameId::World, FrameId::Id(1), Transform3D::identity()),
        max_velocity: Some(200.0),
    }
}

fn planned(
    source: MotionSegment,
    waypoint_range: std::ops::Range<usize>,
    time_range: std::ops::Range<f64>,
) -> PlannedSegment {
    PlannedSegment {
        origin: source.origin().clone(),
        source,
        trajectory: Trajectory::new(vec![]),
        waypoint_range,
        time_range,
        operation_id: None,
        role: None,
    }
}

/// Two segments (MoveJ then MoveL) with NON-uniform timestamps
/// `0.0, 0.5, 1.5` — proves real timestamps reach the wire, not the legacy
/// uniform spacing. Duration = 1.5 s (last waypoint).
fn movej_then_movel_plan() -> CompiledPlan {
    let merged = Trajectory::new(vec![
        TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
        TrajectoryPoint::new(vec![0.5, 0.3], 0.5),
        TrajectoryPoint::new(vec![1.0, 0.6], 1.5),
    ]);
    CompiledPlan::new(
        merged,
        vec![
            planned(movej("op-j", vec![0.5, 0.3]), 0..2, 0.0..0.5),
            planned(movel("op-l"), 2..3, 0.5..1.5),
        ],
    )
}

/// Single MoveJ segment with EVENLY spaced timestamps `0.0, 0.5, 1.0, 1.5` —
/// the shape the legacy `build_manifest` produced, so the chain's wire output
/// must match the legacy format for the case where both are defined to agree.
fn evenly_spaced_movej_plan() -> CompiledPlan {
    let merged = Trajectory::new(vec![
        TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
        TrajectoryPoint::new(vec![0.5, 0.3], 0.5),
        TrajectoryPoint::new(vec![1.0, 0.6], 1.0),
        TrajectoryPoint::new(vec![1.5, 0.9], 1.5),
    ]);
    CompiledPlan::new(
        merged,
        vec![planned(movej("op-j", vec![1.5, 0.9]), 0..4, 0.0..1.5)],
    )
}

/// Helper: run the full chain and return the wire lines as strings.
fn wire_lines(plan: &CompiledPlan) -> Vec<String> {
    let execution = ExecutionPlanBuilder::build(plan).expect("plan must build");
    let manifest = ExecutionManifestBuilder::build(&execution).expect("manifest must build");
    let mut lines = Vec::new();
    lines.push(Esp32Codec::encode_manifest_full(
        manifest.metadata.dof_count,
        manifest.metadata.total_samples,
        manifest.metadata.duration_us,
        64,
        1,
    ));
    for (i, seg) in manifest.segments.iter().enumerate() {
        let inst = match seg.instruction {
            thalos_runtime::execution_boundary::ManifestInstruction::MoveJ => "movej",
            thalos_runtime::execution_boundary::ManifestInstruction::MoveL => "movel",
        };
        lines.push(Esp32Codec::encode_segment(i, inst, seg.sample_start, seg.sample_count));
    }
    for wp in &manifest.samples {
        lines.push(Esp32Codec::encode_sample(&wp.joints, wp.dt_us));
    }
    lines.push(Esp32Codec::encode_end_upload());
    lines
}



// ---------------------------------------------------------------------------
// Spec scenario: integration_compiled_plan_to_wire_output
// ---------------------------------------------------------------------------

/// The full chain `CompiledPlan → ExecutionPlan → ExecutionManifest →
/// encode_manifest` MUST emit the exact wire lines: `MANIFEST`, one `SEGMENT`
/// per planned segment (with `movej`/`movel`), one `SAMPLE` per waypoint with
/// delta `dt_us` (first `0`), and `END_UPLOAD`.
#[test]
fn integration_compiled_plan_to_wire_output() {
    let plan = movej_then_movel_plan();
    assert_eq!(plan.segments.len(), 2);
    assert_eq!(plan.waypoint_count, 3);

    let lines = wire_lines(&plan);

    // MANIFEST + 2 SEGMENT + 3 SAMPLE + END_UPLOAD = 7 lines.
    assert_eq!(lines.len(), 7);

    // MANIFEST <dof> <N> <duration_us> — duration 1.5 s → 1_500_000 µs.
    assert_eq!(lines[0], "MANIFEST 2 3 1500000 64 1\n");

    // Segment provenance 1:1, MoveJ then MoveL.
    assert_eq!(lines[1], "SEGMENT 0 movej 0 2\n");
    assert_eq!(lines[2], "SEGMENT 1 movel 2 1\n");

    // Non-uniform timestamps → non-uniform delta dt_us, first sample dt = 0.
    assert_eq!(lines[3], "SAMPLE 0.000000 0.000000 0\n");
    assert_eq!(lines[4], "SAMPLE 0.500000 0.300000 500000\n");
    assert_eq!(lines[5], "SAMPLE 1.000000 0.600000 1000000\n");

    assert_eq!(lines[6], "END_UPLOAD\n");
}

/// Triangulation: for an evenly spaced single-segment plan the chain MUST emit
/// the same wire shape the legacy `build_manifest` produced — uniform
/// `dt_us = duration_us / (N-1)`, one `SEGMENT 0 movej` covering all samples.
#[test]
fn integration_evenly_spaced_movej_wire_matches_legacy_format() {
    let plan = evenly_spaced_movej_plan();
    assert_eq!(plan.segments.len(), 1);
    assert_eq!(plan.waypoint_count, 4);

    let lines = wire_lines(&plan);

    // MANIFEST + 1 SEGMENT + 4 SAMPLE + END_UPLOAD = 7 lines.
    assert_eq!(lines.len(), 7);
    assert_eq!(lines[0], "MANIFEST 2 4 1500000 64 1\n");
    assert_eq!(lines[1], "SEGMENT 0 movej 0 4\n");

    // dt evenly spaced: 1_500_000 / 3 = 500_000 per sample gap, first = 0.
    assert_eq!(lines[2], "SAMPLE 0.000000 0.000000 0\n");
    assert_eq!(lines[3], "SAMPLE 0.500000 0.300000 500000\n");
    assert_eq!(lines[4], "SAMPLE 1.000000 0.600000 500000\n");
    assert_eq!(lines[5], "SAMPLE 1.500000 0.900000 500000\n");

    assert_eq!(lines[6], "END_UPLOAD\n");
}
