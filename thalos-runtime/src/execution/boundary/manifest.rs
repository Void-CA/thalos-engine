//! Pure domain types for execution manifests.
//!
//! An `ExecutionManifest` is a flat sequence of timed waypoints organized into
//! segments, ready for upload to a hardware backend (e.g. ESP32). The types in
//! this module have no I/O, no serialization, and no protocol dependencies.
//!
//! # Ownership
//!
//! These types live in `execution_boundary` so that all plan-to-hardware
//! translation shares the same boundary module.

/// Type of motion for a segment of an execution manifest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ManifestInstruction {
    /// Joint-space movement (PTP / movej).
    MoveJ,
    /// Cartesian-space linear movement (movel).
    MoveL,
}

/// A single timed waypoint in an execution manifest.
///
/// Uses delta timing (`dt_us`) instead of absolute timestamps so the
/// firmware-side loop is trivial: set joints, wait `dt_us`, repeat.
#[derive(Debug, Clone, PartialEq)]
pub struct TimedWaypoint {
    /// Joint positions in radians.
    pub joints: Vec<f64>,
    /// Microseconds since the previous waypoint (delta, not cumulative).
    ///
    /// The first waypoint in the manifest always has `dt_us = 0`.
    pub dt_us: u32,
}

/// A segment of an execution manifest, mapping back to a motion segment of
/// the plan that produced it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ManifestSegment {
    /// Index of this segment in the original plan's segment list.
    pub index: usize,
    /// Type of motion for this segment.
    pub instruction: ManifestInstruction,
    /// Index of the first sample in the flat `samples` array.
    pub sample_start: usize,
    /// Number of samples belonging to this segment.
    ///
    /// Invariant: `sample_start + sample_count <= samples.len()`
    pub sample_count: usize,
}

/// Metadata describing an execution manifest.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ManifestMetadata {
    /// Number of joints (degrees of freedom).
    pub dof_count: usize,
    /// Total number of samples across all segments.
    pub total_samples: usize,
    /// Total execution duration in microseconds.
    pub duration_us: u64,
    /// Firmware-side repeat count (v3): the ESP32 executor loops the
    /// trajectory `repeat_count` times back-to-back with NO re-upload between
    /// passes. Default 1 = single pass (v2-compatible wire).
    pub repeat_count: u32,
}

/// A complete execution manifest ready for upload to a hardware backend.
///
/// Contains a flat array of `TimedWaypoint` samples organized into
/// segments, with metadata describing the overall execution.
///
/// # Invariants (guaranteed after construction by `ManifestBuilder`)
///
/// - `samples.len() == metadata.total_samples`
/// - Segments are in strictly ascending index order
/// - No segment overlap: `segments[i].sample_start + segments[i].sample_count`
///   <= `segments[i+1].sample_start`
/// - The last segment covers all samples:
///   `segments.last().sample_start + segments.last().sample_count == total_samples`
#[derive(Debug, Clone, PartialEq)]
pub struct ExecutionManifest {
    pub metadata: ManifestMetadata,
    pub segments: Vec<ManifestSegment>,
    pub samples: Vec<TimedWaypoint>,
}
