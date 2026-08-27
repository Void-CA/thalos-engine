//! Semantic command language over [`PlanningProgram`] (design D1, D2).
//!
//! A `ProgramEdit` is a corrections command: it transforms a motion program
//! via explicit operations (replace, insert, remove, split, merge, move
//! waypoint). `apply` is non-mutating — it returns a NEW program — and
//! `inverse` produces the edit that restores the original program (roundtrip
//! contract, spec "inverse Operation").

use serde::{Deserialize, Serialize};
use thalos_core::motion::segment::MotionSegment;

use crate::motion::program::PlanningProgram;

/// Errors returned by [`ProgramEdit::apply`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum EditError {
    /// An index fell outside `0..segments.len()`.
    #[error("index {index} out of bounds (program has {len} segments)")]
    IndexOutOfBounds { index: usize, len: usize },
    /// The edit describes an invalid range (empty insert/remove, empty split
    /// point, non-adjacent merge, or a count that cannot fit).
    #[error("invalid range: {message}")]
    InvalidRange { message: String },
    /// The edit targets a segment of the wrong kind (e.g. `SplitMove` on a
    /// `MoveL`, or a merge of two different kinds).
    #[error("segment {index} is {actual}; expected {expected}")]
    WrongSegmentKind {
        index: usize,
        expected: &'static str,
        actual: &'static str,
    },
}

/// Semantic command over [`PlanningProgram`] (design D1).
///
/// Six variants, exhaustive-match friendly. `apply` NEVER mutates the input —
/// it returns a new program. Variants that destroy information carry capture
/// fields (`original`, `removed`, `originals`, `old_target`) so `inverse` can
/// restore the exact original program (spec "inverse Operation").
///
/// Forward application is valid with capture fields unset; the roundtrip
/// guarantee of `inverse()` holds when the captures are populated (as the
/// apply pipeline does, design D6).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ProgramEdit {
    /// Replace the segment range at `index` — spanning `original.len()`
    /// segments (1 when the capture is unset) — with `replacement` segments.
    /// The replacement may be one-to-one, one-to-many, or one-to-zero.
    /// `original` captures the FULL pre-apply range: a single segment in
    /// forward edits (materializers replace one target), the whole replaced
    /// range in the roundtrip inverse — so `inverse()` restores the exact
    /// original program (R3-002, spec "inverse Operation").
    ReplaceSegment {
        index: usize,
        replacement: Vec<MotionSegment>,
        original: Option<Vec<MotionSegment>>,
    },
    /// Insert `segments` at position `at` (boundary `at == len` appends).
    /// At least one segment is required.
    InsertSegments {
        at: usize,
        segments: Vec<MotionSegment>,
    },
    /// Remove `count` segments starting at `at`. `removed` captures the
    /// removed segments for the roundtrip inverse. `count == 0` is rejected.
    RemoveSegments {
        at: usize,
        count: usize,
        removed: Option<Vec<MotionSegment>>,
    },
    /// Split the MoveJ at `index` into two segments at waypoint `point`: the
    /// first keeps the original origin/limits and targets `point`, the second
    /// keeps the original target.
    SplitMove { index: usize, point: Vec<f64> },
    /// Merge adjacent segments `first` and `second == first + 1` into one:
    /// the merged segment keeps the first's origin/limits and the second's
    /// target (reverses a `SplitMove`). `originals` captures both segments
    /// for the roundtrip inverse.
    MergeMoves {
        first: usize,
        second: usize,
        originals: Option<(MotionSegment, MotionSegment)>,
    },
    /// Move the MoveJ target of `segment_index` to `new_target`. `old_target`
    /// captures the previous target for the roundtrip inverse.
    MoveWaypoint {
        segment_index: usize,
        new_target: Vec<f64>,
        old_target: Option<Vec<f64>>,
    },
}

impl ProgramEdit {
    /// Apply the edit to `program`, returning a NEW program — the input is
    /// never modified. Bounds are validated before any mutation.
    pub fn apply(&self, program: &PlanningProgram) -> Result<PlanningProgram, EditError> {
        match self {
            ProgramEdit::ReplaceSegment {
                index, replacement, original,
            } => {
                Self::check_index(program, *index)?;
                // The edit replaces `original.len()` segments (default 1 when
                // the capture is unset). Forward materializers replace ONE
                // target segment; the roundtrip inverse collapses the whole
                // replaced range back (R3-002).
                let replaced_len = original.as_ref().map_or(1, |o| o.len().max(1));
                let end = index.checked_add(replaced_len - 1).ok_or_else(|| {
                    EditError::InvalidRange {
                        message: "segment index overflow".to_string(),
                    }
                })?;
                if end >= program.segments.len() {
                    return Err(EditError::IndexOutOfBounds {
                        index: end,
                        len: program.segments.len(),
                    });
                }
                let mut next = program.segments.clone();
                let _ = next.splice(*index..=end, replacement.clone());
                Ok(PlanningProgram::new(next))
            }
            ProgramEdit::InsertSegments { at, segments } => {
                if segments.is_empty() {
                    return Err(EditError::InvalidRange {
                        message: "inserting zero segments".to_string(),
                    });
                }
                if *at > program.segments.len() {
                    return Err(EditError::IndexOutOfBounds {
                        index: *at,
                        len: program.segments.len(),
                    });
                }
                let mut next = program.segments.clone();
                let mut tail = next.split_off(*at);
                next.extend(segments.iter().cloned());
                next.append(&mut tail);
                Ok(PlanningProgram::new(next))
            }
            ProgramEdit::RemoveSegments { at, count, .. } => {
                if *count == 0 {
                    return Err(EditError::InvalidRange {
                        message: "removing zero segments".to_string(),
                    });
                }
                let end = at
                    .checked_add(*count)
                    .ok_or_else(|| EditError::InvalidRange {
                        message: "segment count overflow".to_string(),
                    })?;
                if end > program.segments.len() {
                    return Err(EditError::IndexOutOfBounds {
                        index: end,
                        len: program.segments.len(),
                    });
                }
                let mut next = program.segments.clone();
                next.drain(*at..end);
                Ok(PlanningProgram::new(next))
            }
            ProgramEdit::SplitMove { index, point } => {
                Self::check_index(program, *index)?;
                if point.is_empty() {
                    return Err(EditError::InvalidRange {
                        message: "split point must name at least one joint".to_string(),
                    });
                }
                let (head, tail) = split_move(&program.segments[*index], point).ok_or(
                    EditError::WrongSegmentKind {
                        index: *index,
                        expected: "MoveJ",
                        actual: "MoveL",
                    },
                )?;
                let mut next = program.segments.clone();
                let _ = next.splice(*index..=*index, [head, tail]);
                Ok(PlanningProgram::new(next))
            }
            ProgramEdit::MergeMoves { first, second, .. } => {
                if second != &(first + 1) {
                    return Err(EditError::InvalidRange {
                        message: format!(
                            "merge targets {first} and {second}, which are not adjacent"
                        ),
                    });
                }
                Self::check_index(program, *second)?;
                let merged = merge_moves(&program.segments[*first], &program.segments[*second])
                    .ok_or(EditError::WrongSegmentKind {
                        index: *first,
                        expected: "two segments of the same kind",
                        actual: "segments of different kinds",
                    })?;
                let mut next = program.segments.clone();
                let _ = next.splice(*first..=*second, [merged]);
                Ok(PlanningProgram::new(next))
            }
            ProgramEdit::MoveWaypoint {
                segment_index,
                new_target,
                ..
            } => {
                Self::check_index(program, *segment_index)?;
                if new_target.is_empty() {
                    return Err(EditError::InvalidRange {
                        message: "target must name at least one joint".to_string(),
                    });
                }
                let moved = move_waypoint(&program.segments[*segment_index], new_target).ok_or(
                    EditError::WrongSegmentKind {
                        index: *segment_index,
                        expected: "MoveJ",
                        actual: "MoveL",
                    },
                )?;
                let mut next = program.segments.clone();
                next[*segment_index] = moved;
                Ok(PlanningProgram::new(next))
            }
        }
    }

    /// The edit that restores the program produced by `self.apply(program)`.
    ///
    /// Exact for edits with capture fields populated (the roundtrip contract,
    /// spec "inverse Operation" + property test). For uncaptured
    /// `RemoveSegments` / `MergeMoves` the inverse is best-effort and its
    /// `apply` fails loudly (empty insert / empty split point) instead of
    /// corrupting the program.
    pub fn inverse(&self) -> ProgramEdit {
        match self {
            ProgramEdit::ReplaceSegment {
                index,
                replacement,
                original,
            } => ProgramEdit::ReplaceSegment {
                index: *index,
                replacement: original.clone().unwrap_or_default(),
                original: Some(replacement.clone()),
            },
            ProgramEdit::InsertSegments { at, segments } => ProgramEdit::RemoveSegments {
                at: *at,
                count: segments.len(),
                removed: Some(segments.clone()),
            },
            ProgramEdit::RemoveSegments { at, removed, .. } => ProgramEdit::InsertSegments {
                at: *at,
                segments: removed.clone().unwrap_or_default(),
            },
            ProgramEdit::SplitMove { index, .. } => ProgramEdit::MergeMoves {
                first: *index,
                second: index + 1,
                originals: None,
            },
            ProgramEdit::MergeMoves {
                first, originals, ..
            } => match originals {
                Some((head, tail)) => ProgramEdit::ReplaceSegment {
                    index: *first,
                    replacement: vec![head.clone(), tail.clone()],
                    original: None,
                },
                None => ProgramEdit::SplitMove {
                    index: *first,
                    point: Vec::new(),
                },
            },
            ProgramEdit::MoveWaypoint {
                segment_index,
                new_target,
                old_target,
            } => ProgramEdit::MoveWaypoint {
                segment_index: *segment_index,
                new_target: old_target.clone().unwrap_or_default(),
                old_target: Some(new_target.clone()),
            },
        }
    }

    fn check_index(program: &PlanningProgram, index: usize) -> Result<(), EditError> {
        if index < program.segments.len() {
            Ok(())
        } else {
            Err(EditError::IndexOutOfBounds {
                index,
                len: program.segments.len(),
            })
        }
    }
}

/// Split a MoveJ into two: the first targets `point`, the second keeps the
/// original target. Both halves inherit origin and motion limits.
fn split_move(segment: &MotionSegment, point: &[f64]) -> Option<(MotionSegment, MotionSegment)> {
    match segment {
        MotionSegment::MoveJ {
            origin,
            target,
            max_velocity,
            max_acceleration,
        } => Some((
            MotionSegment::MoveJ {
                origin: origin.clone(),
                target: point.to_vec(),
                max_velocity: *max_velocity,
                max_acceleration: *max_acceleration,
            },
            MotionSegment::MoveJ {
                origin: origin.clone(),
                target: target.clone(),
                max_velocity: *max_velocity,
                max_acceleration: *max_acceleration,
            },
        )),
        MotionSegment::MoveL { .. } => None,
        MotionSegment::MoveLPosition { .. } => None,
    }
}

/// Merge two same-kind adjacent segments: the result keeps the first's origin
/// and limits and the second's target (the exact inverse of [`split_move`]).
fn merge_moves(first: &MotionSegment, second: &MotionSegment) -> Option<MotionSegment> {
    match (first, second) {
        (
            MotionSegment::MoveJ {
                origin,
                max_velocity,
                max_acceleration,
                ..
            },
            MotionSegment::MoveJ { target, .. },
        ) => Some(MotionSegment::MoveJ {
            origin: origin.clone(),
            target: target.clone(),
            max_velocity: *max_velocity,
            max_acceleration: *max_acceleration,
        }),
        (
            MotionSegment::MoveL {
                origin,
                frame,
                max_velocity,
                ..
            },
            MotionSegment::MoveL { target_pose, .. },
        ) => Some(MotionSegment::MoveL {
            origin: origin.clone(),
            frame: *frame,
            target_pose: target_pose.clone(),
            max_velocity: *max_velocity,
        }),
        _ => None,
    }
}

/// Retarget a MoveJ to `new_target`, preserving origin and motion limits.
fn move_waypoint(segment: &MotionSegment, new_target: &[f64]) -> Option<MotionSegment> {
    match segment {
        MotionSegment::MoveJ {
            origin,
            max_velocity,
            max_acceleration,
            ..
        } => Some(MotionSegment::MoveJ {
            origin: origin.clone(),
            target: new_target.to_vec(),
            max_velocity: *max_velocity,
            max_acceleration: *max_acceleration,
        }),
        MotionSegment::MoveL { .. } => None,
        MotionSegment::MoveLPosition { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use thalos_core::ids::OperationId;
    use thalos_core::motion::segment::MotionSegment;
    use thalos_core::spatial::frame::FrameId;
    use thalos_core::spatial::pose::Pose;
    use thalos_math::Transform3D;

    use super::{EditError, ProgramEdit};
    use crate::motion::program::PlanningProgram;

    fn move_j(target: Vec<f64>) -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId("op-j".to_string()),
            target,
            max_velocity: Some(500.0),
            max_acceleration: Some(1000.0),
        }
    }

    fn move_l() -> MotionSegment {
        MotionSegment::MoveL {
            origin: OperationId("op-l".to_string()),
            frame: FrameId::World,
            target_pose: Pose::new(FrameId::World, FrameId::Id(1), Transform3D::identity()),
            max_velocity: Some(200.0),
        }
    }

    /// Five segments: MoveJ, MoveL, MoveJ, MoveL, MoveJ (spec "5 segments").
    fn five_segment_program() -> PlanningProgram {
        PlanningProgram::new(vec![
            move_j(vec![1.0, 1.0]),
            move_l(),
            move_j(vec![2.0, 2.0]),
            move_l(),
            move_j(vec![3.0, 3.0]),
        ])
    }

    #[test]
    fn replace_segment_in_bounds_replaces_and_keeps_length() {
        // Spec program-edit "ReplaceSegment in bounds": 5 segments, index 2,
        // replacement → new program has segment at index 2 replaced and the
        // same length.
        let p = five_segment_program();
        let replacement = move_j(vec![9.0, 9.0]);
        let edit = ProgramEdit::ReplaceSegment {
            index: 2,
            replacement: vec![replacement.clone()],
            original: Some(vec![p.segments[2].clone()]),
        };

        let result = edit.apply(&p).expect("in-bounds replace must succeed");

        assert_eq!(result.segments.len(), 5, "length must stay unchanged");
        assert_eq!(
            result.segments[2], replacement,
            "segment 2 must be replaced"
        );
        // Neighbours are untouched — only index 2 changed.
        assert_eq!(result.segments[0], p.segments[0]);
        assert_eq!(result.segments[1], p.segments[1]);
        assert_eq!(result.segments[4], p.segments[4]);
        // The input program is NOT mutated (apply returns a new instance).
        assert_eq!(p.segments[2].origin(), &OperationId("op-j".to_string()));
    }

    #[test]
    fn replace_segment_out_of_bounds_returns_error_and_program_unchanged() {
        // Spec program-edit "Index out of bounds": index 10 of a 5-segment
        // program → EditError::IndexOutOfBounds, program unchanged.
        let p = five_segment_program();
        let edit = ProgramEdit::ReplaceSegment {
            index: 10,
            replacement: vec![move_j(vec![9.0, 9.0])],
            original: None,
        };

        let err = edit.apply(&p).expect_err("index 10 must be rejected");

        assert!(
            matches!(err, EditError::IndexOutOfBounds { index: 10, .. }),
            "expected IndexOutOfBounds, got {err:?}"
        );
        assert_eq!(p.segments.len(), 5, "input program must remain unchanged");
    }

    #[test]
    fn insert_segments_at_end_boundary_appends() {
        // Spec program-edit "InsertSegments at boundary": 5 segments,
        // at: 5 (== len) → segments appended at the end.
        let p = five_segment_program();
        let to_insert = vec![move_j(vec![7.0, 7.0]), move_j(vec![8.0, 8.0])];
        let edit = ProgramEdit::InsertSegments {
            at: 5,
            segments: to_insert.clone(),
        };

        let result = edit.apply(&p).expect("boundary insert must succeed");

        assert_eq!(result.segments.len(), 7, "two segments appended");
        assert_eq!(
            result.segments[5], to_insert[0],
            "first inserted at index 5"
        );
        assert_eq!(
            result.segments[6], to_insert[1],
            "second inserted at index 6"
        );
        // Original five segments are preserved in order.
        assert_eq!(result.segments[..5], p.segments[..]);
    }

    #[test]
    fn remove_segments_empty_range_returns_invalid_range() {
        // Spec program-edit "RemoveSegments empty range": at: 2, count: 0 →
        // EditError::InvalidRange.
        let p = five_segment_program();
        let edit = ProgramEdit::RemoveSegments {
            at: 2,
            count: 0,
            removed: None,
        };

        let err = edit.apply(&p).expect_err("empty removal must be rejected");

        assert!(
            matches!(err, EditError::InvalidRange { .. }),
            "expected InvalidRange, got {err:?}"
        );
        assert_eq!(p.segments.len(), 5, "input program must remain unchanged");
    }

    // ── Forward behavior of the remaining variants (triangulation) ──────────

    #[test]
    fn insert_segments_in_middle_inserts_at_position() {
        let p = five_segment_program();
        let edit = ProgramEdit::InsertSegments {
            at: 2,
            segments: vec![move_j(vec![7.0, 7.0])],
        };

        let result = edit.apply(&p).expect("mid insert must succeed");

        assert_eq!(result.segments.len(), 6);
        assert_eq!(result.segments[2], move_j(vec![7.0, 7.0]));
        assert_eq!(
            result.segments[..2],
            p.segments[..2],
            "head order preserved"
        );
        assert_eq!(
            result.segments[3..],
            p.segments[2..],
            "tail order preserved"
        );
    }

    #[test]
    fn remove_segments_in_bounds_removes_exact_count() {
        let p = five_segment_program();
        let edit = ProgramEdit::RemoveSegments {
            at: 1,
            count: 2,
            removed: Some(p.segments[1..3].to_vec()),
        };

        let result = edit.apply(&p).expect("in-bounds removal must succeed");

        assert_eq!(result.segments.len(), 3);
        assert_eq!(result.segments[0], p.segments[0]);
        assert_eq!(
            result.segments[1], p.segments[3],
            "segment 3 slides to index 1"
        );
        assert_eq!(result.segments[2], p.segments[4]);
    }

    #[test]
    fn remove_segments_past_the_end_returns_index_out_of_bounds() {
        let p = five_segment_program();
        let edit = ProgramEdit::RemoveSegments {
            at: 4,
            count: 2,
            removed: None,
        };

        let err = edit
            .apply(&p)
            .expect_err("removal past the end must be rejected");
        assert!(matches!(err, EditError::IndexOutOfBounds { .. }));
    }

    #[test]
    fn split_move_splits_one_movej_into_two() {
        let p = five_segment_program();
        let edit = ProgramEdit::SplitMove {
            index: 0,
            point: vec![1.5, 1.5],
        };

        let result = edit.apply(&p).expect("split must succeed");

        assert_eq!(result.segments.len(), 6, "one segment becomes two");
        match (&result.segments[0], &result.segments[1]) {
            (
                MotionSegment::MoveJ {
                    origin,
                    target,
                    max_velocity,
                    ..
                },
                MotionSegment::MoveJ { target: tail, .. },
            ) => {
                assert_eq!(target, &vec![1.5, 1.5], "head targets the split point");
                assert_eq!(tail, &vec![1.0, 1.0], "tail keeps the original target");
                assert_eq!(origin, &OperationId("op-j".to_string()), "origin preserved");
                assert_eq!(*max_velocity, Some(500.0), "limits preserved");
            }
            _ => panic!("split must produce MoveJ segments"),
        }
        assert_eq!(
            result.segments[2..],
            p.segments[1..],
            "rest of the program untouched"
        );
    }

    #[test]
    fn split_move_on_movel_returns_wrong_segment_kind() {
        let p = five_segment_program();
        let edit = ProgramEdit::SplitMove {
            index: 1, // MoveL
            point: vec![1.5, 1.5],
        };

        let err = edit
            .apply(&p)
            .expect_err("MoveL cannot be split at a joint point");
        assert!(matches!(
            err,
            EditError::WrongSegmentKind {
                index: 1,
                expected: "MoveJ",
                actual: "MoveL"
            }
        ));
    }

    #[test]
    fn merge_moves_merges_adjacent_into_one() {
        // Two adjacent MoveJ segments — the mergeable pair.
        let p = PlanningProgram::new(vec![
            move_j(vec![1.0, 1.0]),
            move_j(vec![2.0, 2.0]),
            move_l(),
        ]);
        let edit = ProgramEdit::MergeMoves {
            first: 0,
            second: 1,
            originals: Some((p.segments[0].clone(), p.segments[1].clone())),
        };

        let result = edit.apply(&p).expect("merge must succeed");

        assert_eq!(result.segments.len(), 2, "two segments become one");
        assert_eq!(
            result.segments[0].origin(),
            &OperationId("op-j".to_string())
        );
        assert_eq!(
            result.segments[0],
            move_j(vec![2.0, 2.0]),
            "merged segment keeps first's origin and second's target"
        );
        assert_eq!(result.segments[1], p.segments[2]);
    }

    #[test]
    fn merge_moves_non_adjacent_returns_invalid_range() {
        let p = five_segment_program();
        let edit = ProgramEdit::MergeMoves {
            first: 0,
            second: 2,
            originals: None,
        };

        let err = edit
            .apply(&p)
            .expect_err("non-adjacent merge must be rejected");
        assert!(matches!(err, EditError::InvalidRange { .. }));
    }

    #[test]
    fn merge_moves_different_kinds_returns_wrong_segment_kind() {
        let p = five_segment_program();
        let edit = ProgramEdit::MergeMoves {
            first: 0,  // MoveJ
            second: 1, // MoveL
            originals: None,
        };

        let err = edit.apply(&p).expect_err("kind mismatch must be rejected");
        assert!(matches!(err, EditError::WrongSegmentKind { index: 0, .. }));
    }

    #[test]
    fn move_waypoint_updates_target_preserving_origin() {
        let p = five_segment_program();
        let edit = ProgramEdit::MoveWaypoint {
            segment_index: 0,
            new_target: vec![4.0, 4.0],
            old_target: Some(vec![1.0, 1.0]),
        };

        let result = edit.apply(&p).expect("move waypoint must succeed");

        assert_eq!(result.segments[0], move_j(vec![4.0, 4.0]));
        assert_eq!(
            result.segments[1..],
            p.segments[1..],
            "other segments untouched"
        );
    }

    #[test]
    fn move_waypoint_on_movel_returns_wrong_segment_kind() {
        let p = five_segment_program();
        let edit = ProgramEdit::MoveWaypoint {
            segment_index: 1, // MoveL
            new_target: vec![4.0, 4.0],
            old_target: None,
        };

        let err = edit
            .apply(&p)
            .expect_err("MoveL has no joint-space target to move");
        assert!(matches!(
            err,
            EditError::WrongSegmentKind {
                index: 1,
                expected: "MoveJ",
                actual: "MoveL"
            }
        ));
    }

    // ── Spec inverse scenarios (spec program-edit "inverse Operation") ─────

    #[test]
    fn replace_segment_one_to_many_inverse_restores_original_program() {
        // R3-002: a 1→N ReplaceSegment (the InsertWaypoint materializer
        // produces TWO segments) must round-trip exactly — the inverse must
        // restore the FULL replaced range, not just the first segment.
        let p = five_segment_program();
        let edit = ProgramEdit::ReplaceSegment {
            index: 2,
            replacement: vec![move_j(vec![9.0, 9.0]), move_j(vec![10.0, 10.0])],
            original: Some(vec![p.segments[2].clone()]),
        };

        let p_prime = edit.apply(&p).expect("apply");
        assert_eq!(p_prime.segments.len(), 6, "one segment becomes two");

        let restored = edit.inverse().apply(&p_prime).expect("inverse apply");
        assert_eq!(
            restored, p,
            "undo of a 1→2 replace must restore the exact original program"
        );
    }

    #[test]
    fn replace_segment_inverse_restores_original_program() {
        // Spec "ReplaceSegment inverse": inverse applied to P' equals P.
        let p = five_segment_program();
        let edit = ProgramEdit::ReplaceSegment {
            index: 2,
            replacement: vec![move_j(vec![9.0, 9.0])],
            original: Some(vec![p.segments[2].clone()]),
        };

        let p_prime = edit.apply(&p).expect("apply");
        let restored = edit.inverse().apply(&p_prime).expect("inverse apply");

        assert_eq!(restored, p);
    }

    #[test]
    fn insert_segments_inverse_is_remove_segments() {
        // Spec "InsertSegments inverse": inverse (RemoveSegments { at, count })
        // applied to P' equals P.
        let p = five_segment_program();
        let to_insert = vec![move_j(vec![7.0, 7.0]), move_j(vec![8.0, 8.0])];
        let edit = ProgramEdit::InsertSegments {
            at: 2,
            segments: to_insert.clone(),
        };

        assert!(matches!(
            edit.inverse(),
            ProgramEdit::RemoveSegments {
                at: 2,
                count: 2,
                removed: Some(_)
            }
        ));

        let p_prime = edit.apply(&p).expect("apply");
        let restored = edit.inverse().apply(&p_prime).expect("inverse apply");
        assert_eq!(restored, p);
    }

    #[test]
    fn split_move_inverse_is_merge_moves() {
        // Spec "SplitMove inverse": inverse (MergeMoves { first: i, second: i+1 })
        // applied to P' equals P.
        let p = five_segment_program();
        let edit = ProgramEdit::SplitMove {
            index: 0,
            point: vec![1.5, 1.5],
        };

        assert!(matches!(
            edit.inverse(),
            ProgramEdit::MergeMoves {
                first: 0,
                second: 1,
                ..
            }
        ));

        let p_prime = edit.apply(&p).expect("apply");
        let restored = edit.inverse().apply(&p_prime).expect("inverse apply");
        assert_eq!(restored, p);
    }

    #[test]
    fn move_waypoint_inverse_restores_old_target() {
        // Spec "MoveWaypoint inverse": inverse (MoveWaypoint with old_position)
        // applied to P' equals P.
        let p = five_segment_program();
        let edit = ProgramEdit::MoveWaypoint {
            segment_index: 0,
            new_target: vec![4.0, 4.0],
            old_target: Some(vec![1.0, 1.0]),
        };

        let p_prime = edit.apply(&p).expect("apply");
        let restored = edit.inverse().apply(&p_prime).expect("inverse apply");
        assert_eq!(restored, p);
    }

    #[test]
    fn remove_segments_inverse_reinserts_captured_segments() {
        let p = five_segment_program();
        let edit = ProgramEdit::RemoveSegments {
            at: 1,
            count: 2,
            removed: Some(p.segments[1..3].to_vec()),
        };

        let p_prime = edit.apply(&p).expect("apply");
        let restored = edit.inverse().apply(&p_prime).expect("inverse apply");
        assert_eq!(restored, p);
    }

    #[test]
    fn merge_moves_inverse_restores_originals() {
        let p = PlanningProgram::new(vec![
            move_j(vec![1.0, 1.0]),
            move_j(vec![2.0, 2.0]),
            move_l(),
        ]);
        let edit = ProgramEdit::MergeMoves {
            first: 0,
            second: 1,
            originals: Some((p.segments[0].clone(), p.segments[1].clone())),
        };

        let p_prime = edit.apply(&p).expect("apply");
        let restored = edit.inverse().apply(&p_prime).expect("inverse apply");
        assert_eq!(restored, p);
    }

    #[test]
    fn program_edit_serde_round_trip() {
        // The enum crosses the wire in PR2 (recommendations[]) — serde must
        // survive a JSON round trip for every variant.
        let edits = vec![
            ProgramEdit::ReplaceSegment {
                index: 1,
                replacement: vec![move_j(vec![9.0, 9.0])],
                original: Some(vec![move_j(vec![1.0, 1.0])]),
            },
            ProgramEdit::InsertSegments {
                at: 2,
                segments: vec![move_l()],
            },
            ProgramEdit::RemoveSegments {
                at: 1,
                count: 2,
                removed: None,
            },
            ProgramEdit::SplitMove {
                index: 0,
                point: vec![1.5, 1.5],
            },
            ProgramEdit::MergeMoves {
                first: 0,
                second: 1,
                originals: None,
            },
            ProgramEdit::MoveWaypoint {
                segment_index: 0,
                new_target: vec![4.0, 4.0],
                old_target: Some(vec![1.0, 1.0]),
            },
        ];
        for edit in edits {
            let json = serde_json::to_string(&edit).expect("serialize");
            let back: ProgramEdit = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(back, edit, "serde round trip must preserve the edit");
        }
    }
}

/// Property-based roundtrip contract (spec "Property test — roundtrip"):
///
/// `inverse(E).apply(E.apply(P)) == P` for every variant, for arbitrary
/// in-bounds edits with capture fields populated. Runs in CI, never by
/// runtime asserts (D6).
#[cfg(test)]
mod property_tests {
    use proptest::prelude::*;

    use super::ProgramEdit;
    use crate::motion::program::PlanningProgram;
    use thalos_core::ids::OperationId;
    use thalos_core::motion::segment::MotionSegment;

    /// Finite joint values only — NaN would break structural equality and
    /// proptest's full-range `f64::ANY` can produce it.
    fn joints_strategy() -> impl Strategy<Value = Vec<f64>> {
        prop::collection::vec(-2.0f64..2.0, 2..=6)
    }

    fn move_j_strategy() -> impl Strategy<Value = MotionSegment> {
        joints_strategy().prop_map(|target| MotionSegment::MoveJ {
            origin: OperationId("prop-op".to_string()),
            target,
            max_velocity: Some(500.0),
            max_acceleration: Some(1000.0),
        })
    }

    /// MoveJ-only programs, 2..=8 segments — every index is valid for every
    /// variant (MoveL forward behavior is covered by the unit tests).
    fn program_strategy() -> impl Strategy<Value = PlanningProgram> {
        prop::collection::vec(move_j_strategy(), 2..=8).prop_map(PlanningProgram::new)
    }

    proptest! {
        // 128 cases per property (matches thalos-core aggregator precedent).
        #![proptest_config(proptest::test_runner::Config::with_cases(128))]

        #[test]
        fn replace_segment_roundtrip(
            p in program_strategy(),
            idx in 0usize..8,
            replacement in move_j_strategy(),
        ) {
            let idx = idx % p.segments.len();
            let edit = ProgramEdit::ReplaceSegment {
                index: idx,
                replacement: vec![replacement],
                original: Some(vec![p.segments[idx].clone()]),
            };
            let p_prime = edit.apply(&p).expect("in-bounds replace must succeed");
            let restored = edit.inverse().apply(&p_prime).expect("inverse must succeed");
            prop_assert_eq!(restored, p);
        }

        // R3-002: one-to-many replacements (1→2, 1→3 — the InsertWaypoint
        // materializer shape) must round-trip exactly. The inverse splices the
        // WHOLE replaced range back, not just the first segment.
        #[test]
        fn replace_segment_one_to_many_roundtrip(
            p in program_strategy(),
            idx in 0usize..8,
            replacement in prop::collection::vec(move_j_strategy(), 2..=3),
        ) {
            let idx = idx % p.segments.len();
            let edit = ProgramEdit::ReplaceSegment {
                index: idx,
                replacement,
                original: Some(vec![p.segments[idx].clone()]),
            };
            let p_prime = edit.apply(&p).expect("in-bounds replace must succeed");
            let restored = edit.inverse().apply(&p_prime).expect("inverse must succeed");
            prop_assert_eq!(restored, p);
        }

        #[test]
        fn insert_segments_roundtrip(
            p in program_strategy(),
            at in 0usize..8,
            segments in prop::collection::vec(move_j_strategy(), 1..=3),
        ) {
            let at = at % (p.segments.len() + 1);
            let edit = ProgramEdit::InsertSegments { at, segments };
            let p_prime = edit.apply(&p).expect("boundary insert must succeed");
            let restored = edit.inverse().apply(&p_prime).expect("inverse must succeed");
            prop_assert_eq!(restored, p);
        }

        #[test]
        fn remove_segments_roundtrip(
            p in program_strategy(),
            at in 0usize..8,
            count in 1usize..=8,
        ) {
            let at = at % p.segments.len();
            let count = count.min(p.segments.len() - at);
            let removed = p.segments[at..at + count].to_vec();
            let edit = ProgramEdit::RemoveSegments { at, count, removed: Some(removed) };
            let p_prime = edit.apply(&p).expect("in-bounds removal must succeed");
            let restored = edit.inverse().apply(&p_prime).expect("inverse must succeed");
            prop_assert_eq!(restored, p);
        }

        #[test]
        fn split_move_roundtrip(
            p in program_strategy(),
            idx in 0usize..8,
            point in joints_strategy(),
        ) {
            let idx = idx % p.segments.len();
            let edit = ProgramEdit::SplitMove { index: idx, point };
            let p_prime = edit.apply(&p).expect("split must succeed");
            let restored = edit.inverse().apply(&p_prime).expect("inverse must succeed");
            prop_assert_eq!(restored, p);
        }

        #[test]
        fn merge_moves_roundtrip(
            p in program_strategy(),
            first in 0usize..8,
        ) {
            let first = first % (p.segments.len() - 1);
            let originals = (p.segments[first].clone(), p.segments[first + 1].clone());
            let edit = ProgramEdit::MergeMoves { first, second: first + 1, originals: Some(originals) };
            let p_prime = edit.apply(&p).expect("merge must succeed");
            let restored = edit.inverse().apply(&p_prime).expect("inverse must succeed");
            prop_assert_eq!(restored, p);
        }

        #[test]
        fn move_waypoint_roundtrip(
            p in program_strategy(),
            seg_idx in 0usize..8,
            new_target in joints_strategy(),
        ) {
            let seg_idx = seg_idx % p.segments.len();
            let old_target = match &p.segments[seg_idx] {
                MotionSegment::MoveJ { target, .. } => target.clone(),
                MotionSegment::MoveL { .. } => unreachable!("property programs are MoveJ-only"),
                MotionSegment::MoveLPosition { .. } => {
                    unreachable!("property programs are MoveJ-only")
                }
            };
            let edit = ProgramEdit::MoveWaypoint {
                segment_index: seg_idx,
                new_target,
                old_target: Some(old_target),
            };
            let p_prime = edit.apply(&p).expect("move waypoint must succeed");
            let restored = edit.inverse().apply(&p_prime).expect("inverse must succeed");
            prop_assert_eq!(restored, p);
        }
    }
}
