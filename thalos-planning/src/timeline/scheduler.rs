//! `TimelineScheduler` — the formal logical → temporal event transformation.
//!
//! `CompiledPlan` is the owner of physical time: a compiled trajectory exists,
//! so real durations exist. The resolver and everything before `PlanCompiler`
//! cannot know real durations (MoveJ/MoveL actual duration, blending,
//! optimizations, speed limits) — absolute time MUST NOT be assigned before
//! compilation.
//!
//! This scheduler is a dedicated, named transformation step: it converts
//! *logical events* (from `MotionResolver`, `at_time == 0`) into *temporal
//! events* (aligned to the compiled trajectory's absolute timeline). It walks
//! the IR-1 instruction stream alongside `CompiledPlan` segment timing
//! (`PlannedSegment.time_range`) and produces a `RuntimeProgram` whose events
//! carry absolute `at_time` (t=0 = plan start).
//!
//! # Cursor semantics
//!
//! A single timeline cursor starts at zero and advances by:
//!
//! - **Motion instructions** (`MoveJ`/`MoveL`): the duration of the
//!   corresponding `PlannedSegment` (`time_range.end - time_range.start`).
//!   Delays are *not* part of the compiled trajectory, so segment durations
//!   are added, never assigned from the segment's own end time — otherwise a
//!   delay would silently shift subsequent absolute times.
//! - **`Delay` events**: the delay's own duration is added to the cursor
//!   after the event is stamped with the current cursor (the delay *starts*
//!   at its `at_time`).
//! - **`SetOutput` events**: stamped with the current cursor; no time added.
//!
//! Events therefore carry absolute times from plan start, independent of
//! segment ordering (spec: RuntimeEvent Absolute Timestamp).

use std::time::Duration;

use thalos_core::{
    execution::program::ExecutionProgram,
    execution::runtime::{RuntimeAction, RuntimeEvent, RuntimeProgram},
};

use crate::motion::program::CompiledPlan;

/// Assigns absolute `at_time` to logical runtime events using the compiled
/// plan's segment timing.
#[derive(Debug, Default)]
pub struct TimelineScheduler;

impl TimelineScheduler {
    pub fn new() -> Self {
        Self
    }

    /// Convert logical events into temporal events.
    ///
    /// Walks the instruction stream alongside the `CompiledPlan` segment
    /// timing. Each `MoveJ`/`MoveL` instruction consumes the next segment and
    /// advances the timeline cursor by that segment's duration; each
    /// `Delay`/`SetOutput` instruction consumes the next logical event and
    /// stamps it with the current cursor (`Delay` also adds its duration to
    /// the cursor).
    ///
    /// The output events are sorted by `at_time` (monotonic cursor ⇒ already
    /// ordered; `RuntimeProgram::new` re-asserts the invariant).
    pub fn schedule(
        &self,
        program: &ExecutionProgram,
        compiled: &CompiledPlan,
        logical: RuntimeProgram,
    ) -> RuntimeProgram {
        let mut cursor = Duration::ZERO;
        let mut segment_idx = 0;
        let mut event_idx = 0;
        let mut temporal: Vec<RuntimeEvent> = Vec::with_capacity(logical.events.len());

        for instruction in &program.instructions {
            match instruction {
                thalos_core::execution::program::ExecutionInstruction::MoveJ { .. }
                | thalos_core::execution::program::ExecutionInstruction::MoveL { .. } => {
                    if let Some(segment) = compiled.segments.get(segment_idx) {
                        let duration = segment.time_range.end - segment.time_range.start;
                        cursor += Duration::from_secs_f64(duration.max(0.0));
                        segment_idx += 1;
                    }
                }
                thalos_core::execution::program::ExecutionInstruction::Delay { .. }
                | thalos_core::execution::program::ExecutionInstruction::SetOutput { .. } => {
                    if let Some(event) = logical.events.get(event_idx) {
                        let mut stamped = event.clone();
                        stamped.at_time = cursor;
                        if let RuntimeAction::Delay(d) = &stamped.action {
                            cursor += *d;
                        }
                        temporal.push(stamped);
                        event_idx += 1;
                    }
                }
            }
        }

        RuntimeProgram::new(temporal)
    }
}
