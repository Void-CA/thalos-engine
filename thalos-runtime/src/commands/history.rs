//! Applied-command history with pre-computed inverses (design D6).
//!
//! PR5 implements `undo` as pop + apply(inverse) in O(1) — the inverse is
//! stored at apply time (PR4) and each entry captures the metrics of the
//! applied plan so the undo endpoint reports the restored health without
//! re-running the analysis pipeline.

use thalos_engine::planning::{
    motion::program::PlanningProgram,
    program_edit::{EditError, ProgramEdit},
};
use thalos_engine::core::motion::segment::MotionSegment;

/// Health metrics captured at apply time (D6) — the undo endpoint reports the
/// restored health from these without re-running the analysis pipeline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandMetrics {
    /// Health (0..1) of the plan BEFORE the command was applied.
    pub health_before: f64,
    /// Health (0..1) of the plan AFTER the command was applied.
    pub health_after: f64,
}

impl CommandMetrics {
    pub fn new(health_before: f64, health_after: f64) -> Self {
        Self {
            health_before,
            health_after,
        }
    }

    /// `health_after - health_before` — the delta the applied command produced.
    pub fn improvement(&self) -> f64 {
        self.health_after - self.health_before
    }
}

/// A command applied to the runtime, with its pre-computed inverse (D6).
///
/// PR4 stored the inverse in memory so PR5 can implement `undo` in O(1) via
/// `apply(inverse)` — no replay, no re-derivation. The history lives on
/// `SceneRuntime` close to the mutation surface (design open question:
/// runtime, not planning — it tracks scene mutations).
///
/// R4-001: each entry is LINKED to the program it produced (`applied_program`).
/// Undo must never apply an inverse to a plan that is not that command's
/// pre-state — `matches_applied_program` is the guard that rejects a stale
/// inverse when another path (e.g. a re-schedule) replaced the active plan.
#[derive(Debug, Clone, PartialEq)]
pub struct AppliedCommand {
    /// The semantic edit that was applied.
    pub command: ProgramEdit,
    /// The edit that restores the previous program (`command.inverse()`).
    pub inverse: ProgramEdit,
    /// Health metrics of the applied plan — undo reports from these (O(1)).
    pub metrics: CommandMetrics,
    /// The program segments the apply WROTE BACK — the only state this
    /// command's inverse may be applied to (R4-001 stale-undo guard).
    pub applied_program: Vec<MotionSegment>,
}

impl AppliedCommand {
    /// O(1) undo: apply the stored inverse to `program` in a SINGLE call —
    /// never a replay of the history (design D6, spec "Undo is O(1)").
    pub fn undo_program(&self, program: &PlanningProgram) -> Result<PlanningProgram, EditError> {
        self.inverse.apply(program)
    }

    /// True when `program` is the exact program this command produced (R4-001).
    ///
    /// Undo is only safe when the inverse is applied to its own pre-state; if
    /// another path replaced the active plan, the stored inverse is stale.
    pub fn matches_applied_program(&self, program: &PlanningProgram) -> bool {
        self.applied_program == program.segments
    }
}

/// O(1) command history: Vec-backed, push/pop on the tail (design D6).
///
/// `undo` pops the last entry in constant time — the history is never
/// replayed or re-derived.
///
/// PR2 versioned undo (spec command-endpoints "Undo version mismatch"): a
/// monotonic `version: u64` counter increments on EVERY mutation (push/pop/
/// clear). The undo flow reads it atomically with the last entry
/// (`last_with_version`) and re-validates it at commit time — closing the
/// TOCTOU window between the peek and the recompile.
/// Default maximum number of retained applied commands (spec command-endpoints
/// "History Cap"). Bounded memory: undo stays O(1) and the oldest entry is
/// discarded when the capacity is exceeded on push.
pub const DEFAULT_HISTORY_CAP: usize = 100;

/// Bounded, versioned undo history.
///
/// Entries are pre-computed inverses in apply order (design D6). The capacity
/// bounds memory (spec "History Cap") while keeping undo O(1) — overflow
/// discards the OLDEST entry (`remove(0)`, O(n) bounded by cap ≤ 100). The
/// monotonic version covers any mutation (push/pop/clear) for the PR2 TOCTOU
/// gate.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandHistory {
    entries: Vec<AppliedCommand>,
    /// Monotonic mutation counter — bumped on push/pop/clear.
    version: u64,
    /// Maximum number of retained entries (spec "History Cap").
    cap: usize,
}

impl Default for CommandHistory {
    /// The default capacity MUST be the documented constant — deriving
    /// `Default` would leave `cap = 0` and silently discard every push.
    fn default() -> Self {
        Self {
            entries: Vec::new(),
            version: 0,
            cap: DEFAULT_HISTORY_CAP,
        }
    }
}

impl CommandHistory {
    pub fn new() -> Self {
        Self::default()
    }

    /// History with a custom capacity (spec "History Cap"): at most `cap`
    /// entries are retained; overflow discards the oldest.
    pub fn with_cap(cap: usize) -> Self {
        Self {
            entries: Vec::new(),
            version: 0,
            cap,
        }
    }

    /// Reconfigure the capacity of an EXISTING history (entry point env
    /// wiring). Existing entries are kept; the next push evicts on overflow.
    pub fn set_cap(&mut self, cap: usize) {
        self.cap = cap;
    }

    /// Current history version (number of mutations applied so far).
    pub fn version(&self) -> u64 {
        self.version
    }

    /// Append an applied command with its pre-computed inverse + metrics.
    ///
    /// Bounded by the capacity (spec "History Cap"): at capacity the OLDEST
    /// entry is evicted so the tail (undo) stays O(1). A zero cap disables
    /// retention entirely (guarded — `remove(0)` would panic on an empty vec).
    pub fn push(&mut self, entry: AppliedCommand) {
        if self.cap > 0 {
            if self.entries.len() >= self.cap {
                self.entries.remove(0);
            }
            self.entries.push(entry);
        }
        self.version += 1;
    }

    /// O(1) — remove and return the LAST applied command (no replay).
    /// Bumps the version only when an entry was actually removed.
    pub fn pop(&mut self) -> Option<AppliedCommand> {
        let popped = self.entries.pop();
        if popped.is_some() {
            self.version += 1;
        }
        popped
    }

    /// O(1) — peek the last applied command without removing it.
    pub fn last(&self) -> Option<&AppliedCommand> {
        self.entries.last()
    }

    /// O(1) — peek the last applied command together with the history version.
    ///
    /// The pair is read under a SINGLE lock (atomic view): the undo flow
    /// recompiles against `entry` and later commits with `version` as the
    /// expected value, so a concurrent mutation between peek and commit is
    /// detected at commit time (PR2 TOCTOU).
    pub fn last_with_version(&self) -> (Option<&AppliedCommand>, u64) {
        (self.entries.last(), self.version)
    }

    /// Remove ALL entries and bump the version — a mutation like any other,
    /// so a concurrent undo commit re-validated against the old version is
    /// rejected (PR2).
    pub fn clear(&mut self) {
        self.entries.clear();
        self.version += 1;
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};
    use thalos_engine::core::ids::OperationId;
    use thalos_engine::core::motion::segment::MotionSegment;

    /// A single-MoveJ program whose target moves under each edit.
    fn sample_program() -> PlanningProgram {
        PlanningProgram::new(vec![MotionSegment::MoveJ {
            origin: OperationId("op-0".to_string()),
            target: vec![0.0, 0.0],
            max_velocity: Some(500.0),
            max_acceleration: Some(1000.0),
        }])
    }

    /// A MoveWaypoint edit with a captured old target — the exact shape the
    /// apply pipeline records (design D6: pre-computed roundtrip inverse).
    fn move_waypoint_edit(new_target: f64) -> ProgramEdit {
        ProgramEdit::MoveWaypoint {
            segment_index: 0,
            new_target: vec![new_target, new_target],
            old_target: Some(vec![new_target - 1.0, new_target - 1.0]),
        }
    }

    /// Target of the single MoveJ segment (behavior signature — the segment
    /// struct is verbose; the target is what each edit moves).
    fn target_of(program: &PlanningProgram) -> Vec<f64> {
        match &program.segments[0] {
            MotionSegment::MoveJ { target, .. } => target.clone(),
            other => panic!("expected a MoveJ segment, got {other:?}"),
        }
    }

    #[test]
    fn undo_is_o1_with_100_plus_history_entries_no_replay() {
        // Spec command-endpoints "Undo is O(1)": a session with N > 100
        // applied commands — undo must be CONSTANT TIME. It pops ONE entry and
        // applies ONE stored inverse; it never replays the history. The cap is
        // raised to 150 so the scenario exercises the FULL 150-entry history
        // (PR3: the default cap would evict the oldest 50).
        let program = sample_program();
        let mut history = CommandHistory::with_cap(150);
        for i in 1..=150 {
            let cmd = move_waypoint_edit(i as f64);
            history.push(AppliedCommand {
                command: cmd.clone(),
                inverse: cmd.inverse(),
                metrics: CommandMetrics::new(0.4, 0.5),
                applied_program: program.segments.clone(),
            });
        }

        let start = Instant::now();
        let popped = history.pop().expect("150 entries → pop must succeed");
        let restored = popped.undo_program(&program).expect("single inverse apply");
        let elapsed = start.elapsed();

        // Exactly ONE entry was consumed — undo never walks the history.
        assert_eq!(
            history.len(),
            149,
            "undo must pop exactly one entry (not replay N-1 commands)"
        );
        // The single inverse apply restores the program to the state BEFORE
        // the last command (target 149), not a replay of all 150 edits.
        assert_eq!(
            target_of(&restored),
            vec![149.0, 149.0],
            "the stored inverse must restore the previous state in one apply"
        );
        // Constant-time guard: pop + one inverse apply is microseconds; an
        // O(n) replay with recompiles blows far past this ceiling.
        assert!(
            elapsed < Duration::from_secs(1),
            "undo with 150 history entries must be O(1), took {elapsed:?}"
        );
    }

    #[test]
    fn undo_with_1000_entries_still_pops_a_single_entry() {
        // Triangulation — a DIFFERENT scale (10x the spec's N > 100): the
        // operation count must stay constant regardless of history size. The
        // cap is raised to 1000 so the FULL 1000 entries are retained (PR3:
        // the default cap would evict the oldest 900).
        let program = sample_program();
        let mut history = CommandHistory::with_cap(1000);
        for i in 1..=1000 {
            let cmd = move_waypoint_edit(i as f64);
            history.push(AppliedCommand {
                command: cmd.clone(),
                inverse: cmd.inverse(),
                metrics: CommandMetrics::new(0.4, 0.5),
                applied_program: program.segments.clone(),
            });
        }

        let popped = history.pop().expect("1000 entries → pop must succeed");
        let restored = popped.undo_program(&program).expect("single inverse apply");

        assert_eq!(history.len(), 999, "one entry popped at any scale");
        assert_eq!(target_of(&restored), vec![999.0, 999.0]);
    }

    #[test]
    fn pop_on_empty_history_returns_none() {
        // Edge: an empty history has nothing to pop (the API maps this to the
        // 409 "Undo with empty history" scenario).
        let mut history = CommandHistory::new();
        assert!(history.is_empty());
        assert_eq!(history.len(), 0);
        assert!(history.pop().is_none(), "empty history → pop returns None");
        assert!(history.last().is_none(), "empty history → last returns None");
    }

    // ── PR2 — versioned undo (spec command-endpoints "Undo version mismatch") ──

    /// A minimal applied-command entry for the version-counter tests.
    fn entry_for(target: f64) -> AppliedCommand {
        let cmd = move_waypoint_edit(target);
        AppliedCommand {
            command: cmd.clone(),
            inverse: cmd.inverse(),
            metrics: CommandMetrics::new(0.4, 0.5),
            applied_program: sample_program().segments.clone(),
        }
    }

    #[test]
    fn version_increments_on_push() {
        // Spec: "The system MUST maintain a monotonic version counter (u64)
        // on command history, incremented on every mutation (push/pop/clear)."
        let mut history = CommandHistory::new();
        assert_eq!(history.version(), 0, "fresh history → version 0");
        history.push(entry_for(1.0));
        assert_eq!(history.version(), 1, "one push → version 1");
        history.push(entry_for(2.0));
        assert_eq!(history.version(), 2, "two pushes → version 2");
    }

    #[test]
    fn version_increments_on_pop_only_when_an_entry_is_removed() {
        // A pop that actually removes an entry is a mutation (bump); a pop on
        // an empty history removes nothing and must leave the version alone.
        let mut history = CommandHistory::new();
        history.push(entry_for(1.0));
        history.push(entry_for(2.0));
        assert_eq!(history.version(), 2);

        assert!(history.pop().is_some());
        assert_eq!(history.version(), 3, "pop of an entry must bump the version");
        assert!(history.pop().is_some());
        assert_eq!(history.version(), 4);

        assert!(history.pop().is_none());
        assert_eq!(
            history.version(),
            4,
            "pop on empty history removes nothing → version unchanged"
        );
    }

    #[test]
    fn last_with_version_returns_atomic_pair() {
        // PR2: the undo flow reads (last entry, version) under a SINGLE lock
        // so the commit can re-validate the version — no TOCTOU window between
        // the peek and the recompile.
        let mut history = CommandHistory::new();
        let (none_entry, v0) = history.last_with_version();
        assert!(none_entry.is_none(), "empty history → no entry");
        assert_eq!(v0, 0, "empty history → pair carries the current version");

        history.push(entry_for(1.0));
        let last = entry_for(2.0);
        history.push(last.clone());

        let (entry, version) = history.last_with_version();
        assert_eq!(version, 2, "pair must carry the CURRENT version");
        assert_eq!(entry, Some(&last), "pair must carry the LAST entry");
        assert_eq!(
            entry, history.last(),
            "pair entry must be the same view as last() (single lock)"
        );
    }

    #[test]
    fn clear_empties_and_bumps_version() {
        // `clear` is a mutation like push/pop — the version counter must
        // advance so a concurrent undo commit re-validated against the old
        // version is rejected.
        let mut history = CommandHistory::new();
        history.push(entry_for(1.0));
        history.push(entry_for(2.0));
        assert_eq!(history.version(), 2);

        history.clear();
        assert!(history.is_empty(), "clear must empty the history");
        assert_eq!(history.len(), 0, "clear must empty the history");
        assert_eq!(
            history.version(),
            3,
            "clear is a mutation → version must bump"
        );
    }

    // ── PR3 — history cap (spec command-endpoints "History Cap") ──

    /// Behavior-relevant target of an entry's command (the MoveWaypoint edit
    /// stores `new_target: vec![f, f]` — `[0]` is the behavior signature).
    fn target_of_edit(entry: &AppliedCommand) -> f64 {
        match &entry.command {
            ProgramEdit::MoveWaypoint { new_target, .. } => new_target[0],
            other => panic!("expected a MoveWaypoint edit, got {other:?}"),
        }
    }

    #[test]
    fn history_bounded_by_default_cap_discards_oldest_on_overflow() {
        // Spec "History Cap / Overflow discards oldest": history is bounded by
        // the default capacity (100) — 150 pushes keep only the 100 MOST
        // RECENT entries; the oldest ones are discarded, the newest preserved.
        let mut history = CommandHistory::new(); // default cap
        for i in 1..=150 {
            history.push(entry_for(i as f64));
        }

        assert_eq!(
            history.len(),
            100,
            "history must stay at the default cap after 150 pushes"
        );
        assert_eq!(
            target_of_edit(history.last().unwrap()),
            150.0,
            "the NEWEST entry must be preserved at overflow"
        );

        // Drain the bounded history — the newest is popped first; the oldest
        // SURVIVING entry is 51 (150 pushes − cap 100 = 50 discarded), and
        // entry 1 was evicted at overflow.
        let mut popped: Vec<f64> = Vec::new();
        while let Some(entry) = history.pop() {
            popped.push(target_of_edit(&entry));
        }
        assert_eq!(popped.len(), 100, "drain yields exactly the cap entries");
        assert_eq!(popped[0], 150.0, "pop drains from the tail — newest first");
        assert_eq!(
            *popped.last().unwrap(),
            51.0,
            "the oldest surviving entry is the 51st (1..=50 discarded)"
        );
        assert!(
            !popped.contains(&1.0),
            "entry 1 (oldest) must have been evicted at overflow"
        );
    }

    #[test]
    fn with_cap_sets_custom_capacity() {
        // Spec "History Cap": capacity is configurable — `with_cap(5)` bounds
        // the history to 5 entries regardless of the default.
        let mut history = CommandHistory::with_cap(5);
        for i in 1..=7 {
            history.push(entry_for(i as f64));
        }

        assert_eq!(history.len(), 5, "custom cap must bound the history");
        assert_eq!(
            target_of_edit(history.last().unwrap()),
            7.0,
            "the NEWEST entry must be preserved"
        );

        let mut popped: Vec<f64> = Vec::new();
        while let Some(entry) = history.pop() {
            popped.push(target_of_edit(&entry));
        }
        assert_eq!(
            popped,
            vec![7.0, 6.0, 5.0, 4.0, 3.0],
            "only the 5 most recent entries survive (1..=2 discarded)"
        );
    }
}
