use chrono::{DateTime, Utc};

use thalos_engine::core::prelude::Trajectory;
use thalos_engine::planning::motion::program::{CompiledPlan, PlannedSegment, SemanticTarget};

use super::motion_type::MotionType;
use super::state::PlanState;

#[derive(Debug, Clone)]
pub struct ActiveMotionPlan {
    pub plan_id: String,
    pub state: PlanState,
    pub trajectory: Trajectory,
    pub motion_type: MotionType,
    /// Per-segment metadata when this plan came from a multi-segment program.
    pub segments: Option<Vec<PlannedSegment>>,
    /// Original semantic motion targets retained for live re-planning.
    pub semantic_targets: Option<Vec<SemanticTarget>>,
    pub created_at: DateTime<Utc>,
    pub started_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
    pub program_id: Option<String>,
    pub program_revision: Option<u64>,
    pub source_fingerprint: Option<String>,
}

impl ActiveMotionPlan {
    pub fn completed(
        plan_id: impl Into<String>,
        trajectory: Trajectory,
        motion_type: MotionType,
    ) -> Self {
        let now = Utc::now();
        Self {
            plan_id: plan_id.into(),
            state: PlanState::Completed,
            trajectory,
            motion_type,
            segments: None,
            semantic_targets: None,
            created_at: now,
            started_at: Some(now),
            completed_at: Some(now),
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
        }
    }

    pub fn from_compiled_plan(plan_id: impl Into<String>, compiled: CompiledPlan) -> Self {
        let segments = Some(compiled.segments.clone());
        let semantic_targets = compiled.semantic_targets.clone();
        Self {
            plan_id: plan_id.into(),
            state: PlanState::Created,
            trajectory: compiled.merged_trajectory,
            motion_type: MotionType::Program,
            segments,
            semantic_targets,
            created_at: Utc::now(),
            started_at: None,
            completed_at: None,
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
        }
    }

    pub fn created(
        plan_id: impl Into<String>,
        trajectory: Trajectory,
        motion_type: MotionType,
    ) -> Self {
        Self {
            plan_id: plan_id.into(),
            state: PlanState::Created,
            trajectory,
            motion_type,
            segments: None,
            semantic_targets: None,
            created_at: Utc::now(),
            started_at: None,
            completed_at: None,
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
        }
    }

    pub fn with_provenance(
        mut self,
        program_id: Option<String>,
        program_revision: Option<u64>,
        source_fingerprint: Option<String>,
    ) -> Self {
        self.program_id = program_id;
        self.program_revision = program_revision;
        self.source_fingerprint = source_fingerprint;
        self
    }

    pub fn start(&mut self) {
        self.state = PlanState::Active;
        self.started_at = Some(Utc::now());
    }

    pub fn complete(&mut self) {
        self.state = PlanState::Completed;
        self.completed_at = Some(Utc::now());
    }

    pub fn cancel(&mut self) {
        self.state = PlanState::Cancelled;
        self.completed_at = Some(Utc::now());
    }

    pub fn fail(&mut self) {
        self.state = PlanState::Failed;
        self.completed_at = Some(Utc::now());
    }

    pub fn progress(&self) -> f64 {
        match self.state {
            PlanState::Completed | PlanState::Cancelled | PlanState::Failed => 1.0,
            PlanState::Created => 0.0,
            PlanState::Active | PlanState::Paused => {
                let duration = self.trajectory.duration();
                if duration <= 0.0 {
                    return 1.0;
                }
                let elapsed = self
                    .started_at
                    .map(|start| (Utc::now() - start).num_seconds() as f64)
                    .unwrap_or(0.0);
                (elapsed / duration).clamp(0.0, 1.0)
            }
        }
    }
}
