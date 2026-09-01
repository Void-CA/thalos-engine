#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanState {
    Created,
    Active,
    Paused,
    Completed,
    Cancelled,
    Failed,
}

impl PlanState {
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            PlanState::Completed | PlanState::Cancelled | PlanState::Failed
        )
    }
}
