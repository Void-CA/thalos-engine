use thalos_semantic::model::{
    MotionKind, MotionTarget, Provenance, ResolvedProgram, ResolvedStatement,
};

#[derive(Debug, Clone, PartialEq)]
pub struct PlanningMotion {
    pub kind: MotionKind,
    pub target: MotionTarget,
    pub provenance: Provenance,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanningInput {
    pub motions: Vec<PlanningMotion>,
}

impl PlanningInput {
    pub fn from_resolved(program: &ResolvedProgram) -> Self {
        let mut motions = Vec::new();
        for stmt in &program.statements {
            if let ResolvedStatement::Motion(m) = stmt {
                motions.push(PlanningMotion {
                    kind: m.kind.clone(),
                    target: m.target.clone(),
                    provenance: m.provenance.clone(),
                });
            }
        }
        Self { motions }
    }
}
