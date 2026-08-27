//! Modelo de evaluación de planes — métricas.
//!
//! Separa el **qué medir** (`PlanMetrics`) del **cómo ponderarlo**.
//! El evaluador (`PlanEvaluator`) convierte análisis existentes en métricas.

pub mod evaluator;
pub mod metrics;

pub use evaluator::PlanEvaluator;
pub use metrics::{
    CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics, MetricKind, PlanMetrics,
};
