use thiserror::Error;

/// Declarative definition of a robot in a catalog: **identity** only.
///
/// It carries no asset paths or asset references (asset/URDF resolution is an
/// adapter concern) and no state or derived kinematic representation.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotCatalogEntry {
    pub id: String,
    pub display_name: String,
    pub manufacturer: String,
    pub model: String,
    pub payload_kg: Option<f64>,
    pub reach_m: Option<f64>,
    pub visual_format: String,
    pub collision_format: String,
}

/// Identity error of a catalog lookup.
///
/// Asset/URDF errors belong to the adapter.
#[derive(Debug, Error, PartialEq)]
pub enum RobotCatalogError {
    #[error("robot definition not found in catalog: {0}")]
    DefinitionNotFound(String),
}
