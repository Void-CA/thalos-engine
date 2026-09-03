use std::path::PathBuf;

use thiserror::Error;

use thalos_engine::core::robot::adapter;
use thalos_engine::core::robot::serial_chain::SerialChain;
use thalos_engine::models::Robot;
use thalos_importer::import_urdf;

/// Definición de un robot del catálogo: identidad técnica + referencia a assets.
///
/// Es la autoridad para "qué robot es". No contiene estado ni representación
/// cinemática derivada — solo identidad declarativa y referencias a los assets
/// (URDF + mallas) desde los que se construyen `SerialChain` y `VisualScene`.
#[derive(Debug, Clone)]
pub struct RobotDefinition {
    pub id: String,
    pub display_name: String,
    pub manufacturer: String,
    pub model: String,
    pub payload_kg: Option<f64>,
    pub reach_m: Option<f64>,
    pub visual_format: String,
    pub collision_format: String,
    /// Raíz del directorio de assets de este robot (contiene `urdf/` y `meshes/`).
    pub asset_root: PathBuf,
}

/// Resultado de resolver una definición: la definición + su URDF en disco.
///
/// Distinción semántica:
/// - `RobotDefinition` = **qué robot es** (identidad).
/// - `RobotDefinitionResolution` = **qué obtuvimos al resolverlo** (URDF accesible).
/// - `SerialChain` = representación cinemática derivada.
#[derive(Debug, Clone)]
pub struct RobotDefinitionResolution {
    pub definition: RobotDefinition,
    pub urdf_path: PathBuf,
}

/// Error de resolución/load de definiciones del catálogo.
#[derive(Debug, Error, PartialEq)]
pub enum RobotCatalogError {
    #[error("robot definition not found in catalog: {0}")]
    DefinitionNotFound(String),
    #[error("URDF asset missing for definition {definition}: {path}")]
    UrdfAssetMissing { definition: String, path: String },
    #[error("invalid URDF for definition {definition}: {message}")]
    InvalidUrdf { definition: String, message: String },
    #[error("cannot build kinematic chain for definition {definition}: {message}")]
    ChainError { definition: String, message: String },
}

/// Catálogo canónico de robots.
///
/// Fuente única de verdad para la **identidad** de robot en el vertical slice.
/// Consumido por el runtime (resolución → `SerialChain`) y por la API (→ DTO).
///
/// Los assets residen en `assets/robots/<definition_id>/` dentro de este crate.
/// En el futuro pueden migrar a persistencia o a un registro dinámico sin
/// cambiar este contrato: el catálogo solo declara qué definiciones existen y
/// dónde están sus assets.
pub struct RobotCatalog {
    definitions: Vec<RobotDefinition>,
}

impl RobotCatalog {
    /// Directorio base de assets del catálogo — `assets/robots/` del crate.
    fn asset_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/robots")
    }

    /// Construye el catálogo canónico (UR10 + ABB IRB 1300).
    pub fn canonical() -> Self {
        let root = Self::asset_root();
        Self {
            definitions: vec![
                RobotDefinition {
                    id: "universal_robots_ur10".to_string(),
                    display_name: "Universal Robots UR10".to_string(),
                    manufacturer: "Universal Robots".to_string(),
                    model: "UR10".to_string(),
                    payload_kg: Some(10.0),
                    reach_m: Some(1.30),
                    visual_format: "dae".to_string(),
                    collision_format: "stl".to_string(),
                    asset_root: root.join("universal_robots_ur10"),
                },
                RobotDefinition {
                    id: "abb_irb1300_10_115".to_string(),
                    display_name: "ABB IRB 1300-10/1.15".to_string(),
                    manufacturer: "ABB".to_string(),
                    model: "IRB 1300-10/1.15".to_string(),
                    payload_kg: Some(10.0),
                    reach_m: Some(1.15),
                    visual_format: "stl".to_string(),
                    collision_format: "stl".to_string(),
                    asset_root: root.join("abb_irb1300_10_115"),
                },
            ],
        }
    }

    pub fn definitions(&self) -> &[RobotDefinition] {
        &self.definitions
    }

    /// Resuelve metadata + referencia de assets para una definición.
    ///
    /// NO carga el URDF ni construye la cadena — es puramente declarativo.
    /// Devuelve la ruta al URDF para permitir el "load" diferido.
    pub fn resolve(&self, definition_id: &str) -> Result<RobotDefinitionResolution, RobotCatalogError> {
        let definition = self
            .definitions
            .iter()
            .find(|d| d.id == definition_id)
            .cloned()
            .ok_or_else(|| RobotCatalogError::DefinitionNotFound(definition_id.to_string()))?;

        let urdf_path = match definition.asset_root.join("urdf").read_dir() {
            Ok(entries) => entries
                .filter_map(|e| e.ok())
                .map(|e| e.path())
                .find(|p| p.extension().map_or(false, |ext| ext == "urdf"))
                .ok_or_else(|| {
                    RobotCatalogError::UrdfAssetMissing {
                        definition: definition.id.clone(),
                        path: definition.asset_root.join("urdf").display().to_string(),
                    }
                })?,
            Err(_) => {
                return Err(RobotCatalogError::UrdfAssetMissing {
                    definition: definition.id.clone(),
                    path: definition.asset_root.join("urdf").display().to_string(),
                })
            }
        };

        Ok(RobotDefinitionResolution {
            definition,
            urdf_path,
        })
    }

    /// Carga la definición: lee el URDF, lo parsea y construye la `SerialChain`.
    ///
    /// El DOF se deriva SIEMPRE del `SerialChain` (`chain.dof_count()`), nunca
    /// de metadata declarativa — el URDF es la fuente de verdad de la topología.
    ///
    /// Devuelve además el modelo `Robot` parseado para que el pipeline visual
    /// (mapping de mallas desde `robot_source`) derive de la misma definición
    /// que produjo la cadena cinemática.
    pub fn load_definition(
        &self,
        definition_id: &str,
    ) -> Result<(RobotDefinitionResolution, SerialChain, Robot), RobotCatalogError> {
        let resolution = self.resolve(definition_id)?;
        let urdf_xml = std::fs::read_to_string(&resolution.urdf_path).map_err(|e| {
            RobotCatalogError::InvalidUrdf {
                definition: definition_id.to_string(),
                message: e.to_string(),
            }
        })?;

        let robot = import_urdf(&urdf_xml).map_err(|e| RobotCatalogError::InvalidUrdf {
            definition: definition_id.to_string(),
            message: e.to_string(),
        })?;

        let chain = adapter::auto(&robot).map_err(|e| RobotCatalogError::ChainError {
            definition: definition_id.to_string(),
            message: e.to_string(),
        })?;

        Ok((resolution, chain, robot))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ur10_resolves_definition_and_derives_6dof_chain() {
        let catalog = RobotCatalog::canonical();
        let (resolution, chain, robot) = catalog
            .load_definition("universal_robots_ur10")
            .expect("UR10 must resolve and load");

        assert_eq!(resolution.definition.display_name, "Universal Robots UR10");
        assert_eq!(resolution.definition.model, "UR10");
        assert!(resolution.definition.asset_root.exists());
        assert_eq!(chain.dof_count(), 6, "UR10 URDF must produce a 6-DOF chain");
        assert_eq!(robot.name, "ur10");
    }

    #[test]
    fn abb_irb1300_resolves_definition_and_derives_6dof_chain() {
        let catalog = RobotCatalog::canonical();
        let (resolution, chain, robot) = catalog
            .load_definition("abb_irb1300_10_115")
            .expect("ABB must resolve and load");

        assert_eq!(resolution.definition.model, "IRB 1300-10/1.15");
        assert_eq!(chain.dof_count(), 6, "ABB URDF must produce a 6-DOF chain");
        assert_eq!(robot.name, "abb_irb1300_10_115");
    }

    #[test]
    fn unknown_definition_is_rejected() {
        let catalog = RobotCatalog::canonical();
        let err = catalog
            .resolve("nonexistent_robot")
            .expect_err("unknown id must fail");
        assert!(matches!(err, RobotCatalogError::DefinitionNotFound(_)));
    }
}

