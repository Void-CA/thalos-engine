use std::sync::Arc;
use serde::{Deserialize, Serialize};
use thalos_engine::prelude::StationId;

use crate::execution::session::{
    CommandProvider, DomainExecutionCoordinator, ExecutionConfiguration,
    ExecutionDomainError, ExecutionSessionId, ExpectedState, ObservationProvider,
    RobotObservationProvider, TelemetryExecutionRunner,
};
use crate::ports::equipment_module_repository::{
    ChannelRecord, EquipmentModuleRecord, EquipmentModuleRepository,
    InterconnectionModuleRecord, RoboticsModuleRecord,
};
use crate::ports::robot_repository::RobotRepository;
use crate::ports::station_repository::StationRepository;
use crate::ports::StationRecord;
use crate::station::equipment_module::{
    Channel, EquipmentModule, EquipmentModuleId, EquipmentModuleKind,
    InterconnectionModuleExtension, RoboticsModuleExtension,
};

// ═══════════════════════════════════════════════════════════════════════════
// Legacy types — kept for backward compatibility during transition
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RoboticsModuleId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct InterconnectionModuleId(pub String);

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoboticsModule {
    pub id: RoboticsModuleId,
    pub station_id: StationId,
    pub name: String,
    pub robot_name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot_definition_id: Option<String>,
    pub controller_binding: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterconnectionModule {
    pub id: InterconnectionModuleId,
    pub station_id: StationId,
    pub name: String,
    pub channels: std::collections::HashMap<String, f64>,
}

/// Station identity — modules are loaded separately via repository.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Station {
    pub id: StationId,
    pub name: String,
}

impl Station {
    pub fn new(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: StationId(id.into()),
            name: name.into(),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Execution types
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ExecutionTarget {
    pub station_id: StationId,
    pub robotics_module_id: RoboticsModuleId,
}

#[derive(Debug, Clone)]
pub struct ExecutionBinding<A, R> {
    pub target: ExecutionTarget,
    pub station: Station,
    pub robotics_module: RoboticsModule,
    pub observation_provider: A,
    pub robot_observation_provider: R,
}

// ═══════════════════════════════════════════════════════════════════════════
// Errors
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Debug, thiserror::Error)]
pub enum StationError {
    #[error("Station not found: {0}")]
    NotFound(String),

    #[error("Robot not found: {0}")]
    RobotNotFound(String),

    #[error("Module not found: {0}")]
    ModuleNotFound(String),

    #[error("Invalid module kind: expected {expected}, got {actual}")]
    InvalidModuleKind { expected: String, actual: String },

    #[error("Invalid channel: {0}")]
    InvalidChannel(String),

    #[error("Execution domain error: {0}")]
    ExecutionDomain(#[from] ExecutionDomainError),

    #[error("Persistence error: {0}")]
    Persistence(String),
}

impl From<crate::ports::PersistenceError> for StationError {
    fn from(e: crate::ports::PersistenceError) -> Self {
        StationError::Persistence(e.to_string())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// StationService — thin service backed by repositories
// ═══════════════════════════════════════════════════════════════════════════

/// Application service for Station lifecycle and module orchestration.
///
/// No in-memory state. All persistence is delegated to repositories.
/// Domain validation happens here; persistence happens in repositories.
pub struct StationService {
    station_repo: Arc<dyn StationRepository>,
    equipment_module_repo: Arc<dyn EquipmentModuleRepository>,
    robot_repo: Arc<dyn RobotRepository>,
}

impl StationService {
    pub fn new(
        station_repo: Arc<dyn StationRepository>,
        equipment_module_repo: Arc<dyn EquipmentModuleRepository>,
        robot_repo: Arc<dyn RobotRepository>,
    ) -> Self {
        Self {
            station_repo,
            equipment_module_repo,
            robot_repo,
        }
    }

    // ─── Station CRUD ──────────────────────────────────────────────

    pub async fn create_station(&self, id: &str, name: &str) -> Result<Station, StationError> {
        let station = Station::new(id, name);
        let now = chrono::Utc::now().to_rfc3339();
        let record = StationRecord {
            id: station.id.0.clone(),
            name: station.name.clone(),
            created_at: now.clone(),
            updated_at: now,
        };
        self.station_repo.save(&record).await?;
        Ok(station)
    }

    pub async fn update_station(&self, id: &StationId, name: &str) -> Result<Station, StationError> {
        let existing = self.station_repo.get(&id.0).await?;
        if existing.is_none() {
            return Err(StationError::NotFound(id.0.clone()));
        }

        let now = chrono::Utc::now().to_rfc3339();
        let record = StationRecord {
            id: id.0.clone(),
            name: name.to_string(),
            created_at: existing.unwrap().created_at,
            updated_at: now,
        };
        self.station_repo.save(&record).await?;

        Ok(Station {
            id: id.clone(),
            name: name.to_string(),
        })
    }

    pub async fn delete_station(&self, id: &StationId) -> Result<(), StationError> {
        let exists = self.station_repo.get(&id.0).await?;
        if exists.is_none() {
            return Err(StationError::NotFound(id.0.clone()));
        }
        // FK CASCADE handles equipment_modules → extensions → channels
        self.station_repo.delete(&id.0).await?;
        Ok(())
    }

    pub async fn get_station(&self, id: &StationId) -> Result<Option<Station>, StationError> {
        let record = self.station_repo.get(&id.0).await?;
        Ok(record.map(|r| Station {
            id: StationId(r.id),
            name: r.name,
        }))
    }

    pub async fn list_stations(&self) -> Result<Vec<Station>, StationError> {
        let records = self.station_repo.list().await?;
        Ok(records
            .into_iter()
            .map(|r| Station {
                id: StationId(r.id),
                name: r.name,
            })
            .collect())
    }

    // ─── Module CRUD (atomic) ──────────────────────────────────────

    pub async fn add_robotics_module(
        &self,
        station_id: &StationId,
        robot_id: &str,
        name: &str,
        configuration_json: &str,
    ) -> Result<EquipmentModule, StationError> {
        // 1. Station exists?
        self.station_repo
            .get(&station_id.0)
            .await?
            .ok_or_else(|| StationError::NotFound(station_id.0.clone()))?;

        // 2. Robot exists? (only if robot_id is provided)
        if !robot_id.is_empty() {
            self.robot_repo
                .get(robot_id)
                .await
                .map_err(|e| StationError::Persistence(e.to_string()))?
                .ok_or_else(|| StationError::RobotNotFound(robot_id.to_string()))?;
        }

        // 3. Create atomically
        let module_id = EquipmentModuleId(uuid::Uuid::new_v4().to_string());
        let now = chrono::Utc::now().to_rfc3339();

        let module_record = EquipmentModuleRecord {
            id: module_id.0.clone(),
            station_id: station_id.0.clone(),
            kind: EquipmentModuleKind::Robotics.to_string(),
            name: name.to_string(),
            created_at: now.clone(),
            updated_at: now,
        };

        let extension_record = RoboticsModuleRecord {
            module_id: module_id.0.clone(),
            robot_id: robot_id.to_string(),
            configuration_json: configuration_json.to_string(),
        };

        self.equipment_module_repo
            .create_robotics_module(&module_record, &extension_record)
            .await?;

        Ok(EquipmentModule {
            id: module_id,
            station_id: station_id.clone(),
            name: name.to_string(),
            kind: EquipmentModuleKind::Robotics,
        })
    }

    pub async fn add_interconnection_module(
        &self,
        station_id: &StationId,
        name: &str,
    ) -> Result<EquipmentModule, StationError> {
        // 1. Station exists?
        self.station_repo
            .get(&station_id.0)
            .await?
            .ok_or_else(|| StationError::NotFound(station_id.0.clone()))?;

        // 2. Create atomically
        let module_id = EquipmentModuleId(uuid::Uuid::new_v4().to_string());
        let now = chrono::Utc::now().to_rfc3339();

        let module_record = EquipmentModuleRecord {
            id: module_id.0.clone(),
            station_id: station_id.0.clone(),
            kind: EquipmentModuleKind::Interconnection.to_string(),
            name: name.to_string(),
            created_at: now.clone(),
            updated_at: now,
        };

        let extension_record = InterconnectionModuleRecord {
            module_id: module_id.0.clone(),
            configuration_json: "{}".to_string(),
        };

        self.equipment_module_repo
            .create_interconnection_module(&module_record, &extension_record)
            .await?;

        Ok(EquipmentModule {
            id: module_id,
            station_id: station_id.clone(),
            name: name.to_string(),
            kind: EquipmentModuleKind::Interconnection,
        })
    }

    pub async fn remove_module(&self, module_id: &EquipmentModuleId) -> Result<(), StationError> {
        // Verify module exists
        self.equipment_module_repo
            .get(&module_id.0)
            .await?
            .ok_or_else(|| StationError::ModuleNotFound(module_id.0.clone()))?;

        // FK CASCADE handles extension + channels
        self.equipment_module_repo
            .delete(&module_id.0)
            .await?;
        Ok(())
    }

    pub async fn update_module(&self, module_id: &EquipmentModuleId, name: &str) -> Result<EquipmentModule, StationError> {
        // Verify module exists
        let existing = self.equipment_module_repo
            .get(&module_id.0)
            .await?
            .ok_or_else(|| StationError::ModuleNotFound(module_id.0.clone()))?;

        let now = chrono::Utc::now().to_rfc3339();
        let record = EquipmentModuleRecord {
            id: existing.id,
            station_id: existing.station_id,
            kind: existing.kind,
            name: name.to_string(),
            created_at: existing.created_at,
            updated_at: now,
        };

        self.equipment_module_repo
            .update_module(&record)
            .await?;

        let kind: EquipmentModuleKind = record.kind.parse().unwrap_or(EquipmentModuleKind::Robotics);
        Ok(EquipmentModule {
            id: module_id.clone(),
            station_id: StationId(record.station_id),
            name: name.to_string(),
            kind,
        })
    }

    // ─── Channel CRUD ──────────────────────────────────────────────

    pub async fn add_channel(
        &self,
        module_id: &EquipmentModuleId,
        symbol: &str,
        name: &str,
        data_type: &str,
        unit: &str,
    ) -> Result<Channel, StationError> {
        // 1. Module exists?
        let module_record = self
            .equipment_module_repo
            .get(&module_id.0)
            .await?
            .ok_or_else(|| StationError::ModuleNotFound(module_id.0.clone()))?;

        // 2. Module kind is interconnection?
        let kind: EquipmentModuleKind = module_record
            .kind
            .parse()
            .map_err(|e: String| StationError::Persistence(e))?;
        if kind != EquipmentModuleKind::Interconnection {
            return Err(StationError::InvalidModuleKind {
                expected: "interconnection".to_string(),
                actual: module_record.kind,
            });
        }

        // 3. Validate channel fields
        let channel = Channel {
            id: uuid::Uuid::new_v4().to_string(),
            interconnection_module_id: module_id.clone(),
            symbol: symbol.to_string(),
            name: name.to_string(),
            data_type: data_type.to_string(),
            unit: unit.to_string(),
        };
        channel.validate().map_err(StationError::InvalidChannel)?;

        // 4. Persist
        let record = ChannelRecord {
            id: channel.id.clone(),
            interconnection_module_id: module_id.0.clone(),
            symbol: channel.symbol.clone(),
            name: channel.name.clone(),
            data_type: channel.data_type.clone(),
            unit: channel.unit.clone(),
        };
        self.equipment_module_repo.save_channel(&record).await?;

        Ok(channel)
    }

    pub async fn remove_channel(&self, channel_id: &str) -> Result<(), StationError> {
        self.equipment_module_repo
            .delete_channel(channel_id)
            .await?;
        Ok(())
    }

    // ─── Granular queries ──────────────────────────────────────────

    pub async fn get_station_modules(
        &self,
        station_id: &StationId,
    ) -> Result<Vec<EquipmentModule>, StationError> {
        let records = self
            .equipment_module_repo
            .list_by_station(&station_id.0)
            .await?;

        Ok(records
            .into_iter()
            .map(|r| {
                let kind: EquipmentModuleKind = r.kind.parse().unwrap_or(EquipmentModuleKind::Robotics);
                EquipmentModule {
                    id: EquipmentModuleId(r.id),
                    station_id: StationId(r.station_id),
                    name: r.name,
                    kind,
                }
            })
            .collect())
    }

    pub async fn get_module_robotics_extension(
        &self,
        module_id: &EquipmentModuleId,
    ) -> Result<Option<RoboticsModuleExtension>, StationError> {
        let record = self
            .equipment_module_repo
            .get_robotics_extension(&module_id.0)
            .await?;

        Ok(record.map(|r| RoboticsModuleExtension {
            module_id: EquipmentModuleId(r.module_id),
            robot_id: r.robot_id,
            configuration_json: r.configuration_json,
        }))
    }

    pub async fn get_module_interconnection_extension(
        &self,
        module_id: &EquipmentModuleId,
    ) -> Result<Option<InterconnectionModuleExtension>, StationError> {
        let record = self
            .equipment_module_repo
            .get_interconnection_extension(&module_id.0)
            .await?;

        Ok(record.map(|r| InterconnectionModuleExtension {
            module_id: EquipmentModuleId(r.module_id),
            configuration_json: r.configuration_json,
        }))
    }

    pub async fn get_module_channels(
        &self,
        module_id: &EquipmentModuleId,
    ) -> Result<Vec<Channel>, StationError> {
        let records = self
            .equipment_module_repo
            .list_channels(&module_id.0)
            .await?;

        Ok(records
            .into_iter()
            .map(|r| Channel {
                id: r.id,
                interconnection_module_id: EquipmentModuleId(r.interconnection_module_id),
                symbol: r.symbol,
                name: r.name,
                data_type: r.data_type,
                unit: r.unit,
            })
            .collect())
    }

    // ─── Execution binding ─────────────────────────────────────────

    pub async fn resolve_binding<A, R>(
        &self,
        target: &ExecutionTarget,
        acq_provider: A,
        robot_provider: R,
    ) -> Result<ExecutionBinding<A, R>, StationError>
    where
        A: ObservationProvider,
        R: RobotObservationProvider,
    {
        // Load station
        let station = self
            .get_station(&target.station_id)
            .await?
            .ok_or_else(|| StationError::NotFound(target.station_id.0.clone()))?;

        // Load module
        let module_record = self
            .equipment_module_repo
            .get(&target.robotics_module_id.0)
            .await?
            .ok_or_else(|| StationError::ModuleNotFound(target.robotics_module_id.0.clone()))?;

        // Load robotics extension
        let ext = self
            .equipment_module_repo
            .get_robotics_extension(&target.robotics_module_id.0)
            .await?
            .ok_or_else(|| StationError::ModuleNotFound(
                format!("No robotics extension for {}", target.robotics_module_id.0)
            ))?;

        // Build legacy RoboticsModule for execution binding
        let robotics_module = RoboticsModule {
            id: RoboticsModuleId(module_record.id.clone()),
            station_id: StationId(module_record.station_id.clone()),
            name: module_record.name.clone(),
            robot_name: ext.robot_id.clone(),
            robot_definition_id: Some(ext.robot_id.clone()),
            controller_binding: serde_json::from_str(&ext.configuration_json)
                .ok()
                .and_then(|v: serde_json::Value| v.get("controller_binding")
                    .and_then(|c| c.as_str().map(|s| s.to_string())))
                .unwrap_or_else(|| "simulation".to_string()),
        };

        Ok(ExecutionBinding {
            target: target.clone(),
            station,
            robotics_module,
            observation_provider: acq_provider,
            robot_observation_provider: robot_provider,
        })
    }

    pub fn prepare_execution_session<A, R, C>(
        &self,
        target: &ExecutionTarget,
        program_id: impl Into<String>,
        config: ExecutionConfiguration,
        acq_provider: A,
        robot_provider: R,
        cmd_provider: C,
        coordinator: &DomainExecutionCoordinator,
    ) -> Result<(ExecutionSessionId, TelemetryExecutionRunner<A, R, C>), StationError>
    where
        A: ObservationProvider + 'static,
        R: RobotObservationProvider + 'static,
        C: CommandProvider + 'static,
    {
        // Note: resolve_binding is now async, but prepare_execution_session
        // needs to be called with a pre-resolved binding.
        // For now, we keep the session creation synchronous.
        let runner = TelemetryExecutionRunner::new(
            acq_provider,
            robot_provider,
            cmd_provider,
            ExpectedState::default(),
        );

        let session_id = coordinator.create_session_with_target(
            target.station_id.0.clone(),
            target.robotics_module_id.0.clone(),
            program_id,
            config,
        );

        Ok((session_id, runner))
    }
}
