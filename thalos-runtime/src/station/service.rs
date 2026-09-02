use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use serde::{Deserialize, Serialize};
use thalos_engine::prelude::StationId;

use crate::execution::session::{
    AcquisitionProvider, DomainExecutionCoordinator, ExecutionConfiguration,
    ExecutionDomainError, ExecutionSessionId, ExpectedState, RobotObservationProvider,
    TelemetryExecutionRunner,
};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RoboticsModuleId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct AcquisitionModuleId(pub String);

/// Módulo de robótica encapsulado dentro del contexto de una `Station`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoboticsModule {
    pub id: RoboticsModuleId,
    pub station_id: StationId,
    pub name: String,
    pub robot_name: String,
    pub controller_binding: String,
}

/// Módulo de adquisición (IIoT / sensores / visión) dentro del contexto de una `Station`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AcquisitionModule {
    pub id: AcquisitionModuleId,
    pub station_id: StationId,
    pub name: String,
    pub channels: HashMap<String, f64>,
}

/// Entidad raíz autoritativa operacional de una celda industrial.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Station {
    pub id: StationId,
    pub name: String,
    pub robotics_modules: HashMap<RoboticsModuleId, RoboticsModule>,
    pub acquisition_modules: HashMap<AcquisitionModuleId, AcquisitionModule>,
}

impl Station {
    pub fn new(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: StationId(id.into()),
            name: name.into(),
            robotics_modules: HashMap::new(),
            acquisition_modules: HashMap::new(),
        }
    }

    pub fn add_robotics_module(&mut self, module: RoboticsModule) {
        self.robotics_modules.insert(module.id.clone(), module);
    }

    pub fn add_acquisition_module(&mut self, module: AcquisitionModule) {
        self.acquisition_modules.insert(module.id.clone(), module);
    }
}

/// Intención de ejecución enviada desde la capa de aplicación o UI.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ExecutionTarget {
    pub station_id: StationId,
    pub robotics_module_id: RoboticsModuleId,
}

/// Binding de infraestructura resuelto y validado, listo para la instanciación de un ExecutionSession.
#[derive(Debug, Clone)]
pub struct ExecutionBinding<A, R> {
    pub target: ExecutionTarget,
    pub station: Station,
    pub robotics_module: RoboticsModule,
    pub acquisition_provider: A,
    pub robot_observation_provider: R,
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum StationServiceError {
    #[error("Station not found: {0:?}")]
    StationNotFound(StationId),

    #[error("Robotics module not found: {0:?}")]
    RoboticsModuleNotFound(RoboticsModuleId),

    #[error("Robotics module station mismatch: target station {target:?}, module station {actual:?}")]
    StationModuleMismatch {
        target: StationId,
        actual: StationId,
    },

    #[error("Execution domain error: {0}")]
    ExecutionDomain(#[from] ExecutionDomainError),
}

/// Servicio de aplicación para la gestión de Stations y la preparación transaccional de ExecutionSessions.
#[derive(Clone, Default)]
pub struct StationService {
    stations: Arc<Mutex<HashMap<StationId, Station>>>,
}

impl StationService {
    pub fn new() -> Self {
        Self {
            stations: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn register_station(&self, station: Station) {
        self.stations.lock().unwrap().insert(station.id.clone(), station);
    }

    pub fn get_station(&self, id: &StationId) -> Option<Station> {
        self.stations.lock().unwrap().get(id).cloned()
    }

    /// Resuelve el `ExecutionTarget` comprobando la existencia de la Station, el RoboticsModule y que pertenezcan a la misma celda.
    pub fn resolve_binding<A, R>(
        &self,
        target: &ExecutionTarget,
        acq_provider: A,
        robot_provider: R,
    ) -> Result<ExecutionBinding<A, R>, StationServiceError>
    where
        A: AcquisitionProvider,
        R: RobotObservationProvider,
    {
        let stations = self.stations.lock().unwrap();
        let station = stations
            .get(&target.station_id)
            .cloned()
            .ok_or_else(|| StationServiceError::StationNotFound(target.station_id.clone()))?;

        let module = station
            .robotics_modules
            .get(&target.robotics_module_id)
            .cloned()
            .ok_or_else(|| StationServiceError::RoboticsModuleNotFound(target.robotics_module_id.clone()))?;

        if module.station_id != target.station_id {
            return Err(StationServiceError::StationModuleMismatch {
                target: target.station_id.clone(),
                actual: module.station_id.clone(),
            });
        }

        Ok(ExecutionBinding {
            target: target.clone(),
            station,
            robotics_module: module,
            acquisition_provider: acq_provider,
            robot_observation_provider: robot_provider,
        })
    }

    /// Preparación transaccional de un ExecutionSession.
    ///
    /// Valida que la estación y los módulos existan y concuerden antes de invocar `coordinator.create_session`.
    /// Si la validación falla, NO se genera ninguna sesión en el runtime.
    pub fn prepare_execution_session<A, R>(
        &self,
        target: &ExecutionTarget,
        program_id: impl Into<String>,
        config: ExecutionConfiguration,
        acq_provider: A,
        robot_provider: R,
        coordinator: &DomainExecutionCoordinator,
    ) -> Result<(ExecutionSessionId, TelemetryExecutionRunner<A, R>), StationServiceError>
    where
        A: AcquisitionProvider,
        R: RobotObservationProvider,
    {
        // 1. Resolver y validar binding transaccionalmente
        let binding = self.resolve_binding(target, acq_provider, robot_provider)?;

        // 2. Crear runner telemetrizado a partir del binding resuelto
        let runner = TelemetryExecutionRunner::new(
            binding.acquisition_provider,
            binding.robot_observation_provider,
            ExpectedState::default(),
        );

        // 3. Crear la sesión en el coordinador de dominio
        let session_id = coordinator.create_session(program_id, config);

        Ok((session_id, runner))
    }
}
