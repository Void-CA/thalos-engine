pub mod equipment_module;
pub mod runtime;
pub mod service;
pub mod state;

pub use equipment_module::{
    AcquisitionModuleExtension, Channel, EquipmentModule, EquipmentModuleId, EquipmentModuleKind,
    RoboticsModuleExtension,
};
pub use runtime::{StationRuntime, StationRuntimeError};
pub use service::{
    AcquisitionModule, AcquisitionModuleId, ExecutionBinding, ExecutionTarget, RoboticsModule,
    RoboticsModuleId, Station, StationError, StationService,
};
pub use state::{ModuleKind, ModuleRuntimeState, OperationalSession, StationRuntimeState};

// Re-export StationId from thalos-engine for downstream consumers
pub use thalos_engine::prelude::StationId;
