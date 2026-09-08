pub use thalos_ports::device;
pub use thalos_ports::robot;
pub mod equipment_module_repository;
pub mod robot_reference_checker;
pub mod robot_repository;
pub mod station_repository;
pub mod workspace_repository;

pub use device::{ChannelId, ChannelObservation, ChannelValue, DeviceTransport, DeviceTransportError};
pub use crate::test_support::FakeDeviceTransport;
pub use robot::{RobotObservation, RobotTransport, TransportError, TransportState};
pub use equipment_module_repository::{
    ChannelRecord, EquipmentModuleRecord, EquipmentModuleRepository,
    InterconnectionModuleRecord, RobotReference as ModuleRobotReference, RoboticsModuleRecord,
};
pub use robot_reference_checker::{RobotReference, RobotReferenceChecker};
pub use robot_repository::{
    PersistenceError, Result as PersistenceResult, RobotRecord, RobotRepository, RobotSource,
};
pub use station_repository::{StationRecord, StationRepository};
pub use workspace_repository::WorkspaceRepository;
