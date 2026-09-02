use std::sync::Arc;
use thalos_engine::prelude::StationId;
use thalos_runtime::execution::session::{
    Action, AcquisitionSnapshot, Decision, DomainExecutionCoordinator, ExecutionConfiguration,
    ExecutionEventBus, ExecutionHistory, ExecutionHistoryStore, InMemoryAcquisitionRegistry,
    LifecycleState, RobotState, SharedRobotObservation, TickOutcome,
};
use thalos_runtime::station::{
    AcquisitionModule, AcquisitionModuleId, ExecutionTarget, RoboticsModule, RoboticsModuleId,
    Station, StationService, StationServiceError,
};

#[test]
fn test_end_to_end_station_service_execution_pipeline() {
    // 1. Inicializar EventBus y HistoryStore
    let bus = ExecutionEventBus::new();
    let history_store = Arc::new(ExecutionHistoryStore::new());
    bus.subscribe(history_store.clone());

    let coordinator = DomainExecutionCoordinator::with_event_bus(bus);
    let service = StationService::new();

    // 2. Configurar la celda industrial Station
    let station_id = StationId("cell_weld_01".to_string());
    let mut station = Station::new("cell_weld_01", "Welding & Inspection Station A");

    let robotics_mod_id = RoboticsModuleId("arm_scara_01".to_string());
    station.add_robotics_module(RoboticsModule {
        id: robotics_mod_id.clone(),
        station_id: station_id.clone(),
        name: "Main Manipulator".to_string(),
        robot_name: "SCARA-01".to_string(),
        controller_binding: "simulated_transport".to_string(),
    });

    let acq_vision_id = AcquisitionModuleId("vision_mod".to_string());
    station.add_acquisition_module(AcquisitionModule {
        id: acq_vision_id,
        station_id: station_id.clone(),
        name: "Vision System".to_string(),
        channels: [("target_x".to_string(), 100.0), ("target_y".to_string(), 50.0)]
            .into_iter()
            .collect(),
    });

    let acq_env_id = AcquisitionModuleId("env_mod".to_string());
    station.add_acquisition_module(AcquisitionModule {
        id: acq_env_id,
        station_id: station_id.clone(),
        name: "Environment Sensors".to_string(),
        channels: [("temperature".to_string(), 120.0)].into_iter().collect(),
    });

    service.register_station(station);

    // 3. Crear providers de telemetría para los canales y la observación del robot
    let acq_registry = InMemoryAcquisitionRegistry::new();
    acq_registry.set_channel("target_x", 95.0);
    acq_registry.set_channel("temperature", 160.0);

    let robot_obs = SharedRobotObservation::new(RobotState {
        joints: vec![15.0, 30.0, 0.0],
        velocities: vec![0.0, 0.0, 0.0],
    });

    // 4. Intent de ejecución (`ExecutionTarget`)
    let target = ExecutionTarget {
        station_id: station_id.clone(),
        robotics_module_id: robotics_mod_id.clone(),
    };

    // 5. Preparación transaccional (`StationService`)
    let (session_id, mut runner) = service
        .prepare_execution_session(
            &target,
            "weld_and_cool_program",
            ExecutionConfiguration::default(),
            acq_registry.clone(),
            robot_obs.clone(),
            &coordinator,
        )
        .expect("Preparación transaccional debe ser exitosa");

    // 6. Iniciar ciclo de vida de la sesión
    coordinator.initialize(&session_id).unwrap();
    coordinator.start(&session_id).unwrap();

    // 7. Evaluar Tick de Control 1 con el TelemetryExecutionRunner preparado
    let eval_logic = |acq: &AcquisitionSnapshot, rob: &RobotState| {
        let tx = acq.channels.get("target_x").copied().unwrap_or(0.0);
        let temp = acq.channels.get("temperature").copied().unwrap_or(0.0);
        let j0 = rob.joints.first().copied().unwrap_or(0.0);

        if tx > 80.0 && temp > 150.0 && j0 > 10.0 {
            (
                Decision::MotionAction {
                    motion_type: "cool_weld".to_string(),
                    target_name: "safe_pose".to_string(),
                },
                Action::DispatchMotion {
                    kind: "cool_weld".to_string(),
                    target: "safe_pose".to_string(),
                },
            )
        } else {
            (Decision::Continue, Action::None)
        }
    };

    let tick_res = coordinator
        .tick_with_runner(&session_id, &mut runner, eval_logic)
        .unwrap();

    assert_eq!(tick_res.tick.index, 1);
    assert_eq!(
        tick_res.decision,
        Decision::MotionAction {
            motion_type: "cool_weld".to_string(),
            target_name: "safe_pose".to_string()
        }
    );
    assert_eq!(tick_res.outcome, TickOutcome::Success);

    // Finalizar sesión
    coordinator.stop(&session_id).unwrap();

    // 8. Verificar reconstrucción completa de ExecutionHistory
    let history: ExecutionHistory = history_store
        .get_history(&session_id)
        .expect("La historia debe haberse reconstruido en el bus");

    assert_eq!(history.session_id, session_id);
    assert_eq!(history.program_id, "weld_and_cool_program");
    assert_eq!(history.final_lifecycle, LifecycleState::Stopped);
    assert_eq!(history.ticks.len(), 1);
    assert_eq!(
        history.ticks[0].result.decision,
        Decision::MotionAction {
            motion_type: "cool_weld".to_string(),
            target_name: "safe_pose".to_string()
        }
    );
}

#[test]
fn test_station_service_transactional_failure_on_invalid_target() {
    let coordinator = DomainExecutionCoordinator::new();
    let service = StationService::new();

    // Estación A
    let station_a_id = StationId("cell_a".to_string());
    let mut station_a = Station::new("cell_a", "Cell A");

    let module_a_id = RoboticsModuleId("robot_a".to_string());
    station_a.add_robotics_module(RoboticsModule {
        id: module_a_id.clone(),
        station_id: station_a_id.clone(),
        name: "Robot A".to_string(),
        robot_name: "KUKA-01".to_string(),
        controller_binding: "sim".to_string(),
    });

    service.register_station(station_a);

    // Intent con RoboticsModule inexistente -> debe fallar transaccionalmente
    let invalid_target = ExecutionTarget {
        station_id: station_a_id.clone(),
        robotics_module_id: RoboticsModuleId("non_existent".to_string()),
    };

    let acq_registry = InMemoryAcquisitionRegistry::new();
    let robot_obs = SharedRobotObservation::new(RobotState::default());

    let err = service
        .prepare_execution_session(
            &invalid_target,
            "failed_program",
            ExecutionConfiguration::default(),
            acq_registry,
            robot_obs,
            &coordinator,
        )
        .unwrap_err();

    assert_eq!(
        err,
        StationServiceError::RoboticsModuleNotFound(RoboticsModuleId("non_existent".to_string()))
    );

    // Verificar que NINGUNA sesión fue creada en el registro del coordinador
    assert!(coordinator.registry.list_sessions().is_empty());
}
