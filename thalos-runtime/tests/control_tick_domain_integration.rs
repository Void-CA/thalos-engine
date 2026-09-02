use thalos_runtime::execution::session::{
    Action, AcquisitionSnapshot, Decision, DomainExecutionCoordinator,
    DomainExecutionSession as ExecutionSession, Environment, EventSubscriber,
    ExecutionConfiguration, ExecutionDomainError, ExecutionEvent, ExecutionEventBus, ExpectedState,
    InMemoryAcquisitionRegistry, LifecycleState, PhysicalRunner, Reactivity, RobotState,
    SharedRobotObservation, SimulationRunner, TelemetryExecutionRunner, TerminationPolicy,
    TickContext, TickOutcome, TickResult,
};

#[test]
fn test_control_tick_deterministic_branching_and_invariants() {
    let config = ExecutionConfiguration {
        environment: Environment::VirtualSimulation,
        reactivity: Reactivity::Reactive,
        ..Default::default()
    };

    let mut session = ExecutionSession::new("reactive_branch_program", config);
    session.initialize().expect("must initialize");
    session.start().expect("must start");
    assert_eq!(session.lifecycle, LifecycleState::Running);

    let eval_logic = |acq: &AcquisitionSnapshot, _robot: &RobotState| {
        let target_x = acq.channels.get("camera.target_x").copied().unwrap_or(0.0);
        if target_x > 80.0 {
            (
                Decision::MotionAction {
                    motion_type: "move_to".to_string(),
                    target_name: "target_high".to_string(),
                },
                Action::DispatchMotion {
                    kind: "move_to".to_string(),
                    target: "target_high".to_string(),
                },
            )
        } else {
            (
                Decision::WaitAction { duration_secs: 1.0 },
                Action::HoldPosition,
            )
        }
    };

    // Tick 0: camera.target_x = 100 -> decision = move_to_target
    let mut acq0 = AcquisitionSnapshot::default();
    acq0.channels.insert("camera.target_x".to_string(), 100.0);
    let ctx0 = TickContext::new(acq0, RobotState::default(), ExpectedState::default());

    let res0: TickResult = session
        .evaluate_tick(ctx0, eval_logic)
        .expect("tick 0 must succeed");

    assert_eq!(res0.tick.index, 1);
    assert_eq!(
        res0.decision,
        Decision::MotionAction {
            motion_type: "move_to".to_string(),
            target_name: "target_high".to_string()
        }
    );
    assert_eq!(
        res0.action,
        Action::DispatchMotion {
            kind: "move_to".to_string(),
            target: "target_high".to_string()
        }
    );
    assert_eq!(res0.outcome, TickOutcome::Success);

    // Tick 1: camera.target_x = 50 -> decision = wait_for_signal (HoldPosition)
    let mut acq1 = AcquisitionSnapshot::default();
    acq1.channels.insert("camera.target_x".to_string(), 50.0);
    let ctx1 = TickContext::new(acq1, RobotState::default(), ExpectedState::default());

    let res1: TickResult = session
        .evaluate_tick(ctx1, eval_logic)
        .expect("tick 1 must succeed");

    assert_eq!(res1.tick.index, 2);
    assert_eq!(res1.decision, Decision::WaitAction { duration_secs: 1.0 });
    assert_eq!(res1.action, Action::HoldPosition);
    assert_eq!(res1.outcome, TickOutcome::Success);
}

#[test]
fn test_control_tick_multi_channel_latching_invariance() {
    let mut session = ExecutionSession::new("latching_program", ExecutionConfiguration::default());
    session.initialize().unwrap();
    session.start().unwrap();

    // Tick k: camera.target_x = 100, camera.target_y = 40
    let mut acq_k = AcquisitionSnapshot::default();
    acq_k.channels.insert("camera.target_x".to_string(), 100.0);
    acq_k.channels.insert("camera.target_y".to_string(), 40.0);
    let ctx_k = TickContext::new(acq_k.clone(), RobotState::default(), ExpectedState::default());

    let res_k = session
        .evaluate_tick(
            ctx_k,
            |acq, _robot| {
                let x = acq.channels.get("camera.target_x").copied().unwrap_or(0.0);
                let y = acq.channels.get("camera.target_y").copied().unwrap_or(0.0);
                assert_eq!(x, 100.0, "x must be latched to tick k snapshot");
                assert_eq!(y, 40.0, "y must be latched to tick k snapshot");
                (Decision::Continue, Action::None)
            },
        )
        .expect("tick k evaluation must succeed");

    // Invariante: res_k.tick contiene la captura inmutable del tick k
    assert_eq!(res_k.tick.acquisition, acq_k);
}

#[test]
fn test_control_tick_termination_policy_evaluation() {
    let config = ExecutionConfiguration {
        termination: TerminationPolicy::Condition("safety_stop".to_string()),
        ..Default::default()
    };

    let mut session = ExecutionSession::new("termination_test", config);
    session.initialize().unwrap();
    session.start().unwrap();

    let mut acq = AcquisitionSnapshot::default();
    acq.channels.insert("safety_stop".to_string(), 1.0);
    let ctx = TickContext::new(acq, RobotState::default(), ExpectedState::default());

    let res = session
        .evaluate_tick(
            ctx,
            |_acq, _robot| (Decision::Continue, Action::None),
        )
        .expect("eval tick must succeed");

    assert_eq!(
        res.decision,
        Decision::TerminateSession {
            reason: "Termination condition 'safety_stop' satisfied".to_string()
        }
    );
    assert_eq!(res.outcome, TickOutcome::SessionCompleted);
    assert_eq!(session.lifecycle, LifecycleState::Completed);
}

#[test]
fn test_coordinator_session_lifecycle_and_tick_rejection_when_paused() {
    let coordinator = DomainExecutionCoordinator::new();

    // 1. create_session
    let session_id = coordinator.create_session("cell_weld_routine", ExecutionConfiguration::default());
    let session = coordinator.registry.get(&session_id).unwrap();
    assert_eq!(session.lifecycle, LifecycleState::Created);

    // 2. initialize
    coordinator.initialize(&session_id).expect("initialize must succeed");
    let session = coordinator.registry.get(&session_id).unwrap();
    assert_eq!(session.lifecycle, LifecycleState::Initializing);

    // 3. start
    coordinator.start(&session_id).expect("start must succeed");
    let session = coordinator.registry.get(&session_id).unwrap();
    assert_eq!(session.lifecycle, LifecycleState::Running);

    let dummy_eval = |_acq: &AcquisitionSnapshot, _rob: &RobotState| (Decision::Continue, Action::None);

    // 4. tick 0 -> ok
    let ctx0 = TickContext::default();
    let res0 = coordinator.tick(&session_id, ctx0, dummy_eval).expect("tick 0 must succeed");
    assert_eq!(res0.tick.index, 1);

    // 5. tick 1 -> ok
    let ctx1 = TickContext::default();
    let res1 = coordinator.tick(&session_id, ctx1, dummy_eval).expect("tick 1 must succeed");
    assert_eq!(res1.tick.index, 2);

    // 6. pause
    coordinator.pause(&session_id).expect("pause must succeed");
    let session = coordinator.registry.get(&session_id).unwrap();
    assert_eq!(session.lifecycle, LifecycleState::Paused);

    // 7. tick 2 -> REJECTED (NotRunning(Paused))
    let ctx2 = TickContext::default();
    let err = coordinator.tick(&session_id, ctx2, dummy_eval).unwrap_err();
    assert_eq!(err, ExecutionDomainError::NotRunning(LifecycleState::Paused));

    // 8. start (resume)
    coordinator.start(&session_id).expect("resume must succeed");
    let session = coordinator.registry.get(&session_id).unwrap();
    assert_eq!(session.lifecycle, LifecycleState::Running);

    // 9. tick 2 -> ok
    let ctx3 = TickContext::default();
    let res3 = coordinator.tick(&session_id, ctx3, dummy_eval).expect("tick 2 must succeed");
    assert_eq!(res3.tick.index, 3);

    // 10. stop
    coordinator.stop(&session_id).expect("stop must succeed");
    let session = coordinator.registry.get(&session_id).unwrap();
    assert_eq!(session.lifecycle, LifecycleState::Stopped);

    // Verify full history
    assert_eq!(
        session.history,
        vec![
            LifecycleState::Created,
            LifecycleState::Initializing,
            LifecycleState::Running,
            LifecycleState::Paused,
            LifecycleState::Running,
            LifecycleState::Stopped,
        ]
    );
}

#[test]
fn test_same_program_runner_polymorphism_reproducibility() {
    let coordinator = DomainExecutionCoordinator::new();

    // Programa reactivo compartido: si camera.target_x > 80, dispatch move_to
    let eval_logic = |acq: &AcquisitionSnapshot, _rob: &RobotState| {
        let target_x = acq.channels.get("camera.target_x").copied().unwrap_or(0.0);
        if target_x > 80.0 {
            (
                Decision::MotionAction {
                    motion_type: "move_to".to_string(),
                    target_name: "target_high".to_string(),
                },
                Action::DispatchMotion {
                    kind: "move_to".to_string(),
                    target: "target_high".to_string(),
                },
            )
        } else {
            (
                Decision::WaitAction { duration_secs: 1.0 },
                Action::HoldPosition,
            )
        }
    };

    // ── Sesión A: SimulationRunner ──
    let session_a = coordinator.create_session(
        "reactive_pick",
        ExecutionConfiguration {
            environment: Environment::VirtualSimulation,
            reactivity: Reactivity::Reactive,
            ..Default::default()
        },
    );
    coordinator.initialize(&session_a).unwrap();
    coordinator.start(&session_a).unwrap();

    let mut acq_sim = AcquisitionSnapshot::default();
    acq_sim.channels.insert("camera.target_x".to_string(), 100.0);
    let mut sim_runner = SimulationRunner::new(TickContext::new(
        acq_sim,
        RobotState::default(),
        ExpectedState::default(),
    ));

    let res_a = coordinator
        .tick_with_runner(&session_a, &mut sim_runner, eval_logic)
        .unwrap();

    // ── Sesión B: PhysicalRunner (Disconnected hardware) ──
    let session_b = coordinator.create_session(
        "reactive_pick",
        ExecutionConfiguration {
            environment: Environment::Physical,
            reactivity: Reactivity::Reactive,
            ..Default::default()
        },
    );
    coordinator.initialize(&session_b).unwrap();
    coordinator.start(&session_b).unwrap();

    let mut acq_phys = AcquisitionSnapshot::default();
    acq_phys.channels.insert("camera.target_x".to_string(), 100.0);
    let mut phys_runner = PhysicalRunner::new(
        TickContext::new(acq_phys, RobotState::default(), ExpectedState::default()),
        false, // disconnected!
    );

    let res_b = coordinator
        .tick_with_runner(&session_b, &mut phys_runner, eval_logic)
        .unwrap();

    // ── Verificación de Invariante de Reproducibilidad ──
    // 1. Ambas sesiones produjeron exactamente la misma decisión semántica
    assert_eq!(res_a.decision, res_b.decision);
    assert_eq!(res_a.action, res_b.action);

    // 2. Ambas avanzaron el tick_index monotónicamente a 1
    assert_eq!(res_a.tick.index, 1);
    assert_eq!(res_b.tick.index, 1);

    // 3. El outcome difiere según la capacidad/estado del runner:
    //    SimulationRunner -> Success
    //    PhysicalRunner (desconectado) -> Faulted
    assert_eq!(res_a.outcome, TickOutcome::Success);
    assert_eq!(
        res_b.outcome,
        TickOutcome::Faulted("Physical hardware disconnected".to_string())
    );
}

#[test]
fn test_telemetry_execution_runner_dynamic_channel_and_robot_observation() {
    let coordinator = DomainExecutionCoordinator::new();

    let acq_registry = InMemoryAcquisitionRegistry::new();
    let robot_obs = SharedRobotObservation::new(RobotState {
        joints: vec![0.0, 0.0, 0.0],
        velocities: vec![0.0, 0.0, 0.0],
    });

    let mut runner = TelemetryExecutionRunner::new(
        acq_registry.clone(),
        robot_obs.clone(),
        ExpectedState::default(),
    );

    let session_id = coordinator.create_session(
        "telemetry_driven_program",
        ExecutionConfiguration::default(),
    );
    coordinator.initialize(&session_id).unwrap();
    coordinator.start(&session_id).unwrap();

    let eval_logic = |acq: &AcquisitionSnapshot, rob: &RobotState| {
        let temp = acq.channels.get("weld_head.temperature").copied().unwrap_or(0.0);
        let j0 = rob.joints.first().copied().unwrap_or(0.0);

        if temp > 150.0 && j0 > 45.0 {
            (
                Decision::MotionAction {
                    motion_type: "cool_down".to_string(),
                    target_name: "safe_home".to_string(),
                },
                Action::DispatchMotion {
                    kind: "cool_down".to_string(),
                    target: "safe_home".to_string(),
                },
            )
        } else {
            (Decision::Continue, Action::None)
        }
    };

    // Tick 1: temp = 100.0, j0 = 0.0 -> Continue
    acq_registry.set_channel("weld_head.temperature", 100.0);
    robot_obs.update(vec![0.0, 10.0, 0.0], vec![0.0, 0.0, 0.0]);

    let res1 = coordinator
        .tick_with_runner(&session_id, &mut runner, eval_logic)
        .unwrap();

    assert_eq!(res1.tick.index, 1);
    assert_eq!(res1.decision, Decision::Continue);
    assert_eq!(res1.action, Action::None);

    // Tick 2: temp = 180.0, j0 = 50.0 -> Cool down motion action
    acq_registry.set_channel("weld_head.temperature", 180.0);
    robot_obs.update(vec![50.0, 10.0, 0.0], vec![0.1, 0.0, 0.0]);

    let res2 = coordinator
        .tick_with_runner(&session_id, &mut runner, eval_logic)
        .unwrap();

    assert_eq!(res2.tick.index, 2);
    assert_eq!(
        res2.decision,
        Decision::MotionAction {
            motion_type: "cool_down".to_string(),
            target_name: "safe_home".to_string()
        }
    );
    assert_eq!(
        res2.action,
        Action::DispatchMotion {
            kind: "cool_down".to_string(),
            target: "safe_home".to_string()
        }
    );
    assert_eq!(res2.outcome, TickOutcome::Success);

    // Verificar que los snapshots del tick k fueron aislados inmutablemente
    assert_eq!(
        res1.tick.acquisition.channels.get("weld_head.temperature"),
        Some(&100.0)
    );
    assert_eq!(
        res2.tick.acquisition.channels.get("weld_head.temperature"),
        Some(&180.0)
    );
    assert_eq!(res1.tick.robot.joints, vec![0.0, 10.0, 0.0]);
    assert_eq!(res2.tick.robot.joints, vec![50.0, 10.0, 0.0]);
}
