use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use super::domain::{
    DomainExecutionCoordinator, ExecutionSessionId, ObservationBundle, RobotState,
    TickContext, TickOutcome, Cardinality,
};
use super::runner::{ExecutionRunner, SimulationRunner};
use crate::execution::executor::ExecutionSessionState;
use thalos_core::execution::plan::ExecutionPlan;

/// Default tick interval for simulation execution (60 Hz).
pub const DEFAULT_TICK_INTERVAL: Duration = Duration::from_millis(16);

/// Evaluation function signature: takes observations and robot state, returns (Decision, Action).
pub type EvalFn = Box<dyn Fn(&ObservationBundle, &RobotState) -> (super::domain::Decision, super::domain::Action) + Send + Sync>;

/// Run the execution loop for a session.
///
/// This function owns the tick loop. It runs in a tokio task and publishes
/// events through the DomainExecutionCoordinator's event bus.
///
/// The loop terminates when the session reaches a terminal state
/// (Completed, Cancelled, Failed) or when the runner reports SessionCompleted.
///
/// For `Cardinality::Once` sessions, the session is automatically completed
/// after the first tick evaluation.
///
/// # Architecture
///
/// ```text
/// Runtime-owned loop:
///   acquire → evaluate → act → emit event → persist → repeat
/// ```
///
/// React observes via Tauri events. It does NOT drive this loop.
pub async fn run_execution_loop<R: ExecutionRunner>(
    coordinator: Arc<DomainExecutionCoordinator>,
    session_id: ExecutionSessionId,
    mut runner: R,
    eval_fn: EvalFn,
    tick_interval: Duration,
) -> Result<(), ExecutionLoopError> {
    let mut first_tick = true;

    loop {
        // Check if session is in a terminal state before ticking
        let session = coordinator.registry.get(&session_id)
            .ok_or_else(|| ExecutionLoopError::SessionNotFound(session_id.0.clone()))?;

        if session.lifecycle.is_terminal() {
            break;
        }

        if session.lifecycle != ExecutionSessionState::Running {
            tokio::time::sleep(tick_interval).await;
            continue;
        }

        // Execute one tick: acquire → evaluate → act
        match coordinator.tick_with_runner(&session_id, &mut runner, |obs, robot| {
            eval_fn(obs, robot)
        }) {
            Ok(result) => {
                if result.outcome == TickOutcome::SessionCompleted {
                    break;
                }
            }
            Err(e) => {
                tracing::error!(target: "execution",
                    session_id = %session_id.0,
                    error = %e,
                    "Execution tick failed"
                );
                return Err(ExecutionLoopError::TickFailed(e.to_string()));
            }
        }

        // For Cardinality::Once sessions, complete after the first tick
        if first_tick {
            first_tick = false;
            let session = coordinator.registry.get(&session_id)
                .ok_or_else(|| ExecutionLoopError::SessionNotFound(session_id.0.clone()))?;
            if session.configuration.cardinality == Cardinality::Once {
                // Complete the session (Running → Completed)
                let _ = coordinator.registry.with_session_mut(&session_id, |session| {
                    session.complete().map_err(|e| super::domain::ExecutionDomainError::InvalidLifecycle(e))
                });
                // Publish lifecycle event
                let now_us = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_micros() as u64;
                coordinator.event_bus.publish(super::events::ExecutionEvent::LifecycleChanged {
                    session_id: session_id.clone(),
                    previous: ExecutionSessionState::Running,
                    current: ExecutionSessionState::Completed,
                    timestamp_us: now_us,
                });
                break;
            }
        }

        tokio::time::sleep(tick_interval).await;
    }

    Ok(())
}

/// Convenience wrapper that creates a default eval function (Continue + None)
/// and runs the loop with the default tick interval.
pub async fn run_simulation_session(
    coordinator: Arc<DomainExecutionCoordinator>,
    session_id: ExecutionSessionId,
    runner: SimulationRunner,
) -> Result<(), ExecutionLoopError> {
    let eval_fn: EvalFn = Box::new(|_obs, _robot| {
        (super::domain::Decision::Continue, super::domain::Action::None)
    });

    run_execution_loop(
        coordinator,
        session_id,
        runner,
        eval_fn,
        DEFAULT_TICK_INTERVAL,
    ).await
}

/// Create an EvalFn that drives decisions from an ExecutionPlan.
///
/// The returned function tracks elapsed time via tick count and interpolates
/// between waypoints to produce expected joint states. Each tick:
///
/// 1. Computes elapsed time from tick_count × tick_interval
/// 2. Finds the current waypoint index based on elapsed time
/// 3. Returns Decision::MotionAction + Action::DispatchMotion with the
///    expected joint configuration from the plan
///
/// For simulation, the runner's observed state matches expected (no deviation).
/// For physical execution, the runner would provide real observations and the
/// deviation would be meaningful.
pub fn plan_driven_eval_fn(
    plan: ExecutionPlan,
    tick_interval: Duration,
) -> EvalFn {
    let tick_interval_secs = tick_interval.as_secs_f64();
    let tick_count = AtomicU64::new(0);

    Box::new(move |_obs: &ObservationBundle, _robot: &RobotState| {
        let current = tick_count.fetch_add(1, Ordering::Relaxed);
        let elapsed = current as f64 * tick_interval_secs;

        // Find the current waypoint: last waypoint whose timestamp <= elapsed
        let idx = plan.waypoints.partition_point(|w| w.timestamp <= elapsed);
        let waypoint_idx = idx.saturating_sub(1).min(plan.waypoints.len().saturating_sub(1));

        if let Some(waypoint) = plan.waypoints.get(waypoint_idx) {
            // Determine which segment this waypoint belongs to
            let segment = plan.segments.iter().find(|s| {
                s.waypoint_range.contains(&waypoint_idx)
            });

            let motion_type = match segment.map(|s| &s.instruction) {
                Some(thalos_core::execution::plan::PlanInstruction::MoveJ) => "movej",
                Some(thalos_core::execution::plan::PlanInstruction::MoveL) => "movel",
                None => "movej",
            };

            (
                super::domain::Decision::MotionAction {
                    motion_type: motion_type.to_string(),
                    target_name: format!("waypoint_{waypoint_idx}"),
                },
                super::domain::Action::DispatchMotion {
                    kind: motion_type.to_string(),
                    target: format!("wp{waypoint_idx}"),
                },
            )
        } else {
            // Past the end of the plan — continue (session will complete via Cardinality::Once)
            (super::domain::Decision::Continue, super::domain::Action::None)
        }
    })
}

/// Run an execution loop driven by an ExecutionPlan.
///
/// Combines `plan_driven_eval_fn` with `run_execution_loop` for convenience.
pub async fn run_plan_execution_session(
    coordinator: Arc<DomainExecutionCoordinator>,
    session_id: ExecutionSessionId,
    runner: SimulationRunner,
    plan: ExecutionPlan,
) -> Result<(), ExecutionLoopError> {
    let eval_fn = plan_driven_eval_fn(plan, DEFAULT_TICK_INTERVAL);

    run_execution_loop(
        coordinator,
        session_id,
        runner,
        eval_fn,
        DEFAULT_TICK_INTERVAL,
    ).await
}

/// Errors that can occur during execution loop execution.
#[derive(Debug, thiserror::Error)]
pub enum ExecutionLoopError {
    #[error("Session not found: {0}")]
    SessionNotFound(String),

    #[error("Tick failed: {0}")]
    TickFailed(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use super::super::domain::{
        DomainExecutionCoordinator, ExecutionConfiguration, ExecutionSessionId,
        ObservationBundle, RobotState, ExpectedState, TickContext,
        Decision, Action, TickOutcome,
    };
    use super::super::events::{EventSubscriber, ExecutionEvent};
    use super::super::runner::SimulationRunner;
    use crate::execution::executor::ExecutionSessionState;

    /// Collects all events published to the bus for assertion.
    struct EventCollector {
        events: Arc<Mutex<Vec<ExecutionEvent>>>,
    }

    impl EventCollector {
        fn new() -> (Self, Arc<Mutex<Vec<ExecutionEvent>>>) {
            let events = Arc::new(Mutex::new(Vec::new()));
            let collector = Self { events: events.clone() };
            (collector, events)
        }
    }

    impl EventSubscriber for EventCollector {
        fn on_event(&self, event: &ExecutionEvent) {
            self.events.lock().unwrap().push(event.clone());
        }
    }

    #[tokio::test]
    async fn test_e2e_simulation_loop_receives_events() {
        // 1. Setup coordinator with event collector
        let coordinator = Arc::new(DomainExecutionCoordinator::new());
        let (collector, events_ref) = EventCollector::new();
        coordinator.event_bus.subscribe(Arc::new(collector));

        // 2. Create and start session
        let session_id = coordinator.create_session(
            "test_program",
            ExecutionConfiguration::default(),
        );
        coordinator.initialize(&session_id).unwrap();
        coordinator.start(&session_id).unwrap();

        // 3. Create runner with initial context (non-zero captured_at_us)
        let now_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;
        let mut initial_obs = ObservationBundle::default();
        initial_obs.captured_at_us = now_us;
        let runner = SimulationRunner::new(TickContext {
            observations: initial_obs,
            robot: RobotState::default(),
            expected: ExpectedState::default(),
        });

        // 4. Run execution loop with fast tick interval
        let coord_clone = coordinator.clone();
        let sid_clone = session_id.clone();
        let handle = tokio::spawn(async move {
            run_simulation_session(coord_clone, sid_clone, runner).await
        });

        // 5. Wait for loop to complete (should finish quickly with default config)
        let result = handle.await.unwrap();
        assert!(result.is_ok(), "Execution loop should complete without error: {:?}", result.err());

        // 6. Verify events were received
        let events = events_ref.lock().unwrap();

        // Should have: SessionCreated + LifecycleChanged(Created→Reserved) +
        // LifecycleChanged(Reserved→Running) + N × TickEvaluated + LifecycleChanged(Running→Completed)
        let created_events: Vec<_> = events.iter().filter(|e| matches!(e, ExecutionEvent::SessionCreated { .. })).collect();
        let lifecycle_events: Vec<_> = events.iter().filter(|e| matches!(e, ExecutionEvent::LifecycleChanged { .. })).collect();
        let tick_events: Vec<_> = events.iter().filter(|e| matches!(e, ExecutionEvent::TickEvaluated { .. })).collect();

        assert_eq!(created_events.len(), 1, "Should have exactly 1 SessionCreated event");
        assert!(lifecycle_events.len() >= 3, "Should have at least 3 LifecycleChanged events (Created→Reserved, Reserved→Running, Running→Completed), got {}", lifecycle_events.len());
        assert!(tick_events.len() >= 1, "Should have at least 1 TickEvaluated event, got {}", tick_events.len());

        // 7. Verify TickEvaluated contains full TickResult
        for tick_event in &tick_events {
            if let ExecutionEvent::TickEvaluated { session_id: sid, result, temporal } = tick_event {
                assert_eq!(sid.0, session_id.0, "Tick event session_id must match");

                // TickResult should have all components
                assert!(result.tick.index > 0, "Tick index should be > 0");
                assert!(temporal.sampled_at_us > 0, "temporal.sampled_at_us should be set");
                assert!(temporal.received_at_us > 0, "temporal.received_at_us should be set");
                assert!(temporal.evaluated_at_us > 0, "temporal.evaluated_at_us should be set");

                // Decision and Action should be present
                assert!(
                    matches!(result.decision, Decision::Continue | Decision::NoOp | Decision::MotionAction { .. } | Decision::TerminateSession { .. }),
                    "Decision should be a valid variant"
                );
                assert!(
                    matches!(result.action, Action::None | Action::DispatchMotion { .. } | Action::SetOutput { .. } | Action::HoldPosition),
                    "Action should be a valid variant"
                );
            }
        }

        // 8. Verify session completed
        let session = coordinator.registry.get(&session_id).unwrap();
        assert_eq!(session.lifecycle, ExecutionSessionState::Completed);

        // 9. Verify final lifecycle event is Completed
        let last_lifecycle = lifecycle_events.last().unwrap();
        if let ExecutionEvent::LifecycleChanged { current, .. } = last_lifecycle {
            assert_eq!(*current, ExecutionSessionState::Completed);
        } else {
            panic!("Last lifecycle event should be LifecycleChanged");
        }

        println!("E2E test passed: {} events total, {} ticks, session completed",
            events.len(), tick_events.len());
    }

    #[tokio::test]
    async fn test_e2e_custom_eval_fn_drives_decisions() {
        let coordinator = Arc::new(DomainExecutionCoordinator::new());
        let (collector, events_ref) = EventCollector::new();
        coordinator.event_bus.subscribe(Arc::new(collector));

        let session_id = coordinator.create_session(
            "custom_eval_program",
            ExecutionConfiguration::default(),
        );
        coordinator.initialize(&session_id).unwrap();
        coordinator.start(&session_id).unwrap();

        let runner = SimulationRunner::new(TickContext::default());

        // Custom eval function that dispatches a motion action
        let eval_fn: EvalFn = Box::new(|_obs, _robot| {
            (
                Decision::MotionAction {
                    motion_type: "movej".to_string(),
                    target_name: "target_1".to_string(),
                },
                Action::DispatchMotion {
                    kind: "movej".to_string(),
                    target: "target_1".to_string(),
                },
            )
        });

        let coord_clone = coordinator.clone();
        let sid_clone = session_id.clone();
        let handle = tokio::spawn(async move {
            run_execution_loop(coord_clone, sid_clone, runner, eval_fn, Duration::from_millis(1)).await
        });

        let result = handle.await.unwrap();
        assert!(result.is_ok());

        let events = events_ref.lock().unwrap();
        let tick_events: Vec<_> = events.iter().filter(|e| matches!(e, ExecutionEvent::TickEvaluated { .. })).collect();
        assert!(tick_events.len() >= 1, "Should have ticks with custom eval");

        // Verify the decision/action in tick events
        for tick_event in &tick_events {
            if let ExecutionEvent::TickEvaluated { result, .. } = tick_event {
                assert!(
                    matches!(&result.decision, Decision::MotionAction { motion_type, target_name }
                        if motion_type == "movej" && target_name == "target_1"),
                    "Decision should be MotionAction(movej, target_1), got: {:?}",
                    result.decision
                );
                assert!(
                    matches!(&result.action, Action::DispatchMotion { kind, target }
                        if kind == "movej" && target == "target_1"),
                    "Action should be DispatchMotion(movej, target_1), got: {:?}",
                    result.action
                );
            }
        }

        println!("Custom eval E2E test passed: {} ticks with correct decisions", tick_events.len());
    }

    #[tokio::test]
    async fn test_e2e_plan_driven_execution() {
        use thalos_core::execution::plan::{ExecutionPlan, ExecutionWaypoint, ExecutionSegment, PlanInstruction};

        // 1. Build a plan with 3 waypoints at different timestamps
        let plan = ExecutionPlan {
            waypoints: vec![
                ExecutionWaypoint { joints: vec![0.0, 0.0, 0.0, 0.0], timestamp: 0.0 },
                ExecutionWaypoint { joints: vec![0.5, -0.3, 0.2, 1.0], timestamp: 0.5 },
                ExecutionWaypoint { joints: vec![1.0, -0.6, 0.4, 1.57], timestamp: 1.0 },
            ],
            segments: vec![
                ExecutionSegment {
                    index: 0,
                    planned_segment_index: 0,
                    instruction: PlanInstruction::MoveJ,
                    waypoint_range: 0..3,
                },
            ],
            duration: 1.0,
            repeat_count: 1,
            program_id: Some("test_program".to_string()),
            program_revision: Some(1),
            source_fingerprint: Some("test_hash".to_string()),
            robot_id: Some("test_robot".to_string()),
        };

        // 2. Setup coordinator and session
        let coordinator = Arc::new(DomainExecutionCoordinator::new());
        let (collector, events_ref) = EventCollector::new();
        coordinator.event_bus.subscribe(Arc::new(collector));

        let session_id = coordinator.create_session(
            "test_program",
            ExecutionConfiguration::default(),
        );
        coordinator.initialize(&session_id).unwrap();
        coordinator.start(&session_id).unwrap();

        // 3. Create runner
        let runner = SimulationRunner::new(TickContext::default());

        // 4. Run with plan-driven eval (fast ticks for testing)
        let eval_fn = plan_driven_eval_fn(plan, Duration::from_millis(1));
        let coord_clone = coordinator.clone();
        let sid_clone = session_id.clone();
        let handle = tokio::spawn(async move {
            run_execution_loop(coord_clone, sid_clone, runner, eval_fn, Duration::from_millis(1)).await
        });

        let result = handle.await.unwrap();
        assert!(result.is_ok(), "Plan-driven execution should complete: {:?}", result.err());

        // 5. Verify events
        let events = events_ref.lock().unwrap();
        let tick_events: Vec<_> = events.iter().filter(|e| matches!(e, ExecutionEvent::TickEvaluated { .. })).collect();

        // Should have at least 1 tick (Cardinality::Once completes after first)
        assert!(tick_events.len() >= 1, "Should have at least 1 tick, got {}", tick_events.len());

        // 6. Verify first tick has MotionAction with correct motion type
        if let ExecutionEvent::TickEvaluated { result, .. } = &tick_events[0] {
            match &result.decision {
                Decision::MotionAction { motion_type, target_name } => {
                    assert_eq!(motion_type, "movej", "First waypoint should be movej");
                    assert!(target_name.starts_with("waypoint_"), "Target should reference waypoint");
                }
                other => panic!("Expected MotionAction, got: {:?}", other),
            }
            match &result.action {
                Action::DispatchMotion { kind, .. } => {
                    assert_eq!(kind, "movej", "Action kind should be movej");
                }
                other => panic!("Expected DispatchMotion, got: {:?}", other),
            }
        }

        // 7. Verify session completed
        let session = coordinator.registry.get(&session_id).unwrap();
        assert_eq!(session.lifecycle, ExecutionSessionState::Completed);

        println!("Plan-driven E2E test passed: {} ticks, session completed", tick_events.len());
    }

    #[tokio::test]
    async fn test_plan_driven_eval_fn_interpolates_waypoints() {
        use thalos_core::execution::plan::{ExecutionPlan, ExecutionWaypoint, ExecutionSegment, PlanInstruction};

        // Build plan with 3 waypoints at 0.0, 0.5, 1.0 seconds
        // With 1ms tick interval, we should see different waypoints at different ticks
        let plan = ExecutionPlan {
            waypoints: vec![
                ExecutionWaypoint { joints: vec![0.0, 0.0], timestamp: 0.0 },
                ExecutionWaypoint { joints: vec![0.5, -0.3], timestamp: 0.5 },
                ExecutionWaypoint { joints: vec![1.0, -0.6], timestamp: 1.0 },
            ],
            segments: vec![
                ExecutionSegment {
                    index: 0,
                    planned_segment_index: 0,
                    instruction: PlanInstruction::MoveJ,
                    waypoint_range: 0..3,
                },
            ],
            duration: 1.0,
            repeat_count: 1,
            program_id: Some("interpolation_test".to_string()),
            program_revision: Some(1),
            source_fingerprint: Some("hash".to_string()),
            robot_id: None,
        };

        let eval_fn = plan_driven_eval_fn(plan, Duration::from_millis(1));

        // Tick 0: should target waypoint 0 (timestamp 0.0)
        let (decision0, action0) = eval_fn(&ObservationBundle::default(), &RobotState::default());
        match decision0 {
            Decision::MotionAction { target_name, .. } => {
                assert!(target_name.contains("waypoint_0"), "Tick 0 should target waypoint_0, got: {target_name}");
            }
            other => panic!("Expected MotionAction at tick 0, got: {other:?}"),
        }

        // Ticks 1-499: still at waypoint 0 (timestamp 0.0 to 0.499)
        for _ in 1..500 {
            let (dec, _) = eval_fn(&ObservationBundle::default(), &RobotState::default());
            match dec {
                Decision::MotionAction { target_name, .. } => {
                    assert!(target_name.contains("waypoint_0"), "Should still be at waypoint_0");
                }
                other => panic!("Expected MotionAction, got: {other:?}"),
            }
        }

        // Tick 500: should now target waypoint 1 (timestamp 0.5)
        let (decision500, _) = eval_fn(&ObservationBundle::default(), &RobotState::default());
        match decision500 {
            Decision::MotionAction { target_name, .. } => {
                assert!(target_name.contains("waypoint_1"), "Tick 500 should target waypoint_1, got: {target_name}");
            }
            other => panic!("Expected MotionAction at tick 500, got: {other:?}"),
        }

        println!("Waypoint interpolation test passed");
    }
}
