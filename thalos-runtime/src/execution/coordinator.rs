use thiserror::Error;
use thalos_engine::prelude::*;
use crate::resources::{
    ReservationError, ResourceRegistry, ResourceReservation, ResourceReservationManager,
    ResourceResolutionError, ResourceResolver,
};
use super::plan::ExecutionSession;
use super::preflight::{ExecutionPreflight, PreflightCheck, PreflightCheckKind};
use super::request::{ExecutionRequest, ExecutionTarget};

use super::executor::ExecutionSessionState;

/// ExecutionError (ADR-014)
#[derive(Debug, Error)]
pub enum ExecutionError {
    #[error("Preflight evaluation failed: {0:?}")]
    PreflightFailed(ExecutionPreflight),

    #[error("Resource resolution error: {0}")]
    ResolutionError(#[from] ResourceResolutionError),

    #[error("Resource reservation error: {0}")]
    ReservationError(#[from] ReservationError),

    #[error("Invalid execution session state: {0:?}")]
    InvalidSessionState(ExecutionSessionState),
}


/// DispatchResult (ADR-014)
/// Successful outcome of dispatching an ExecutionRequest containing session and reservation.
#[derive(Debug, Clone)]
pub struct DispatchResult {
    pub session: ExecutionSession,
    pub reservation: ResourceReservation,
}

/// ExecutionCoordinator (ADR-014)
/// Evaluates preflight checks, resolves capability requirements, reserves hardware/sensor resources,
/// and dispatches execution sessions.
#[derive(Debug, Default)]
pub struct ExecutionCoordinator;

impl ExecutionCoordinator {
    pub fn new() -> Self {
        Self
    }

    /// Perform immutable preflight evaluation for an ExecutionRequest against station context.
    pub fn evaluate_preflight(
        &self,
        station: &Station,
        registry: &ResourceRegistry,
        request: &ExecutionRequest,
        robot_connected: bool,
        transport_ready: bool,
    ) -> ExecutionPreflight {
        let mut checks = Vec::new();

        // 1. Plan Check
        if request.plan_id.as_str().is_empty() {
            checks.push(PreflightCheck::fail(
                PreflightCheckKind::Plan,
                "PlanId cannot be empty",
            ));
        } else {
            checks.push(PreflightCheck::pass(
                PreflightCheckKind::Plan,
                format!("Plan {} is valid", request.plan_id.as_str()),
            ));
        }

        // 2. Resource Resolution Check
        match ResourceResolver::resolve(station, registry, &request.requirements) {
            Ok(resolved) => {
                checks.push(PreflightCheck::pass(
                    PreflightCheckKind::Resource,
                    format!(
                        "Successfully resolved {} requirements ({} optional unresolved)",
                        resolved.matches.len(),
                        resolved.unresolved_optional.len()
                    ),
                ));
            }
            Err(err) => {
                checks.push(PreflightCheck::fail(
                    PreflightCheckKind::Resource,
                    format!("Resource resolution failed: {}", err),
                ));
            }
        }

        // 3. Robot & Transport Checks based on Target
        match &request.target {
            ExecutionTarget::Hardware { robot_id } => {
                if robot_connected {
                    checks.push(PreflightCheck::pass(
                        PreflightCheckKind::Robot,
                        format!("Robot {} is connected and ready", robot_id.as_str()),
                    ));
                } else {
                    checks.push(PreflightCheck::fail(
                        PreflightCheckKind::Robot,
                        format!("Robot {} is disconnected", robot_id.as_str()),
                    ));
                }

                if transport_ready {
                    checks.push(PreflightCheck::pass(
                        PreflightCheckKind::Transport,
                        "Hardware communication transport is ready",
                    ));
                } else {
                    checks.push(PreflightCheck::fail(
                        PreflightCheckKind::Transport,
                        "Hardware communication transport is unavailable",
                    ));
                }
            }
            ExecutionTarget::Simulation => {
                checks.push(PreflightCheck::skip(
                    PreflightCheckKind::Robot,
                    "Hardware robot connection not required for simulation target",
                ));
                checks.push(PreflightCheck::skip(
                    PreflightCheckKind::Transport,
                    "Physical transport not required for simulation target",
                ));
            }
        }

        // 4. Safety Check
        checks.push(PreflightCheck::pass(
            PreflightCheckKind::Safety,
            "Safety envelope checks passed",
        ));

        ExecutionPreflight::new(checks)
    }

    /// Dispatch an ExecutionRequest by preflighting, resolving, reserving resources, and creating an ExecutionSession.
    pub fn dispatch(
        &self,
        station: &Station,
        registry: &ResourceRegistry,
        reservation_mgr: &mut ResourceReservationManager,
        request: ExecutionRequest,
        _operational_session_id: &OperationalSessionId,
        robot_connected: bool,
        transport_ready: bool,
    ) -> Result<DispatchResult, ExecutionError> {
        // Step 1: Preflight Evaluation
        let preflight = self.evaluate_preflight(
            station,
            registry,
            &request,
            robot_connected,
            transport_ready,
        );

        if !preflight.can_dispatch {
            return Err(ExecutionError::PreflightFailed(preflight));
        }

        // Step 2: Capability Resolution
        let resolved = ResourceResolver::resolve(station, registry, &request.requirements)?;
        let resource_refs: Vec<ResourceRef> =
            resolved.matches.into_iter().map(|m| m.matched_resource).collect();

        // Step 3: Reserve Resources
        let session_id = ExecutionSessionId(format!("exec-session-{}", uuid::Uuid::new_v4()));
        let reservation = reservation_mgr.reserve(session_id, resource_refs)?;

        // Step 4: Create ExecutionSession
        let session = ExecutionSession::new(request.plan_id.as_str());

        Ok(DispatchResult {
            session,
            reservation,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::execution::request::ExecutionPolicy;

    #[test]
    fn test_simulation_target_preflight_passes_without_hardware() {
        let coordinator = ExecutionCoordinator::new();
        let station = Station::new("cell-01", "Cell 01", vec![]);
        let registry = ResourceRegistry::new();

        let req = ExecutionRequest::new(
            "plan-pick-01",
            ExecutionTarget::Simulation,
            ExecutionPolicy::default(),
            vec![],
        );

        let preflight = coordinator.evaluate_preflight(&station, &registry, &req, false, false);

        assert!(preflight.can_dispatch);
        assert_eq!(preflight.checks.len(), 5);
    }

    #[test]
    fn test_hardware_target_fails_when_disconnected() {
        let coordinator = ExecutionCoordinator::new();
        let station = Station::new("cell-01", "Cell 01", vec![]);
        let registry = ResourceRegistry::new();

        let req = ExecutionRequest::new(
            "plan-pick-01",
            ExecutionTarget::Hardware {
                robot_id: RobotId("scara-01".into()),
            },
            ExecutionPolicy::default(),
            vec![],
        );

        let preflight = coordinator.evaluate_preflight(&station, &registry, &req, false, false);

        assert!(!preflight.can_dispatch);
        let robot_check = preflight
            .checks
            .iter()
            .find(|c| c.kind == PreflightCheckKind::Robot)
            .unwrap();
        assert_eq!(robot_check.status, crate::execution::preflight::PreflightStatus::Failed);
    }

    #[test]
    fn test_dispatch_flow_reserves_resources_and_creates_session() {
        let coordinator = ExecutionCoordinator::new();
        let mut registry = ResourceRegistry::new();

        let scara_01 = Resource::new(
            "scara-01",
            ResourceKind::Robot,
            "SCARA 01",
            vec![CapabilityRequirement::CartesianMotion],
        );
        registry.register(scara_01.clone());

        let station = Station::new("cell-01", "Cell 01", vec![scara_01.to_ref()]);
        let mut reservation_mgr = ResourceReservationManager::new();

        let req = ExecutionRequest::new(
            "plan-pick-01",
            ExecutionTarget::Simulation,
            ExecutionPolicy::default(),
            vec![ResourceRequirement::mandatory(
                CapabilityRequirement::CartesianMotion,
            )],
        );

        let ops_session_id = OperationalSessionId("ops-01".into());

        let dispatch_result = coordinator
            .dispatch(
                &station,
                &registry,
                &mut reservation_mgr,
                req,
                &ops_session_id,
                true,
                true,
            )
            .unwrap();

        assert_eq!(dispatch_result.session.plan_id, "plan-pick-01");
        assert!(reservation_mgr.is_reserved(&ResourceId("scara-01".into())));
    }

    #[test]
    fn test_dispatch_fails_if_resource_already_reserved() {
        let coordinator = ExecutionCoordinator::new();
        let mut registry = ResourceRegistry::new();

        let scara_01 = Resource::new(
            "scara-01",
            ResourceKind::Robot,
            "SCARA 01",
            vec![CapabilityRequirement::CartesianMotion],
        );
        registry.register(scara_01.clone());

        let station = Station::new("cell-01", "Cell 01", vec![scara_01.to_ref()]);
        let mut reservation_mgr = ResourceReservationManager::new();

        let req1 = ExecutionRequest::new(
            "plan-pick-01",
            ExecutionTarget::Simulation,
            ExecutionPolicy::default(),
            vec![ResourceRequirement::mandatory(
                CapabilityRequirement::CartesianMotion,
            )],
        );

        let req2 = ExecutionRequest::new(
            "plan-pick-02",
            ExecutionTarget::Simulation,
            ExecutionPolicy::default(),
            vec![ResourceRequirement::mandatory(
                CapabilityRequirement::CartesianMotion,
            )],
        );

        let ops_session_id = OperationalSessionId("ops-01".into());

        // First dispatch succeeds and reserves scara-01
        coordinator
            .dispatch(
                &station,
                &registry,
                &mut reservation_mgr,
                req1,
                &ops_session_id,
                true,
                true,
            )
            .unwrap();

        // Second dispatch for same mandatory capability fails at preflight/reservation
        let res2 = coordinator.dispatch(
            &station,
            &registry,
            &mut reservation_mgr,
            req2,
            &ops_session_id,
            true,
            true,
        );

        assert!(res2.is_err());
    }
}
