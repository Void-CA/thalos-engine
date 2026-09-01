use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use thalos_engine::prelude::*;

/// ReservationError (ADR-014)
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReservationError {
    #[error("Resource {0} is already reserved by active session")]
    ResourceAlreadyReserved(String),

    #[error("Reservation {0} not found")]
    ReservationNotFound(String),
}

/// ResourceReservation (ADR-014)
/// Immutable record of assigned hardware/sensor resources for an active session.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResourceReservation {
    pub id: ResourceReservationId,
    pub session_id: ExecutionSessionId,
    pub resources: Vec<ResourceRef>,
    pub created_at: String,
}

/// ResourceReservationManager (ADR-014)
/// Operational concurrency manager preventing simultaneous hardware/channel reservations.
#[derive(Debug, Default, Clone)]
pub struct ResourceReservationManager {
    reservations: HashMap<ResourceReservationId, ResourceReservation>,
}

impl ResourceReservationManager {
    pub fn new() -> Self {
        Self {
            reservations: HashMap::new(),
        }
    }

    /// Check if a specific ResourceId is currently reserved by any active session.
    pub fn is_reserved(&self, resource_id: &ResourceId) -> bool {
        self.reservations.values().any(|res| {
            res.resources
                .iter()
                .any(|r| r.id.as_str() == resource_id.as_str())
        })
    }

    /// Attempt to reserve a set of resources for a given session.
    pub fn reserve(
        &mut self,
        session_id: ExecutionSessionId,
        resources: Vec<ResourceRef>,
    ) -> Result<ResourceReservation, ReservationError> {
        // 1. Check for conflicts
        for r_ref in &resources {
            if self.is_reserved(&r_ref.id) {
                return Err(ReservationError::ResourceAlreadyReserved(
                    r_ref.id.as_str().to_string(),
                ));
            }
        }

        // 2. Create reservation
        let reservation_id = ResourceReservationId(format!("resv-{}", uuid::Uuid::new_v4()));
        let reservation = ResourceReservation {
            id: reservation_id.clone(),
            session_id,
            resources,
            created_at: chrono::Utc::now().to_rfc3339(),
        };

        self.reservations
            .insert(reservation_id, reservation.clone());
        Ok(reservation)
    }

    /// Release an active reservation.
    pub fn release(&mut self, id: &ResourceReservationId) -> Result<(), ReservationError> {
        if self.reservations.remove(id).is_some() {
            Ok(())
        } else {
            Err(ReservationError::ReservationNotFound(
                id.as_str().to_string(),
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reserve_free_resource_succeeds() {
        let mut mgr = ResourceReservationManager::new();
        let robot = ResourceRef::new("scara-01", ResourceKind::Robot);

        let resv = mgr
            .reserve(ExecutionSessionId("session-01".into()), vec![robot])
            .unwrap();

        assert!(mgr.is_reserved(&ResourceId("scara-01".into())));
        assert_eq!(resv.resources.len(), 1);
    }

    #[test]
    fn test_reserve_already_reserved_resource_fails() {
        let mut mgr = ResourceReservationManager::new();
        let robot = ResourceRef::new("scara-01", ResourceKind::Robot);

        mgr.reserve(ExecutionSessionId("session-01".into()), vec![robot.clone()])
            .unwrap();

        let res = mgr.reserve(ExecutionSessionId("session-02".into()), vec![robot]);

        assert_eq!(
            res,
            Err(ReservationError::ResourceAlreadyReserved("scara-01".into()))
        );
    }

    #[test]
    fn test_release_reservation_frees_resource() {
        let mut mgr = ResourceReservationManager::new();
        let robot = ResourceRef::new("scara-01", ResourceKind::Robot);

        let resv = mgr
            .reserve(ExecutionSessionId("session-01".into()), vec![robot])
            .unwrap();

        mgr.release(&resv.id).unwrap();
        assert!(!mgr.is_reserved(&ResourceId("scara-01".into())));
    }
}
