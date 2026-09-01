use std::sync::Arc;

use tokio::sync::RwLock;

use super::controller::RobotController;
use crate::error::ControllerError;
use crate::session::execution_source::ExecutionSource;

/// An available execution backend.
#[derive(Clone)]
pub struct BackendEntry {
    pub id: String,
    pub name: String,
    pub controller: Option<Arc<RwLock<dyn RobotController + Send + Sync>>>,
    pub port: Option<String>,
}

/// Infrastructure layer that owns controller connections and lifecycle.
///
/// Lives ABOVE the runtime: `SceneService → BackendManager → Runtime → RobotController`.
/// The runtime does NOT know about connection management — it obtains the
/// active controller through the manager.
pub struct BackendManager {
    active: RwLock<Option<Arc<RwLock<dyn RobotController + Send + Sync>>>>,
    /// Id of the active backend entry.
    active_id: RwLock<Option<String>>,
    /// All registered backends.
    registered: RwLock<Vec<BackendEntry>>,
}

impl BackendManager {
    pub fn new() -> Self {
        Self {
            active: RwLock::new(None),
            active_id: RwLock::new(None),
            registered: RwLock::new(Vec::new()),
        }
    }

    // ── Backend management ─────────────────────────────────────────

    /// Register an available backend entry.
    pub async fn register(&self, entry: BackendEntry) {
        self.registered.write().await.push(entry);
    }

    /// All registered backends (metadata snapshot; controllers shared via Arc).
    pub async fn list_backends(&self) -> Vec<BackendEntry> {
        self.registered.read().await.clone()
    }

    /// Id of the currently active backend entry.
    pub async fn active_id(&self) -> Option<String> {
        self.active_id.read().await.clone()
    }

    /// Make `id` the active backend: disconnects the previous active
    /// controller and points the runtime at the new one.
    pub async fn activate(&self, id: &str) -> Result<(), ControllerError> {
        let entry = {
            let entries = self.registered.read().await;
            entries.iter().find(|e| e.id == id).cloned()
        };
        let entry = entry.ok_or_else(|| ControllerError::NotFound(id.to_string()))?;

        // Disconnect the previous active controller (if any).
        {
            let mut active = self.active.write().await;
            if let Some(prev) = active.take() {
                let _ = prev.write().await.disconnect().await;
            }
        }

        // Connect the new controller OUTSIDE the active write lock.
        let connected = if let Some(ctrl) = &entry.controller {
            let mut guard = ctrl.write().await;
            if !guard.is_connected() {
                if let Err(e) = guard.connect().await {
                    *self.active_id.write().await = None;
                    return Err(e);
                }
            }
            Some(ctrl.clone())
        } else {
            None
        };

        {
            let mut active = self.active.write().await;
            *active = connected;
        }
        *self.active_id.write().await = Some(id.to_string());
        Ok(())
    }

    /// Disconnect a connected backend. `not_connected` when the backend
    /// has no connected controller.
    pub async fn disconnect_backend(&self, id: &str) -> Result<(), ControllerError> {
        let mut entries = self.registered.write().await;
        let entry = entries
            .iter_mut()
            .find(|e| e.id == id)
            .ok_or_else(|| ControllerError::NotFound(id.to_string()))?;
        let ctrl = entry
            .controller
            .take()
            .ok_or(ControllerError::NotConnected)?;
        let mut guard = ctrl.write().await;
        if guard.is_connected() {
            guard.disconnect().await?;
        }
        if self.active_id.read().await.as_deref() == Some(id) {
            *self.active.write().await = None;
            *self.active_id.write().await = None;
        }
        Ok(())
    }

    // ── Legacy lifecycle ──────────────────────────────────────

    /// Register a controller as the active one (sets it connected).
    pub async fn set_active(
        &self,
        controller: Arc<RwLock<dyn RobotController + Send + Sync>>,
    ) -> Result<(), ControllerError> {
        let mut active = self.active.write().await;
        if active.is_some() {
            return Err(ControllerError::AlreadyConnected);
        }
        controller.write().await.connect().await?;
        *active = Some(controller);
        *self.active_id.write().await = Some("simulation".to_string());
        Ok(())
    }

    /// Disconnect and remove the active controller.
    pub async fn disconnect(&self) -> Result<(), ControllerError> {
        let mut active = self.active.write().await;
        if let Some(ctrl) = active.take() {
            ctrl.write().await.disconnect().await?;
            *self.active_id.write().await = None;
        }
        Ok(())
    }

    /// Replace the active controller with a new one.
    pub async fn replace_controller(
        &self,
        controller: Arc<RwLock<dyn RobotController + Send + Sync>>,
    ) -> Result<(), ControllerError> {
        let mut active = self.active.write().await;
        if let Some(prev) = active.take() {
            let mut guard = prev.write().await;
            let _ = guard.disconnect().await;
        }
        controller.write().await.connect().await?;
        *active = Some(controller.clone());
        *self.active_id.write().await = Some("simulation".to_string());
        if let Some(entry) = self
            .registered
            .write()
            .await
            .iter_mut()
            .find(|e| e.id == "simulation")
        {
            entry.controller = Some(controller);
        }
        Ok(())
    }

    /// Is any controller connected?
    pub async fn is_connected(&self) -> bool {
        self.active.read().await.is_some()
    }

    /// Get the active controller for use.
    pub async fn get_controller(&self) -> Option<Arc<RwLock<dyn RobotController + Send + Sync>>> {
        self.active.read().await.clone()
    }

    /// Execution source of the ACTIVE controller.
    pub async fn active_source(&self) -> ExecutionSource {
        match self.get_controller().await {
            Some(ctrl) => ctrl.read().await.execution_source(),
            None => ExecutionSource::Simulation,
        }
    }
}

impl Default for BackendManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backends::controller::tests::MockController;

    async fn make_controller() -> Arc<RwLock<dyn RobotController + Send + Sync>> {
        let ctrl = MockController::new();
        Arc::new(RwLock::new(ctrl))
    }

    #[tokio::test]
    async fn list_backends_returns_registered_entries() {
        let manager = BackendManager::new();
        let ctrl = make_controller().await;
        manager
            .register(BackendEntry {
                id: "simulation".into(),
                name: "Simulation".into(),
                controller: Some(ctrl),
                port: None,
            })
            .await;

        let backends = manager.list_backends().await;
        assert_eq!(backends.len(), 1);
        assert_eq!(backends[0].id, "simulation");
    }

    #[tokio::test]
    async fn activate_switches_active_backend() {
        let manager = BackendManager::new();
        let ctrl = make_controller().await;
        manager
            .register(BackendEntry {
                id: "simulation".into(),
                name: "Simulation".into(),
                controller: Some(ctrl.clone()),
                port: None,
            })
            .await;
        manager.activate("simulation").await.unwrap();

        assert_eq!(manager.active_id().await.as_deref(), Some("simulation"));
        assert!(manager.get_controller().await.is_some());
    }

    #[tokio::test]
    async fn activate_unknown_backend_returns_not_found() {
        let manager = BackendManager::new();
        let err = manager.activate("unknown").await.unwrap_err();
        assert_eq!(err, ControllerError::NotFound("unknown".into()));
    }

    #[tokio::test]
    async fn test_set_active_connects() {
        let manager = BackendManager::new();
        let ctrl = make_controller().await;

        manager.set_active(ctrl.clone()).await.unwrap();
        assert!(manager.is_connected().await);
    }

    #[tokio::test]
    async fn test_double_set_active_rejected() {
        let manager = BackendManager::new();
        let ctrl1 = make_controller().await;
        let ctrl2 = make_controller().await;

        manager.set_active(ctrl1).await.unwrap();
        let err = manager.set_active(ctrl2).await.unwrap_err();
        assert_eq!(err, ControllerError::AlreadyConnected);
    }

    #[tokio::test]
    async fn test_disconnect_cleans() {
        let manager = BackendManager::new();
        let ctrl = make_controller().await;

        manager.set_active(ctrl).await.unwrap();
        assert!(manager.is_connected().await);

        manager.disconnect().await.unwrap();
        assert!(!manager.is_connected().await);
    }

    #[tokio::test]
    async fn test_replace_controller_switches() {
        let manager = BackendManager::new();
        let ctrl1 = make_controller().await;
        let ctrl2 = make_controller().await;

        manager.set_active(ctrl1).await.unwrap();
        assert!(manager.is_connected().await);

        manager.replace_controller(ctrl2).await.unwrap();
        assert!(manager.is_connected().await);
    }
}
