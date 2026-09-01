use std::sync::Arc;

use tokio::sync::RwLock;

use super::controller::RobotController;
use super::esp32::Esp32Backend;
use super::transport::{SerialTransport, Transport};
use crate::error::ControllerError;
use crate::session::execution_source::ExecutionSource;

/// ESP32 serial baud (protocol v2, C): the firmware is flashed with
/// `Serial.begin(460800)` — the throughput lever that takes a 92KB upload from
/// ~8s to ~2s. Overridable via `THALOS_ESP32_BAUD` for diagnosis; the override
/// changes only the SPEED, never the protocol version (v1/v2 compatibility
/// still requires the matching firmware flash).
pub(crate) fn esp32_baud() -> u32 {
    std::env::var("THALOS_ESP32_BAUD")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(460_800)
}

/// An available execution backend (resilience-presentation PR2a).
///
/// `controller` is `None` until the hardware backend connects for the FIRST
/// time — the lazy Esp32 factory keeps the serial port closed at boot and
/// only opens it on `connect_with_port`.
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
    /// Id of the active backend entry ("simulation" | "esp32").
    active_id: RwLock<Option<String>>,
    /// All registered backends; Simulation is always present, Esp32 is
    /// registered conditionally from the environment.
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

    // ── Backend management (PR2a) ─────────────────────────────────────────

    /// Register an available backend entry. Never opens a serial port.
    pub async fn register(&self, entry: BackendEntry) {
        self.registered.write().await.push(entry);
    }

    /// Register the lazy Esp32 hardware backend from an env-provided port.
    /// The controller is created on the first `connect_with_port` call — no
    /// serial port is opened here.
    pub async fn register_esp32(&self, port: &str) {
        self.register(BackendEntry {
            id: "esp32".into(),
            name: "Hardware (ESP32)".into(),
            controller: None,
            port: Some(port.to_string()),
        })
        .await;
    }

    /// All registered backends (metadata snapshot; controllers shared via Arc).
    pub async fn list_backends(&self) -> Vec<BackendEntry> {
        self.registered.read().await.clone()
    }

    /// Id of the currently active backend entry.
    pub async fn active_id(&self) -> Option<String> {
        self.active_id.read().await.clone()
    }

    /// Make `id` the active backend (PR2a): disconnects the previous active
    /// controller (closing its serial port) and points the runtime at the new
    /// one. A hardware entry that is not yet connected leaves the runtime with
    /// no controller until `connect_with_port` succeeds — execution then
    /// reports the backend's connection state.
    ///
    /// R4-002: the controller `connect` (a serial handshake that can take up
    /// to the read timeout) runs WITHOUT the `active` write lock — a slow or
    /// hung handshake must not block `get_controller()` consumers (the tick
    /// loop, scene ops). On failure the previous active controller is already
    /// disconnected and `active`/`active_id` stay consistently empty (clean
    /// rollback, nothing wedged).
    pub async fn activate(&self, id: &str) -> Result<(), ControllerError> {
        let entry = {
            let entries = self.registered.read().await;
            entries.iter().find(|e| e.id == id).cloned()
        };
        let entry = entry.ok_or_else(|| ControllerError::NotFound(id.to_string()))?;

        // Disconnect the previous active controller (if any) — closes its port.
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
                    // Clean rollback: the runtime is left without a controller
                    // and the id reflects it (consistent empty state).
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

    /// Connect the hardware backend `id` to `port` (lazy Esp32 factory, PR2a).
    ///
    /// - unknown id → `NotFound`
    /// - serial port cannot be opened (missing/occupied device) → `PortInUse`
    /// - port opens but no firmware answers the HELLO handshake → `NoFirmware`
    ///
    /// On success the connected controller is stored in the backend entry and
    /// becomes the runtime controller when that backend is active.
    pub async fn connect_with_port(&self, id: &str, port: &str) -> Result<(), ControllerError> {
        tracing::info!(backend = %id, %port, "connect_with_port — opening serial transport");
        let transport = SerialTransport::new(port, esp32_baud());
        self.connect_with_transport(id, port, Box::new(transport))
            .await
    }

    /// Connect with an injected transport — the shared implementation behind
    /// `connect_with_port`. Test-support by contract: production code always
    /// builds a `SerialTransport`; tests inject a fake to exercise the
    /// firmware-handshake paths without a real serial device.
    pub async fn connect_with_transport(
        &self,
        id: &str,
        port: &str,
        mut transport: Box<dyn Transport>,
    ) -> Result<(), ControllerError> {
        {
            let entries = self.registered.read().await;
            if !entries.iter().any(|e| e.id == id) {
                return Err(ControllerError::NotFound(id.to_string()));
            }
        }
        // Port-level failure (missing/occupied device) → port_in_use.
        transport.connect().await.map_err(|e| {
            tracing::error!(backend = %id, %port, error = %e, "connect — serial open failed");
            ControllerError::PortInUse(e.to_string())
        })?;

        // Port opened but no firmware answers the HELLO handshake → no_firmware.
        // R4-002: the handshake read is bounded by the transport timeout, so
        // this returns FAST on a silent device, and the explicit `drop` closes
        // the serial device — a retry does NOT hit port_in_use.
        let mut backend = Esp32Backend::new(transport);
        if let Err(e) = backend.connect().await {
            tracing::error!(backend = %id, %port, error = %e, "connect — handshake failed (no firmware)");
            drop(backend);
            return Err(ControllerError::NoFirmware);
        }
        tracing::info!(backend = %id, %port, "connect — handshake OK, controller stored");

        let ctrl = Arc::new(RwLock::new(backend)) as Arc<RwLock<dyn RobotController + Send + Sync>>;
        {
            let mut entries = self.registered.write().await;
            if let Some(entry) = entries.iter_mut().find(|e| e.id == id) {
                entry.controller = Some(ctrl.clone());
                entry.port = Some(port.to_string());
            }
        }
        // If this backend is the active one, point the runtime at the new
        // controller immediately.
        if self.active_id.read().await.as_deref() == Some(id) {
            *self.active.write().await = Some(ctrl);
        }
        Ok(())
    }

    /// Disconnect a connected backend (PR2a). `not_connected` when the backend
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
            // R3-001: keep active_id consistent with active — the runtime now
            // has NO controller, so no backend can be reported active.
            *self.active_id.write().await = None;
        }
        Ok(())
    }

    // ── Legacy lifecycle (unchanged) ──────────────────────────────────────

    /// Register a controller as the active one (sets it connected).
    ///
    /// Legacy lifecycle path: the controller it installs is the simulation
    /// controller, so `active_id` tracks the `simulation` entry (R3-001) —
    /// keeping `active_id` consistent with the controller the runtime uses.
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
    ///
    /// Disconnects and removes the previous controller, then connects
    /// and sets the new one. Useful when the robot changes (e.g., new DOF).
    ///
    /// R3-001: keeps `active_id` consistent with `active` — the replacement
    /// controller is the simulation controller (the registered `simulation`
    /// entry is synced below), so `active_id` points at `simulation` instead
    /// of diverging (e.g. staying `esp32` after a robot change).
    pub async fn replace_controller(
        &self,
        controller: Arc<RwLock<dyn RobotController + Send + Sync>>,
    ) -> Result<(), ControllerError> {
        let mut active = self.active.write().await;
        // Disconnect previous if any
        if let Some(prev) = active.take() {
            let mut guard = prev.write().await;
            let _ = guard.disconnect().await;
        }
        // Connect and set the new one
        controller.write().await.connect().await?;
        *active = Some(controller.clone());
        *self.active_id.write().await = Some("simulation".to_string());
        // Keep the registered Simulation entry in sync so `GET /backends`
        // reflects the controller the runtime actually uses (PR2a).
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
    /// Returns `None` if no controller is connected.
    pub async fn get_controller(&self) -> Option<Arc<RwLock<dyn RobotController + Send + Sync>>> {
        self.active.read().await.clone()
    }

    /// Execution source of the ACTIVE controller (R4-001) — reflects the real
    /// backend (Simulation vs Hardware) on the wire instead of a hardcoded
    /// value. Falls back to Simulation when no controller is connected.
    pub async fn active_source(&self) -> ExecutionSource {
        match self.get_controller().await {
            Some(ctrl) => ctrl.read().await.execution_source(),
            None => ExecutionSource::Simulation,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backends::controller::tests::MockController;
    use crate::backends::transport::{FakeTransport, TransportError};
    use crate::error::ControllerError;
    use async_trait::async_trait;
    use std::time::Duration;
    use tokio_serial::SerialPort;

    /// Test seam: a transport whose `connect` always fails with an IO error —
    /// used to prove that an OPEN failure maps to `ControllerError::PortInUse`
    /// deterministically (no reliance on real /dev paths).
    struct FailingTransport;

    #[async_trait]
    impl Transport for FailingTransport {
        async fn connect(&mut self) -> Result<(), TransportError> {
            Err(TransportError::Io(std::io::Error::new(
                std::io::ErrorKind::PermissionDenied,
                "simulated open failure",
            )))
        }
        async fn disconnect(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        async fn send(&mut self, _data: &[u8]) -> Result<(), TransportError> {
            Ok(())
        }
        async fn receive(&mut self) -> Result<Vec<u8>, TransportError> {
            Err(TransportError::Timeout)
        }
    }

    /// Wait on the PTY master until the host's HELLO line arrives, then answer
    /// the handshake. Ignores leftover bytes (e.g. the STOP written on the
    /// previous disconnect) and retries on EIO while the slave is not yet open.
    async fn answer_handshake(master: &mut tokio_serial::SerialStream) {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        let mut buf = [0u8; 512];
        let mut pending: Vec<u8> = Vec::new();
        loop {
            match master.read(&mut buf).await {
                Ok(n) => {
                    pending.extend_from_slice(&buf[..n]);
                    if let Some(pos) = pending.iter().position(|&b| b == b'\n') {
                        let line: Vec<u8> = pending.drain(..=pos).collect();
                        if line.windows(5).any(|w| w == b"HELLO") {
                            master.write_all(b"HELLO 2 OK\r\n").await.unwrap();
                            master.flush().await.unwrap();
                            return;
                        }
                    }
                }
                Err(_) => {
                    // EIO while the slave device is closed (disconnect) or not
                    // yet opened — retry shortly until the next connect opens it.
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            }
        }
    }

    async fn make_controller() -> Arc<RwLock<dyn RobotController + Send + Sync>> {
        let ctrl = MockController::new();
        Arc::new(RwLock::new(ctrl))
    }

    // ── Backend management (resilience-presentation PR2a) ────────────────

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
        manager.register_esp32("/dev/ttyUSB0").await;

        let backends = manager.list_backends().await;
        assert_eq!(backends.len(), 2);
        assert_eq!(backends[0].id, "simulation");
        assert_eq!(backends[1].id, "esp32");
        assert_eq!(backends[1].port.as_deref(), Some("/dev/ttyUSB0"));
        assert!(
            backends[1].controller.is_none(),
            "esp32 must NOT open a port at boot (lazy factory)"
        );
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
    async fn activate_esp32_without_connect_leaves_runtime_without_controller() {
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
        manager.activate("simulation").await.unwrap();
        manager.register_esp32("/dev/ttyUSB0").await;
        manager.activate("esp32").await.unwrap();

        assert_eq!(manager.active_id().await.as_deref(), Some("esp32"));
        assert!(
            manager.get_controller().await.is_none(),
            "hardware active-but-not-connected has no controller"
        );
    }

    #[tokio::test]
    async fn disconnect_esp32_without_controller_returns_not_connected() {
        let manager = BackendManager::new();
        manager.register_esp32("/dev/ttyUSB0").await;
        let err = manager.disconnect_backend("esp32").await.unwrap_err();
        assert_eq!(err, ControllerError::NotConnected);
    }

    #[tokio::test]
    async fn connect_with_port_unknown_backend_returns_not_found() {
        let manager = BackendManager::new();
        let err = manager
            .connect_with_port("nope", "/dev/ttyUSB0")
            .await
            .unwrap_err();
        assert_eq!(err, ControllerError::NotFound("nope".into()));
    }

    #[tokio::test]
    async fn connect_with_port_to_invalid_device_returns_port_in_use() {
        let manager = BackendManager::new();
        manager
            .register_esp32("/dev/thalos-tests-nonexistent-7f3c")
            .await;
        let err = manager
            .connect_with_port("esp32", "/dev/thalos-tests-nonexistent-7f3c")
            .await
            .unwrap_err();
        assert!(
            matches!(err, ControllerError::PortInUse(_)),
            "open failure must map to port_in_use, got {err:?}"
        );
    }

    #[tokio::test]
    async fn connect_with_transport_no_firmware_response_returns_no_firmware() {
        let manager = BackendManager::new();
        manager.register_esp32("/dev/ttyUSB0").await;
        // FakeTransport with NO injected HELLO response → handshake times out.
        let transport = Box::new(FakeTransport::new());
        let err = manager
            .connect_with_transport("esp32", "/dev/ttyUSB0", transport)
            .await
            .unwrap_err();
        assert_eq!(err, ControllerError::NoFirmware);

        let entry = manager
            .list_backends()
            .await
            .into_iter()
            .find(|e| e.id == "esp32")
            .unwrap();
        assert!(
            entry.controller.is_none(),
            "failed connect must not leave a controller"
        );
    }

    /// R4-002: a REAL `SerialTransport` over a silent serial device must fail
    /// the handshake FAST (`NoFirmware`, no infinite hang) AND release the
    /// device — a retry on the same path must work (no `port_in_use` wedge).
    #[tokio::test]
    async fn connect_to_silent_serial_device_returns_no_firmware_without_wedging() {
        let manager = BackendManager::new();
        manager.register_esp32("/dev/thalos-silent-ptty").await;
        // Real virtual serial device: hold the master open (never written to →
        // the slave read blocks) and use the slave's PTY path as the port.
        let (mut master, slave) = tokio_serial::SerialStream::pair().unwrap();
        let port = slave.name().expect("PTY slave must expose its device path");

        // First connect: silent device → handshake read timeout → NoFirmware.
        let t1 = SerialTransport::new(&port, 115200).with_read_timeout(Duration::from_millis(150));
        let start = std::time::Instant::now();
        let err = manager
            .connect_with_transport("esp32", &port, Box::new(t1))
            .await
            .unwrap_err();
        assert_eq!(err, ControllerError::NoFirmware);
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "handshake against a silent device must not hang"
        );

        // Drain the HELLO bytes the failed connect wrote to the master so the
        // retry genuinely re-exercises the silent-device read timeout.
        let mut drain = [0u8; 128];
        loop {
            match master.try_read(&mut drain) {
                Ok(0) | Err(_) => break,
                Ok(_) => continue,
            }
        }

        // Retry on the SAME path: the failed connect must have closed the
        // device (no port_in_use wedge) — the retry also fails fast.
        let t2 = SerialTransport::new(&port, 115200).with_read_timeout(Duration::from_millis(150));
        let start = std::time::Instant::now();
        let err = manager
            .connect_with_transport("esp32", &port, Box::new(t2))
            .await
            .unwrap_err();
        assert_eq!(
            err,
            ControllerError::NoFirmware,
            "retry must NOT fail with port_in_use (device released after failure)"
        );
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "retry handshake must not hang"
        );
    }

    #[tokio::test]
    async fn connect_with_transport_success_stores_controller_and_port() {
        let manager = BackendManager::new();
        manager.register_esp32("/dev/ttyUSB0").await;
        let transport = FakeTransport::new();
        transport.inject_response(b"HELLO 2 OK\n".to_vec());
        manager
            .connect_with_transport("esp32", "/dev/ttyUSB0", Box::new(transport))
            .await
            .unwrap();

        let entry = manager
            .list_backends()
            .await
            .into_iter()
            .find(|e| e.id == "esp32")
            .unwrap();
        assert!(
            entry.controller.is_some(),
            "controller stored after connect"
        );
        assert_eq!(entry.port.as_deref(), Some("/dev/ttyUSB0"));
    }

    #[tokio::test]
    async fn connect_active_backend_becomes_runtime_controller() {
        let manager = BackendManager::new();
        manager.register_esp32("/dev/ttyUSB0").await;
        manager.activate("esp32").await.unwrap();
        assert!(manager.get_controller().await.is_none());

        let transport = FakeTransport::new();
        transport.inject_response(b"HELLO 2 OK\n".to_vec());
        manager
            .connect_with_transport("esp32", "/dev/ttyUSB0", Box::new(transport))
            .await
            .unwrap();

        assert!(
            manager.get_controller().await.is_some(),
            "connecting the active backend must point the runtime at it"
        );
    }

    /// Port-level failure: a transport whose OPEN fails with an IO error must
    /// map to `ControllerError::PortInUse` — the deterministic, injectable
    /// version of the real-device test (`connect_with_port_to_invalid_device_*`).
    #[tokio::test]
    async fn connect_with_transport_open_failure_maps_to_port_in_use() {
        let manager = BackendManager::new();
        manager.register_esp32("/dev/ttyUSB0").await;

        let err = manager
            .connect_with_transport("esp32", "/dev/ttyUSB0", Box::new(FailingTransport))
            .await
            .unwrap_err();
        assert!(
            matches!(err, ControllerError::PortInUse(_)),
            "open failure must map to port_in_use, got {err:?}"
        );

        // Failed open must not leave a controller or point the runtime at it.
        let entry = manager
            .list_backends()
            .await
            .into_iter()
            .find(|e| e.id == "esp32")
            .unwrap();
        assert!(entry.controller.is_none());
        assert!(manager.get_controller().await.is_none());
    }

    /// Reconnect on the SAME device path: connect → disconnect → connect again
    /// over a REAL virtual serial device must succeed both times, with no
    /// wedged port, no stale controller, and a clean active state. Each connect
    /// answers the HELLO handshake through the PTY master.
    #[tokio::test]
    async fn reconnect_same_port_after_disconnect_works() {
        let manager = BackendManager::new();
        manager.register_esp32("/dev/thalos-reconnect-ptty").await;
        manager.activate("esp32").await.unwrap();
        assert!(manager.get_controller().await.is_none());

        let (mut master, slave) = tokio_serial::SerialStream::pair().unwrap();
        let path = slave.name().expect("PTY slave must expose its device path");
        // Close the test's copy of the slave — the backend opens the device
        // path itself on each connect (true open/close cycle).
        drop(slave);

        let mut connect = async |manager: &BackendManager, path: &str| {
            let t =
                SerialTransport::new(path, 115200).with_read_timeout(Duration::from_millis(300));
            let c = tokio::time::timeout(
                Duration::from_secs(3),
                manager.connect_with_transport("esp32", path, Box::new(t)),
            );
            let a = tokio::time::timeout(Duration::from_secs(3), answer_handshake(&mut master));
            let (cr, ar) = tokio::join!(c, a);
            assert!(
                ar.is_ok(),
                "PTY master must answer the handshake within the test bound"
            );
            cr.expect("connect must complete within the test bound")
                .unwrap();
        };

        // First connect → controller stored and runtime pointed at it.
        connect(&manager, &path).await;
        assert!(manager.get_controller().await.is_some());
        assert_eq!(manager.active_id().await.as_deref(), Some("esp32"));
        {
            let entry = manager
                .list_backends()
                .await
                .into_iter()
                .find(|e| e.id == "esp32")
                .unwrap();
            assert!(entry.controller.is_some());
        } // entry dropped BEFORE disconnect: it clones the controller Arc,
        // and keeping it alive would leak the serial fd (device stays busy).

        // Disconnect → controller removed, runtime and active_id clean.
        manager.disconnect_backend("esp32").await.unwrap();
        assert!(manager.get_controller().await.is_none());
        assert_eq!(
            manager.active_id().await,
            None,
            "disconnect clears active_id"
        );
        {
            let entry = manager
                .list_backends()
                .await
                .into_iter()
                .find(|e| e.id == "esp32")
                .unwrap();
            assert!(
                entry.controller.is_none(),
                "no stale controller after disconnect"
            );
        } // entry dropped before the reconnect, for the same Arc reason.

        // Second connect on the SAME path → succeeds (device was released, no
        // port_in_use wedge), fresh controller stored.
        connect(&manager, &path).await;
        {
            let entry = manager
                .list_backends()
                .await
                .into_iter()
                .find(|e| e.id == "esp32")
                .unwrap();
            assert!(
                entry.controller.is_some(),
                "reconnect must store a fresh controller"
            );
            assert_eq!(entry.port.as_deref(), Some(path.as_str()));
        } // entry dropped before activate (same Arc-leak reason as above).

        // disconnect_backend cleared active_id (R3-001) — re-activating the
        // backend must point the runtime at the fresh controller, proving the
        // reconnect left no contaminated connected/cache state behind.
        manager.activate("esp32").await.unwrap();
        assert!(manager.get_controller().await.is_some());
        assert_eq!(manager.active_id().await.as_deref(), Some("esp32"));
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

    /// R3-001: after a robot change with a hardware backend active-but-not-
    /// connected, `replace_controller` must point `active_id` at the controller
    /// the runtime ACTUALLY uses (simulation) — no active_id/active divergence.
    #[tokio::test]
    async fn replace_controller_after_robot_change_syncs_active_id() {
        let manager = BackendManager::new();
        let sim = make_controller().await;
        manager
            .register(BackendEntry {
                id: "simulation".into(),
                name: "Simulation".into(),
                controller: Some(sim),
                port: None,
            })
            .await;
        manager.register_esp32("/dev/ttyUSB0").await;
        manager.activate("simulation").await.unwrap();
        manager.activate("esp32").await.unwrap();

        assert_eq!(manager.active_id().await.as_deref(), Some("esp32"));
        assert!(
            manager.get_controller().await.is_none(),
            "esp32 active-but-not-connected has no runtime controller"
        );

        // Robot change (SceneService.execute) silently replaces with a fresh
        // SimulationController.
        manager
            .replace_controller(make_controller().await)
            .await
            .unwrap();

        assert_eq!(
            manager.active_id().await.as_deref(),
            Some("simulation"),
            "active_id must follow the controller the runtime actually uses"
        );
        assert!(manager.get_controller().await.is_some());
    }

    /// R3-001: `set_active` must keep `active_id` consistent with `active`.
    #[tokio::test]
    async fn set_active_syncs_active_id() {
        let manager = BackendManager::new();
        let sim = make_controller().await;
        manager
            .register(BackendEntry {
                id: "simulation".into(),
                name: "Simulation".into(),
                controller: Some(sim),
                port: None,
            })
            .await;
        manager.set_active(make_controller().await).await.unwrap();
        assert!(manager.get_controller().await.is_some());
        assert_eq!(
            manager.active_id().await.as_deref(),
            Some("simulation"),
            "set_active must point active_id at the simulation entry"
        );
    }

    /// R3-001: `disconnect` must clear `active_id` alongside `active`.
    #[tokio::test]
    async fn disconnect_clears_active_id() {
        let manager = BackendManager::new();
        let sim = make_controller().await;
        manager
            .register(BackendEntry {
                id: "simulation".into(),
                name: "Simulation".into(),
                controller: Some(sim),
                port: None,
            })
            .await;
        manager.activate("simulation").await.unwrap();
        assert_eq!(manager.active_id().await.as_deref(), Some("simulation"));

        manager.disconnect().await.unwrap();
        assert!(manager.get_controller().await.is_none());
        assert_eq!(
            manager.active_id().await,
            None,
            "disconnect must clear active_id (no active/active_id divergence)"
        );
    }

    #[tokio::test]
    async fn test_get_controller_returns_none_when_empty() {
        let manager = BackendManager::new();
        assert!(manager.get_controller().await.is_none());
    }
}
