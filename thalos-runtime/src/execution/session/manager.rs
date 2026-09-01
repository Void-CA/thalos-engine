use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use chrono::Utc;
use tokio::sync::RwLock;

use crate::motion_trace::MotionTrace;
use crate::plan::session_status::SessionStatus;
use crate::plan::ExecutionMode;
use crate::telemetry::ExecutionTrace;

use super::execution_source::ExecutionSource;
use super::session_data::{SessionData, SessionWithTrace};

const THALOS_DIR: &str = ".thls/sessions";

/// Gestiona el ciclo de vida completo de las ejecuciones.
///
/// En memoria para acceso rápido, con persistencia a archivos
/// en `~/.thls/sessions/{id}/` para supervivencia entre reinicios.
pub struct SessionManager {
    sessions: RwLock<Vec<SessionData>>,
    traces: RwLock<Vec<(u64, MotionTrace)>>,
    base_path: PathBuf,
    next_id: AtomicU64,
}

impl SessionManager {
    /// Crear un SessionManager con persistencia en el directorio por defecto.
    pub fn new() -> Self {
        let base = dirs_next()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(THALOS_DIR);
        Self::with_path(base)
    }

    /// Crear un SessionManager con una ruta base específica (útil para tests).
    pub fn with_path(base: PathBuf) -> Self {
        let mut manager = Self {
            sessions: RwLock::new(Vec::new()),
            traces: RwLock::new(Vec::new()),
            base_path: base.clone(),
            next_id: AtomicU64::new(1),
        };

        // Cargar sesiones existentes del disco
        if let Ok(existing) = Self::load_all_from_disk(&base) {
            let max_id = existing.iter().map(|s| s.id).max().unwrap_or(0);
            manager.next_id = AtomicU64::new(max_id + 1);
            if let Ok(mut sessions) = manager.sessions.try_write() {
                *sessions = existing;
            }
        }

        manager
    }

    /// Registrar una nueva sesión (empieza en estado Running).
    ///
    /// SM4: accepts the `ExecutionMode` and initializes `iteration = 1` and
    /// `total_iterations = mode.total_iterations()`.
    pub async fn register(
        &self,
        source: ExecutionSource,
        plan_id: String,
        duration: f64,
        joint_count: usize,
        robot_name: String,
        mode: ExecutionMode,
    ) -> SessionData {
        let id = self.next_id.fetch_add(1, Ordering::SeqCst);
        let session = SessionData {
            id,
            plan_id,
            source,
            status: SessionStatus::Running,
            started_at: Some(Utc::now()),
            paused_at: None,
            completed_at: None,
            duration,
            joint_count,
            robot_name,
            mode,
            iteration: 1,
            total_iterations: mode.total_iterations(),
        };

        self.sessions.write().await.push(session.clone());
        let _ = self.save_to_disk(&session).await;
        session
    }

    /// Actualizar el contador de iteración de una sesión (R5, R3).
    ///
    /// The scene service persists the current iteration on every iteration
    /// transition (intermediate completions AND the terminal one) so the
    /// session list / DTOs always carry the live value.
    pub async fn set_iteration(&self, id: u64, iteration: u32) -> Option<SessionData> {
        let mut sessions = self.sessions.write().await;
        if let Some(session) = sessions.iter_mut().find(|s| s.id == id) {
            session.iteration = iteration;
            let s = session.clone();
            let _ = self.save_to_disk(&s).await;
            return Some(s);
        }
        None
    }

    /// Marcar sesión como completada y guardar el trace.
    pub async fn complete(&self, id: u64, trace: MotionTrace) -> Option<SessionData> {
        self.complete_with_status(id, trace, SessionStatus::Completed)
            .await
    }

    /// Marcar sesión con un estado terminal específico y guardar el trace.
    pub async fn complete_with_status(
        &self,
        id: u64,
        trace: MotionTrace,
        status: SessionStatus,
    ) -> Option<SessionData> {
        debug_assert!(
            status.is_terminal(),
            "complete_with_status requires terminal status"
        );
        let mut sessions = self.sessions.write().await;
        if let Some(session) = sessions.iter_mut().find(|s| s.id == id) {
            session.status = status;
            session.completed_at = Some(Utc::now());
            self.traces.write().await.push((id, trace));
            let s = session.clone();
            let _ = self.save_to_disk(&s).await;
            let _ = self.save_trace_to_disk(id, &s).await;
            return Some(s);
        }
        None
    }

    /// Actualizar el estado de una sesión.
    pub async fn set_status(&self, id: u64, status: SessionStatus) -> Option<SessionData> {
        let mut sessions = self.sessions.write().await;
        if let Some(session) = sessions.iter_mut().find(|s| s.id == id) {
            session.status = status;
            if status.is_terminal() {
                session.completed_at = Some(Utc::now());
            }
            let s = session.clone();
            let _ = self.save_to_disk(&s).await;
            return Some(s);
        }
        None
    }

    /// Obtener datos de una sesión.
    pub async fn get(&self, id: u64) -> Option<SessionData> {
        let sessions = self.sessions.read().await;
        sessions.iter().find(|s| s.id == id).cloned()
    }

    /// Obtener el trace de una sesión.
    pub async fn get_trace(&self, id: u64) -> Option<MotionTrace> {
        let traces = self.traces.read().await;
        traces
            .iter()
            .find(|(sid, _)| *sid == id)
            .map(|(_, t)| t.clone())
    }

    /// Obtener sesión + trace.
    pub async fn get_with_trace(&self, id: u64) -> Option<SessionWithTrace> {
        let session = self.get(id).await?;
        let trace = self.get_trace(id).await;
        Some(SessionWithTrace { session, trace })
    }

    /// Listar todas las sesiones.
    pub async fn list(&self) -> Vec<SessionData> {
        let sessions = self.sessions.read().await;
        let mut result = sessions.clone();
        result.sort_by(|a, b| b.id.cmp(&a.id)); // más reciente primero
        result
    }

    /// Guardar un `ExecutionTrace` asociado a una sesión.
    pub async fn save_execution_trace(&self, id: u64, trace: ExecutionTrace) {
        let dir = self.base_path.join(format!("{:06}", id));
        let path = dir.join("execution_trace.json");
        if let Ok(json) = serde_json::to_string_pretty(&trace) {
            let _ = tokio::fs::write(path, json).await;
        }
    }

    /// Obtener el `ExecutionTrace` de una sesión desde disco.
    pub async fn get_execution_trace(&self, id: u64) -> Option<ExecutionTrace> {
        let path = self
            .base_path
            .join(format!("{:06}", id))
            .join("execution_trace.json");
        let content = tokio::fs::read_to_string(path).await.ok()?;
        serde_json::from_str(&content).ok()
    }

    /// Importar un trace como nueva sesión.
    pub async fn import(
        &self,
        source: ExecutionSource,
        trace: MotionTrace,
        robot_name: String,
    ) -> SessionData {
        let duration = trace.duration().as_secs_f64();
        let joint_count = trace.samples().first().map(|s| s.joints.len()).unwrap_or(0);
        let session = self
            .register(source, "imported".into(), duration, joint_count, robot_name, ExecutionMode::Once)
            .await;
        self.complete(session.id, trace).await;
        session
    }

    // ── Persistencia a disco ──

    async fn save_to_disk(&self, session: &SessionData) -> Result<(), std::io::Error> {
        let dir = self.base_path.join(format!("{:06}", session.id));
        tokio::fs::create_dir_all(&dir).await?;
        let path = dir.join("session.json");
        let json = serde_json::to_string_pretty(session)?;
        tokio::fs::write(path, json).await
    }

    async fn save_trace_to_disk(
        &self,
        id: u64,
        _session: &SessionData,
    ) -> Result<(), std::io::Error> {
        let traces = self.traces.read().await;
        if let Some((_, trace)) = traces.iter().find(|(sid, _)| *sid == id) {
            let dir = self.base_path.join(format!("{:06}", id));
            let path = dir.join("trace.json");
            let json = serde_json::to_string_pretty(trace)?;
            return tokio::fs::write(path, json).await;
        }
        Ok(())
    }

    fn load_all_from_disk(base: &PathBuf) -> Result<Vec<SessionData>, std::io::Error> {
        let mut sessions = Vec::new();
        if !base.exists() {
            return Ok(sessions);
        }
        for entry in std::fs::read_dir(base)? {
            let entry = entry?;
            let dir_path = entry.path();
            if !dir_path.is_dir() {
                continue;
            }
            let session_path = dir_path.join("session.json");
            if !session_path.exists() {
                continue;
            }
            if let Ok(json) = std::fs::read_to_string(&session_path) {
                if let Ok(session) = serde_json::from_str::<SessionData>(&json) {
                    sessions.push(session);
                }
            }
        }
        Ok(sessions)
    }

    /// Cargar un trace desde disco (para importación).
    pub async fn load_trace_from_file(path: &str) -> Result<MotionTrace, String> {
        let content = tokio::fs::read_to_string(path)
            .await
            .map_err(|e| e.to_string())?;
        serde_json::from_str(&content).map_err(|e| e.to_string())
    }
}

impl Default for SessionManager {
    fn default() -> Self {
        Self::new()
    }
}

fn dirs_next() -> Option<PathBuf> {
    // Try XDG_DATA_HOME first, then $HOME
    if let Ok(val) = std::env::var("XDG_DATA_HOME") {
        Some(PathBuf::from(val))
    } else if let Ok(home) = std::env::var("HOME") {
        Some(PathBuf::from(home))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion_trace::{MotionSample, MotionTrace};

    fn sample_trace() -> MotionTrace {
        let mut trace = MotionTrace::new();
        trace.push(MotionSample {
            timestamp: std::time::Duration::from_secs_f64(0.0),
            joints: vec![0.0, 0.0],
            velocities: vec![0.0, 0.0],
            target_joints: None,
            progress: 0.0,
            errors: vec![],
        });
        trace.push(MotionSample {
            timestamp: std::time::Duration::from_secs_f64(1.0),
            joints: vec![1.0, 0.5],
            velocities: vec![1.0, 0.5],
            target_joints: None,
            progress: 1.0,
            errors: vec![],
        });
        trace
    }

    fn tmp_path() -> PathBuf {
        let dir = std::env::temp_dir().join(format!("thalos-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    #[tokio::test]
    async fn register_and_get() {
        let manager = SessionManager::with_path(tmp_path());
        let session = manager
            .register(
                ExecutionSource::Simulation,
                "plan-1".into(),
                2.0,
                6,
                "test_robot".into(),
                ExecutionMode::Once,
            )
            .await;
        assert_eq!(session.id, 1);
        assert_eq!(session.status, SessionStatus::Running);

        let retrieved = manager.get(1).await;
        assert!(retrieved.is_some());
        assert_eq!(retrieved.unwrap().plan_id, "plan-1");
    }

    #[tokio::test]
    async fn complete_with_trace() {
        let manager = SessionManager::with_path(tmp_path());
        let session = manager
            .register(
                ExecutionSource::Hardware,
                "plan-2".into(),
                1.0,
                2,
                "robot".into(),
                ExecutionMode::Once,
            )
            .await;
        let trace = sample_trace();

        let completed = manager.complete(session.id, trace.clone()).await;
        assert!(completed.is_some());
        assert_eq!(completed.unwrap().status, SessionStatus::Completed);

        let stored_trace = manager.get_trace(session.id).await;
        assert!(stored_trace.is_some());
        assert_eq!(stored_trace.unwrap().len(), 2);
    }

    #[tokio::test]
    async fn list_returns_most_recent_first() {
        let manager = SessionManager::with_path(tmp_path());
        manager
            .register(ExecutionSource::Simulation, "p1".into(), 1.0, 2, "r".into(), ExecutionMode::Once)
            .await;
        manager
            .register(ExecutionSource::Simulation, "p2".into(), 1.0, 2, "r".into(), ExecutionMode::Once)
            .await;

        let list = manager.list().await;
        assert_eq!(list.len(), 2);
        assert_eq!(list[0].id, 2);
        assert_eq!(list[1].id, 1);
    }

    #[test]
    fn source_display() {
        assert_eq!(ExecutionSource::Simulation.to_string(), "Simulation");
        assert_eq!(ExecutionSource::Hardware.to_string(), "Hardware");
    }
}
