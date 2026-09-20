//! Terminal Controller transport adapter.
//!
//! Wraps a byte-level [`Transport`] (TCP) and adapts the Fase 4.0 terminal
//! encoding to the `RobotTransport` port. It performs the `HELLO`/`CAPABILITIES`
//! negotiation once, exposing the **negotiated** descriptor — the endpoint
//! declares its own capability; the host never invents it.

use std::collections::VecDeque;

use thalos_ports::robot::{
    RobotCommand, RobotObservation, RobotTransport, TransportError, TransportState,
};

use crate::common::Transport;
use crate::terminal_controller::codec::{
    self, FrameError, RemoteCapabilities, TerminalFrame,
};

/// Default contract version spoken by this adapter.
pub const CONTRACT_VERSION: &str = "1.0";

/// Adapter over the terminal-controller line encoding.
pub struct TerminalControllerAdapter<T: Transport> {
    inner: T,
    state: TransportState,
    capabilities: Option<RemoteCapabilities>,
    command_seq: u64,
    observation_queue: VecDeque<RobotObservation>,
}

impl<T: Transport> TerminalControllerAdapter<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            state: TransportState::Disconnected,
            capabilities: None,
            command_seq: 0,
            observation_queue: VecDeque::new(),
        }
    }

    /// The negotiated descriptor, if the handshake has completed.
    pub fn capabilities(&self) -> Option<&RemoteCapabilities> {
        self.capabilities.as_ref()
    }

    /// Connect and negotiate `HELLO` → `CAPABILITIES`.
    ///
    /// The handshake is mandatory: without a negotiated descriptor the adapter
    /// stays disconnected and no command may be sent.
    pub async fn handshake(&mut self) -> Result<RemoteCapabilities, TransportError> {
        self.inner
            .connect()
            .await
            .map_err(|e| TransportError::CommunicationFailure(e.to_string()))?;
        self.state = TransportState::Connecting;

        self.inner
            .send(codec::encode_hello(CONTRACT_VERSION).as_bytes())
            .await
            .map_err(|e| TransportError::CommunicationFailure(e.to_string()))?;

        let line = self.inner.receive().await.map_err(map_io)?;
        let text = String::from_utf8_lossy(&line);
        match codec::decode(&text) {
            Ok(TerminalFrame::Capabilities(caps)) => {
                self.capabilities = Some(caps.clone());
                self.state = TransportState::Connected;
                Ok(caps)
            }
            Ok(TerminalFrame::Rejected { reason, .. }) => {
                self.state = TransportState::Faulted;
                Err(TransportError::CommunicationFailure(format!(
                    "handshake rejected: {reason}"
                )))
            }
            Ok(other) => Err(TransportError::CommunicationFailure(format!(
                "unexpected handshake frame: {other:?}"
            ))),
            Err(FrameError(message)) => {
                Err(TransportError::CommunicationFailure(format!("bad capabilities frame: {message}")))
            }
        }
    }

    fn next_command_id(&mut self) -> String {
        self.command_seq += 1;
        format!("cmd-{}", self.command_seq)
    }

    /// Send a `trajectory_batch` command (ordered joint samples with dt).
    ///
    /// This is the ROS 2 provider's dispatch path: Thalos plans, the resource
    /// reproduces. It is deliberately NOT on the narrow `RobotTransport` port
    /// (which carries `MoveJoints`/`Stop` only) — a `Ros2ControlController`
    /// calls it directly.
    pub async fn send_trajectory_batch(
        &mut self,
        samples: &[(u64, Vec<f64>)],
    ) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }
        let payload = format!("samples={}", encode_samples(samples));
        let command_id = self.next_command_id();
        let frame = codec::encode_command(&command_id, "trajectory_batch", &payload);
        self.inner
            .send(frame.as_bytes())
            .await
            .map_err(map_io)
    }
}

/// Encode `[(dt_us, joints)]` as `dt:j0,j1,...|dt:j0,j1,...`.
fn encode_samples(samples: &[(u64, Vec<f64>)]) -> String {
    samples
        .iter()
        .map(|(dt_us, joints)| format!("{dt_us}:{}", join_f64(joints)))
        .collect::<Vec<_>>()
        .join("|")
}

fn map_io(error: crate::common::IoTransportError) -> TransportError {
    match error {
        crate::common::IoTransportError::Timeout => TransportError::Timeout,
        crate::common::IoTransportError::Disconnected => TransportError::Disconnected,
        other => TransportError::CommunicationFailure(other.to_string()),
    }
}

impl<T: Transport> RobotTransport for TerminalControllerAdapter<T> {
    fn state(&self) -> TransportState {
        self.state
    }

    fn send(&mut self, command: RobotCommand) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }

        let (capability, payload) = match command {
            RobotCommand::MoveJoints { positions_rad, .. } => {
                let payload = format!("joints={}", join_f64(&positions_rad));
                ("move_j".to_string(), payload)
            }
            RobotCommand::Stop => ("move_j".to_string(), String::new()),
        };

        let command_id = self.next_command_id();
        let frame = codec::encode_command(&command_id, &capability, &payload);
        let handle = tokio::runtime::Handle::current();
        tokio::task::block_in_place(|| handle.block_on(self.inner.send(frame.as_bytes())))
            .map_err(map_io)
    }

    fn stop(&mut self) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }
        let frame = codec::encode_stop();
        let handle = tokio::runtime::Handle::current();
        tokio::task::block_in_place(|| handle.block_on(self.inner.send(frame.as_bytes())))
            .map_err(map_io)
    }

    fn try_receive_observation(&mut self) -> Result<Option<RobotObservation>, TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }
        if let Some(obs) = self.observation_queue.pop_front() {
            return Ok(Some(obs));
        }

        let handle = tokio::runtime::Handle::current();
        let received = tokio::task::block_in_place(|| handle.block_on(self.inner.receive()));
        match received {
            Ok(bytes) => {
                let text = String::from_utf8_lossy(&bytes);
                match codec::decode(&text) {
                    // Terminal Controller provides no joint samples in 4.0;
                    // lifecycle/state frames are not observations.
                    Ok(_) => Ok(None),
                    Err(_) => Ok(None),
                }
            }
            Err(crate::common::IoTransportError::Timeout) => Ok(None),
            Err(e) => Err(map_io(e)),
        }
    }
}

fn join_f64(values: &[f64]) -> String {
    values
        .iter()
        .map(|v| v.to_string())
        .collect::<Vec<_>>()
        .join(",")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::FakeTransport;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn handshake_negotiates_remote_capabilities() {
        let transport = FakeTransport::new();
        transport.inject_response(
            b"CAPABILITIES accepted=move_j;observations=joint_position;max_evidence=E1;target_resolution=host\n"
                .to_vec(),
        );
        let mut adapter = TerminalControllerAdapter::new(transport);

        let caps = adapter.handshake().await.unwrap();
        assert_eq!(caps.accepted, vec!["move_j"]);
        assert_eq!(caps.max_evidence, "E1");
        assert_eq!(adapter.state(), TransportState::Connected);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn handshake_rejected_leaves_adapter_faulted() {
        let transport = FakeTransport::new();
        transport.inject_response(b"REJECTED contract_version 2\n".to_vec());
        let mut adapter = TerminalControllerAdapter::new(transport);

        assert!(adapter.handshake().await.is_err());
        assert_eq!(adapter.state(), TransportState::Faulted);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn send_requires_a_completed_handshake() {
        let transport = FakeTransport::new();
        let mut adapter = TerminalControllerAdapter::new(transport);
        assert_eq!(
            adapter.send(RobotCommand::Stop),
            Err(TransportError::Disconnected)
        );
    }
}
