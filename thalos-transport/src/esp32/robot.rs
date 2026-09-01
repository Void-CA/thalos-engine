use std::collections::VecDeque;
use thalos_ports::robot::{
    RobotCommand, RobotObservation, RobotTransport, TransportError, TransportState,
};
use thalos_ports::SignalQuality;

use crate::common::Transport;
use crate::esp32::codec::{Esp32Codec, Esp32Frame};

/// Physical ESP32 Robot Transport Adapter (ADR-014).
///
/// Wraps a byte-level `Transport` (Serial/TCP) and adapts protocol frames to `RobotTransport`.
pub struct Esp32RobotAdapter<T: Transport> {
    inner_transport: T,
    state: TransportState,
    observation_queue: VecDeque<RobotObservation>,
    sequence: u64,
}

impl<T: Transport> Esp32RobotAdapter<T> {
    pub fn new(inner_transport: T) -> Self {
        Self {
            inner_transport,
            state: TransportState::Disconnected,
            observation_queue: VecDeque::new(),
            sequence: 0,
        }
    }

    pub async fn connect(&mut self) -> Result<(), TransportError> {
        self.inner_transport
            .connect()
            .await
            .map_err(|e| TransportError::CommunicationFailure(e.to_string()))?;
        self.state = TransportState::Connected;
        Ok(())
    }

    pub async fn disconnect(&mut self) -> Result<(), TransportError> {
        self.inner_transport
            .disconnect()
            .await
            .map_err(|e| TransportError::CommunicationFailure(e.to_string()))?;
        self.state = TransportState::Disconnected;
        Ok(())
    }

    pub fn push_observation(&mut self, obs: RobotObservation) {
        self.observation_queue.push_back(obs);
    }
}

impl<T: Transport> RobotTransport for Esp32RobotAdapter<T> {
    fn state(&self) -> TransportState {
        self.state
    }

    fn send(&mut self, command: RobotCommand) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }

        match command {
            RobotCommand::MoveJoints {
                positions_rad,
                velocities_rad_s: _,
            } => {
                let frame = Esp32Codec::encode_sample(&positions_rad, 10_000);
                let handle = tokio::runtime::Handle::current();
                let send_res = tokio::task::block_in_place(|| {
                    handle.block_on(self.inner_transport.send(frame.as_bytes()))
                });
                send_res.map_err(|e| TransportError::CommunicationFailure(e.to_string()))?;
            }
            RobotCommand::Stop => {
                let frame = Esp32Codec::encode_stop();
                let handle = tokio::runtime::Handle::current();
                let send_res = tokio::task::block_in_place(|| {
                    handle.block_on(self.inner_transport.send(frame.as_bytes()))
                });
                send_res.map_err(|e| TransportError::CommunicationFailure(e.to_string()))?;
            }
        }

        Ok(())
    }

    fn stop(&mut self) -> Result<(), TransportError> {
        self.send(RobotCommand::Stop)
    }

    fn try_receive_observation(&mut self) -> Result<Option<RobotObservation>, TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }

        // Return queued observations
        if let Some(obs) = self.observation_queue.pop_front() {
            return Ok(Some(obs));
        }

        // Try reading line non-blockingly or from buffer
        let handle = tokio::runtime::Handle::current();
        let recv_res = tokio::task::block_in_place(|| handle.block_on(self.inner_transport.receive()));

        match recv_res {
            Ok(bytes) => {
                let line = String::from_utf8_lossy(&bytes);
                if let Ok(Esp32Frame::SampleFrame { timestamp_us, values }) = Esp32Codec::parse_response(&line) {
                    self.sequence += 1;
                    let obs = RobotObservation {
                        sampled_at_ns: timestamp_us * 1_000,
                        sequence: self.sequence,
                        joint_positions_rad: values.clone(),
                        joint_velocities_rad_s: vec![0.0; values.len()],
                        tcp_pose: None,
                        signal_quality: SignalQuality::Nominal,
                    };
                    return Ok(Some(obs));
                }
                Ok(None)
            }
            Err(crate::common::TransportError::Timeout) => Ok(None),
            Err(e) => Err(TransportError::CommunicationFailure(e.to_string())),
        }
    }
}
