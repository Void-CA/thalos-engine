use std::collections::VecDeque;
use super::transport::{RobotCommand, RobotObservation, RobotTransport, TransportError, TransportState};

/// In-memory fake RobotTransport for testing & simulation (L1 Test Double).
pub struct FakeRobotTransport {
    pub state: TransportState,
    pub sent_commands: Vec<RobotCommand>,
    pub observation_queue: VecDeque<RobotObservation>,
    pub should_fail_send: bool,
}

impl FakeRobotTransport {
    pub fn new() -> Self {
        Self {
            state: TransportState::Connected,
            sent_commands: Vec::new(),
            observation_queue: VecDeque::new(),
            should_fail_send: false,
        }
    }

    pub fn push_observation(&mut self, obs: RobotObservation) {
        self.observation_queue.push_back(obs);
    }
}

impl Default for FakeRobotTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl RobotTransport for FakeRobotTransport {
    fn state(&self) -> TransportState {
        self.state
    }

    fn send(&mut self, command: RobotCommand) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }
        if self.should_fail_send {
            return Err(TransportError::CommunicationFailure(
                "Simulated hardware transmission fault".to_string(),
            ));
        }
        self.sent_commands.push(command);
        Ok(())
    }

    fn stop(&mut self) -> Result<(), TransportError> {
        self.send(RobotCommand::Stop)
    }

    fn try_receive_observation(&mut self) -> Result<Option<RobotObservation>, TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::Disconnected);
        }
        Ok(self.observation_queue.pop_front())
    }
}
