use std::collections::{HashSet, VecDeque};
use thalos_ports::device::{
    ChannelId, ChannelObservation, ChannelSubscription, DeviceTransport, DeviceTransportError,
};
use thalos_ports::robot::TransportState;

/// In-memory fake DeviceTransport for acquisition runtime testing (L2 Test Double).
pub struct FakeDeviceTransport {
    pub state: TransportState,
    pub active_subscriptions: HashSet<ChannelSubscription>,
    pub observation_queue: VecDeque<ChannelObservation>,
}

impl FakeDeviceTransport {
    pub fn new() -> Self {
        Self {
            state: TransportState::Connected,
            active_subscriptions: HashSet::new(),
            observation_queue: VecDeque::new(),
        }
    }

    pub fn push_observation(&mut self, obs: ChannelObservation) {
        self.observation_queue.push_back(obs);
    }
}

impl Default for FakeDeviceTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceTransport for FakeDeviceTransport {
    fn state(&self) -> TransportState {
        self.state
    }

    fn subscribe(&mut self, subscription: ChannelSubscription) -> Result<(), DeviceTransportError> {
        if self.state != TransportState::Connected {
            return Err(DeviceTransportError::Disconnected);
        }
        self.active_subscriptions
            .retain(|s| s.channel_id != subscription.channel_id);
        self.active_subscriptions.insert(subscription);
        Ok(())
    }

    fn unsubscribe(&mut self, channel_id: &ChannelId) -> Result<(), DeviceTransportError> {
        if self.state != TransportState::Connected {
            return Err(DeviceTransportError::Disconnected);
        }
        self.active_subscriptions
            .retain(|s| &s.channel_id != channel_id);
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<ChannelObservation>, DeviceTransportError> {
        if self.state != TransportState::Connected {
            return Err(DeviceTransportError::Disconnected);
        }
        Ok(self.observation_queue.pop_front())
    }
}
