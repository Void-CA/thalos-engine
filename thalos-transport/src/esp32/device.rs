use std::collections::{HashMap, HashSet, VecDeque};
use thalos_ports::device::{
    ChannelId, ChannelObservation, ChannelSubscription, ChannelValue, DeviceTransport,
    DeviceTransportError,
};
use thalos_ports::robot::TransportState;
use thalos_ports::SignalQuality;

use crate::esp32::codec::{Esp32Codec, Esp32Frame};

/// Binding mapping a raw ESP32 telemetry channel index to a Thalos ChannelId.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelBinding {
    pub channel_index: usize,
    pub channel_id: ChannelId,
    pub unit: Option<String>,
}

/// Physical/Simulated ESP32 IIoT Telemetry Device Adapter (ADR-014).
///
/// Decodes raw ESP32 wire frames (`SAMPLE <ts_us> <v0> <v1>...`) into semantic `ChannelObservation`s.
pub struct Esp32DeviceAdapter<T: DeviceTransport> {
    inner_transport: T,
    bindings: HashMap<ChannelId, ChannelBinding>,
    active_subscriptions: HashSet<ChannelSubscription>,
    observation_queue: VecDeque<ChannelObservation>,
}

impl<T: DeviceTransport> Esp32DeviceAdapter<T> {
    pub fn new(inner_transport: T, bindings: Vec<ChannelBinding>) -> Self {
        let binding_map = bindings
            .into_iter()
            .map(|b| (b.channel_id.clone(), b))
            .collect();

        Self {
            inner_transport,
            bindings: binding_map,
            active_subscriptions: HashSet::new(),
            observation_queue: VecDeque::new(),
        }
    }

    /// Process a raw wire response line from ESP32 into semantic observations.
    pub fn process_raw_line(&mut self, line: &str) -> Result<usize, DeviceTransportError> {
        let parsed = Esp32Codec::parse_response(line)
            .map_err(|e| DeviceTransportError::Transport(e.to_string()))?;

        let mut count = 0;
        if let Esp32Frame::SampleFrame {
            timestamp_us,
            values,
        } = parsed
        {
            let received_at_ns = timestamp_us * 1_000;

            for sub in &self.active_subscriptions {
                if let Some(binding) = self.bindings.get(&sub.channel_id) {
                    if let Some(&raw_value) = values.get(binding.channel_index) {
                        let obs = ChannelObservation {
                            channel_id: binding.channel_id.clone(),
                            sampled_at_ns: timestamp_us * 1_000,
                            received_at_ns,
                            value: ChannelValue::Scalar(raw_value),
                            unit: binding.unit.clone(),
                            quality: SignalQuality::Nominal,
                        };
                        self.observation_queue.push_back(obs);
                        count += 1;
                    }
                }
            }
        }

        Ok(count)
    }

    pub fn inner_transport(&self) -> &T {
        &self.inner_transport
    }

    pub fn inner_transport_mut(&mut self) -> &mut T {
        &mut self.inner_transport
    }
}

impl<T: DeviceTransport> DeviceTransport for Esp32DeviceAdapter<T> {
    fn state(&self) -> TransportState {
        self.inner_transport.state()
    }

    fn subscribe(&mut self, subscription: ChannelSubscription) -> Result<(), DeviceTransportError> {
        if !self.bindings.contains_key(&subscription.channel_id) {
            return Err(DeviceTransportError::ChannelNotFound(
                subscription.channel_id.clone(),
            ));
        }

        self.inner_transport.subscribe(subscription.clone())?;
        self.active_subscriptions
            .retain(|s| s.channel_id != subscription.channel_id);
        self.active_subscriptions.insert(subscription);
        Ok(())
    }

    fn unsubscribe(&mut self, channel_id: &ChannelId) -> Result<(), DeviceTransportError> {
        self.inner_transport.unsubscribe(channel_id)?;
        self.active_subscriptions
            .retain(|s| &s.channel_id != channel_id);
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<ChannelObservation>, DeviceTransportError> {
        if let Some(obs) = self.observation_queue.pop_front() {
            return Ok(Some(obs));
        }
        self.inner_transport.try_receive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestTransport {
        state: TransportState,
        active_subscriptions: HashSet<ChannelSubscription>,
    }

    impl TestTransport {
        fn new() -> Self {
            Self {
                state: TransportState::Connected,
                active_subscriptions: HashSet::new(),
            }
        }
    }

    impl DeviceTransport for TestTransport {
        fn state(&self) -> TransportState {
            self.state
        }
        fn subscribe(&mut self, subscription: ChannelSubscription) -> Result<(), DeviceTransportError> {
            self.active_subscriptions.insert(subscription);
            Ok(())
        }
        fn unsubscribe(&mut self, channel_id: &ChannelId) -> Result<(), DeviceTransportError> {
            self.active_subscriptions.retain(|s| &s.channel_id != channel_id);
            Ok(())
        }
        fn try_receive(&mut self) -> Result<Option<ChannelObservation>, DeviceTransportError> {
            Ok(None)
        }
    }

    #[test]
    fn decodes_sample_frame_into_channel_observation() {
        let fake = TestTransport::new();
        let bindings = vec![ChannelBinding {
            channel_index: 0,
            channel_id: "temp_sensor".into(),
            unit: Some("°C".into()),
        }];

        let mut adapter = Esp32DeviceAdapter::new(fake, bindings);
        adapter
            .subscribe(ChannelSubscription {
                channel_id: "temp_sensor".into(),
                target_hz: 10,
            })
            .unwrap();

        let count = adapter.process_raw_line("SAMPLE 1000 24.5\n").unwrap();
        assert_eq!(count, 1);

        let obs = adapter.try_receive().unwrap().unwrap();
        assert_eq!(obs.channel_id, "temp_sensor");
        assert_eq!(obs.value, ChannelValue::Scalar(24.5));
    }
}
