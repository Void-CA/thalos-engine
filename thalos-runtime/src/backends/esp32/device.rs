use std::collections::{HashMap, HashSet, VecDeque};
use crate::execution::observation::SignalQuality;
use thalos_ports::device::{ChannelId, ChannelObservation, ChannelValue};
use crate::ports::device::transport::{
    ChannelSubscription, DeviceTransport, DeviceTransportError,
};
use crate::ports::robot::transport::TransportState;
use crate::backends::esp32::protocol::{Esp32Protocol, ParsedResponse};

/// Configured semantic binding mapping a raw wire channel index to a Thalos ChannelId.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelBinding {
    pub channel_index: usize,
    pub channel_id: ChannelId,
    pub unit: Option<String>,
}

/// Infrastructure adapter connecting physical/simulated ESP32 wire frames to DeviceTransport.
///
/// Converts wire-level sample frames (`SAMPLE <ts_us> <v0> <v1>...`) into semantic `ChannelObservation`s
/// based on configured `ChannelBinding`s.
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

    /// Process a raw wire line (e.g. `SAMPLE <ts_us> <val0> <val1>...`) into semantic observations.
    pub fn process_raw_line(&mut self, line: &str) -> Result<usize, DeviceTransportError> {
        let parsed = Esp32Protocol::parse_response(line)
            .map_err(|e| DeviceTransportError::Transport(e.to_string()))?;

        let mut count = 0;
        if let ParsedResponse::Sample(sample) = parsed {
            let received_at_ns = sample.timestamp_us * 1_000;

            for sub in &self.active_subscriptions {
                if let Some(binding) = self.bindings.get(&sub.channel_id) {
                    if let Some(&raw_value) = sample.joints.get(binding.channel_index) {
                        let obs = ChannelObservation {
                            channel_id: binding.channel_id.clone(),
                            sampled_at_ns: sample.timestamp_us * 1_000,
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

    /// Access inner transport reference for testing.
    pub fn inner_transport(&self) -> &T {
        &self.inner_transport
    }

    /// Access inner transport mutable reference for testing.
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
        // First drain queued observations parsed from wire frames
        if let Some(obs) = self.observation_queue.pop_front() {
            return Ok(Some(obs));
        }

        // Delegate to inner transport
        self.inner_transport.try_receive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::acquisition::requirement::{AcquisitionRequirement, SamplingRequirement};
    use crate::acquisition::runtime::AcquisitionRuntime;
    use crate::test_support::FakeDeviceTransport;

    #[test]
    fn esp32_wire_frame_decodes_into_semantic_channel_observation() {
        let fake = FakeDeviceTransport::new();
        let bindings = vec![
            ChannelBinding {
                channel_index: 0,
                channel_id: "chamber.temperature".into(),
                unit: Some("°C".into()),
            },
            ChannelBinding {
                channel_index: 1,
                channel_id: "chamber.humidity".into(),
                unit: Some("%".into()),
            },
        ];

        let adapter = Esp32DeviceAdapter::new(fake, bindings);
        let mut runtime = AcquisitionRuntime::new(adapter);

        // 1. Acquire lease for chamber.temperature
        let req = AcquisitionRequirement {
            channel_id: "chamber.temperature".into(),
            sampling: SamplingRequirement::Continuous { target_hz: 10 },
            required: true,
        };
        let lease = runtime.acquire_lease(&req).unwrap();

        // 2. Process wire frame: "SAMPLE 5000 26.4 61.2"
        let wire_line = "SAMPLE 5000 26.4 61.2\n";
        let count = runtime.transport_mut().process_raw_line(wire_line).unwrap();
        assert_eq!(count, 1, "only subscribed channel should produce observation");

        // 3. Tick runtime and verify observation
        runtime.tick().unwrap();
        let obs = runtime.drain_observations();
        assert_eq!(obs.len(), 1);
        assert_eq!(obs[0].channel_id, "chamber.temperature");
        assert_eq!(obs[0].value, ChannelValue::Scalar(26.4));
        assert_eq!(obs[0].unit.as_deref(), Some("°C"));

        runtime.release_lease(lease).unwrap();
    }
}
