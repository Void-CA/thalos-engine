use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use crate::acquisition::lease::{AcquisitionLease, LeaseId};
use crate::acquisition::requirement::{AcquisitionRequirement, SamplingRequirement};
use thalos_ports::device::{ChannelId, ChannelObservation};
use crate::ports::device::transport::{
    ChannelSubscription, DeviceTransport, DeviceTransportError,
};

static NEXT_LEASE_ID: AtomicU64 = AtomicU64::new(1);

/// Internal state tracking active leases for a channel.
#[derive(Debug)]
struct ChannelLeaseState {
    leases: HashMap<LeaseId, u32>,
}

impl ChannelLeaseState {
    fn new() -> Self {
        Self {
            leases: HashMap::new(),
        }
    }

    fn max_target_hz(&self) -> u32 {
        self.leases.values().copied().max().unwrap_or(0)
    }
}

/// AcquisitionRuntime coordinates active leases and manages underlying DeviceTransport subscriptions.
///
/// Invariants:
/// 1. Subscribes to DeviceTransport ONLY when at least one active lease exists for a channel.
/// 2. Unsubscribes from DeviceTransport when the last lease for a channel is released.
/// 3. Ingested ChannelObservations are routed to an internal observation buffer.
pub struct AcquisitionRuntime<T: DeviceTransport> {
    transport: T,
    channel_states: HashMap<ChannelId, ChannelLeaseState>,
    observation_buffer: VecDeque<ChannelObservation>,
}

impl<T: DeviceTransport> AcquisitionRuntime<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            channel_states: HashMap::new(),
            observation_buffer: VecDeque::new(),
        }
    }

    /// Acquire an operational lease for a channel based on an AcquisitionRequirement.
    pub fn acquire_lease(
        &mut self,
        req: &AcquisitionRequirement,
    ) -> Result<AcquisitionLease, DeviceTransportError> {
        let target_hz = match req.sampling {
            SamplingRequirement::Continuous { target_hz } => target_hz,
            SamplingRequirement::OnDemand => 1,
        };

        let lease_id = LeaseId(NEXT_LEASE_ID.fetch_add(1, Ordering::SeqCst));
        let entry = self
            .channel_states
            .entry(req.channel_id.clone())
            .or_insert_with(ChannelLeaseState::new);

        entry.leases.insert(lease_id, target_hz);
        let max_hz = entry.max_target_hz();

        // If this is the first lease or target_hz changed, update DeviceTransport subscription
        self.transport.subscribe(ChannelSubscription {
            channel_id: req.channel_id.clone(),
            target_hz: max_hz,
        })?;

        Ok(AcquisitionLease {
            id: lease_id,
            channel_id: req.channel_id.clone(),
            target_hz,
        })
    }

    /// Release an operational lease. Unsubscribes from transport if no leases remain.
    pub fn release_lease(&mut self, lease: AcquisitionLease) -> Result<(), DeviceTransportError> {
        if let Some(entry) = self.channel_states.get_mut(&lease.channel_id) {
            entry.leases.remove(&lease.id);
            if entry.leases.is_empty() {
                self.channel_states.remove(&lease.channel_id);
                self.transport.unsubscribe(&lease.channel_id)?;
            } else {
                let max_hz = entry.max_target_hz();
                self.transport.subscribe(ChannelSubscription {
                    channel_id: lease.channel_id.clone(),
                    target_hz: max_hz,
                })?;
            }
        }
        Ok(())
    }

    /// Number of active leases for a channel.
    pub fn active_lease_count(&self, channel_id: &ChannelId) -> usize {
        self.channel_states
            .get(channel_id)
            .map(|s| s.leases.len())
            .unwrap_or(0)
    }

    /// Poll underlying transport for observations.
    pub fn tick(&mut self) -> Result<usize, DeviceTransportError> {
        let mut count = 0;
        while let Some(obs) = self.transport.try_receive()? {
            self.observation_buffer.push_back(obs);
            count += 1;
        }
        Ok(count)
    }

    /// Drain accumulated observations.
    pub fn drain_observations(&mut self) -> Vec<ChannelObservation> {
        self.observation_buffer.drain(..).collect()
    }

    /// Reference to underlying transport for test double assertions.
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Mutable reference to underlying transport.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::execution::observation::SignalQuality;
    use crate::test_support::FakeDeviceTransport;
    use thalos_ports::device::ChannelValue;

    #[test]
    fn acquisition_lease_lifecycle_manages_subscriptions() {
        let fake_transport = FakeDeviceTransport::new();
        let mut runtime = AcquisitionRuntime::new(fake_transport);

        let req1 = AcquisitionRequirement {
            channel_id: "temp_01".into(),
            sampling: SamplingRequirement::Continuous { target_hz: 10 },
            required: true,
        };

        // 1. Acquire lease 1 -> subscribes to transport
        let lease1 = runtime.acquire_lease(&req1).unwrap();
        assert_eq!(runtime.active_lease_count(&"temp_01".into()), 1);
        assert_eq!(
            runtime.transport().active_subscriptions.len(),
            1,
            "transport should have 1 active subscription"
        );

        // 2. Acquire lease 2 for same channel -> 2 leases, still 1 transport subscription
        let req2 = AcquisitionRequirement {
            channel_id: "temp_01".into(),
            sampling: SamplingRequirement::Continuous { target_hz: 50 },
            required: true,
        };
        let lease2 = runtime.acquire_lease(&req2).unwrap();
        assert_eq!(runtime.active_lease_count(&"temp_01".into()), 2);

        // 3. Release lease 1 -> 1 lease remains, transport subscription still active
        runtime.release_lease(lease1).unwrap();
        assert_eq!(runtime.active_lease_count(&"temp_01".into()), 1);
        assert_eq!(runtime.transport().active_subscriptions.len(), 1);

        // 4. Release lease 2 -> 0 leases remain, transport subscription removed (IDLE)
        runtime.release_lease(lease2).unwrap();
        assert_eq!(runtime.active_lease_count(&"temp_01".into()), 0);
        assert_eq!(
            runtime.transport().active_subscriptions.len(),
            0,
            "releasing all leases must unsubscribe transport"
        );
    }

    #[test]
    fn tick_ingests_and_drains_observations() {
        let fake_transport = FakeDeviceTransport::new();
        let mut runtime = AcquisitionRuntime::new(fake_transport);

        let req = AcquisitionRequirement {
            channel_id: "vibration_01".into(),
            sampling: SamplingRequirement::Continuous { target_hz: 100 },
            required: true,
        };
        let lease = runtime.acquire_lease(&req).unwrap();

        // Inject 2 observations into fake transport
        runtime.transport_mut().push_observation(ChannelObservation {
            channel_id: "vibration_01".into(),
            sampled_at_ns: 100,
            received_at_ns: 105,
            value: ChannelValue::Scalar(0.12),
            unit: Some("g".into()),
            quality: SignalQuality::Nominal,
        });
        runtime.transport_mut().push_observation(ChannelObservation {
            channel_id: "vibration_01".into(),
            sampled_at_ns: 200,
            received_at_ns: 205,
            value: ChannelValue::Scalar(0.15),
            unit: Some("g".into()),
            quality: SignalQuality::Nominal,
        });

        // Tick polls transport
        let count = runtime.tick().unwrap();
        assert_eq!(count, 2);

        let drained = runtime.drain_observations();
        assert_eq!(drained.len(), 2);
        assert_eq!(drained[0].value, ChannelValue::Scalar(0.12));
        assert_eq!(drained[1].value, ChannelValue::Scalar(0.15));

        runtime.release_lease(lease).unwrap();
    }
}
