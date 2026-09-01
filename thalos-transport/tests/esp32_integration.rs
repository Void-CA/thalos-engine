use thalos_ports::robot::{RobotCommand, RobotObservation, RobotTransport, TransportState};
use thalos_ports::SignalQuality;
use thalos_transport::{FakeTransport, esp32::Esp32RobotAdapter};

#[tokio::test(flavor = "multi_thread")]
async fn esp32_robot_adapter_connect_send_stop() {

    let fake = FakeTransport::new();
    let mut adapter = Esp32RobotAdapter::new(fake);

    assert_eq!(adapter.state(), TransportState::Disconnected);
    adapter.connect().await.unwrap();
    assert_eq!(adapter.state(), TransportState::Connected);

    adapter
        .send(RobotCommand::MoveJoints {
            positions_rad: vec![0.5, 0.3],
            velocities_rad_s: None,
        })
        .unwrap();

    adapter.stop().unwrap();
    adapter.disconnect().await.unwrap();
    assert_eq!(adapter.state(), TransportState::Disconnected);
}

#[tokio::test]
async fn esp32_robot_adapter_queue_observations() {
    let fake = FakeTransport::new();
    let mut adapter = Esp32RobotAdapter::new(fake);
    adapter.connect().await.unwrap();

    let obs = RobotObservation {
        sampled_at_ns: 1000,
        sequence: 1,
        joint_positions_rad: vec![0.1, 0.2],
        joint_velocities_rad_s: vec![0.0, 0.0],
        tcp_pose: None,
        signal_quality: SignalQuality::Nominal,
    };

    adapter.push_observation(obs.clone());
    let recv = adapter.try_receive_observation().unwrap();
    assert_eq!(recv, Some(obs));
}
