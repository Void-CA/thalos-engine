//! Consumer scenario **B2 — forward kinematics** — *aligned in the D1–D5 audit*.
//!
//! # Intent
//! "Given these joints, where is the tool?" → `robot.forward(&joints)`.
//!
//! # What the consumer must know now
//! - `intent::{Manipulator, TcpPose}` — and nothing else.
//!
//! # Mechanism now hidden
//! FK was already served by `KinematicContext` (the precedent of a well-designed
//! intent API), but it was not reachable from the robot value and its example
//! built a chain from the catalog. It is now `Manipulator::forward`, delegating
//! internally to the same shared authority.
//!
//! # Gate
//! This file cannot name `ForwardKinematics`, `KinematicContext`, `SerialChain`
//! or `FrameId`.

#[path = "common/mod.rs"]
mod common;

use thalos_api::intent::{Manipulator, TcpPose};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Manipulator::from_urdf(common::PLANAR_2R_URDF)?;

    let joints = [0.3, 0.4];
    let tcp: TcpPose = robot.forward(&joints)?;

    println!("FK at {joints:?} → position {:?}", tcp.position);
    Ok(())
}
