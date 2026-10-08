//! Consumer scenario **B1 — load a robot** — *resolved in slice D5*.
//!
//! # Intent
//! "Give me a robot I can query." → `Manipulator::from_urdf(source)`.
//!
//! # What the consumer must know now (after D5)
//! - `intent::{Manipulator, CartesianTarget}` — and nothing else.
//!
//! # Mechanism now hidden
//! Before D5 the consumer assembled the load by hand: `import_urdf` →
//! `models::Robot` → `adapter::from_robot` → `SerialChain`. That pipeline
//! (`URDF → model → chain extraction → SerialChain`) is now inside
//! `Manipulator::from_urdf`.
//!
//! # Gate (D5)
//! This file cannot name `SerialChain`, `Robot`, `adapter::from_urdf`,
//! `FrameId` or `ForwardKinematics`. The engine performs no I/O, so the URDF is
//! passed as **source text**, not a path.

use thalos_api::intent::{CartesianTarget, Manipulator};

const PLANAR_2R_URDF: &str = r#"
<robot name="planar_2r">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="tool0"/>
  <joint name="j_tool" type="fixed">
    <parent link="link2"/><child link="tool0"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"#;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Manipulator::from_urdf(PLANAR_2R_URDF)?;

    let solution = robot.inverse(CartesianTarget::position(1.0, 0.5, 0.0))?;

    println!(
        "loaded '{}' ({} DOF) → IK converged={} joints={:?}",
        robot.end_effector_name().unwrap_or("?"),
        robot.dof(),
        solution.converged(),
        solution.joints()
    );
    Ok(())
}
