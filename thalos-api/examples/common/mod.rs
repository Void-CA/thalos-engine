//! Shared fixture for the consumer scenarios.
//!
//! Not an example itself (no `main.rs` sibling, so cargo does not build it as a
//! target); examples include it via `#[path = "common/mod.rs"] mod common;`.

#![allow(dead_code)]

/// A planar 2R with a fixed tool offset.
///
/// The fixed joint to `tool0` matters: without it the end-effector would sit on
/// the last joint axis, leaving its Jacobian column zero and making IK unable to
/// use that DOF.
pub const PLANAR_2R_URDF: &str = r#"
<robot name="planar_2r">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool0"/>
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
  <joint name="j_tool" type="fixed">
    <parent link="link2"/><child link="tool0"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"#;
