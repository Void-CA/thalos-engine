//! Canonical robot model types.
//!
//! This crate defines **what a robot is**: its structure, components,
//! and physical properties. Every type here is pure data — no kinematic
//! algorithms, no runtime state, no frame systems.
//!
//! These types map 1:1 to URDF concepts and can be serialised without
//! loss of meaning.
//!
//! ── Sub-modules ──────────────────────────────────────────────────
//!
//! | Module      | Contains                                     |
//! |-------------|----------------------------------------------|
//! | `robot`     | `Robot`, the top-level container             |
//! | `link`      | `Link`, `Inertial`                           |
//! | `joint`     | `Joint`, `JointKind`, `JointLimits`          |
//! | `geometry`  | `Geometry`, `Visual`, `Collision`            |
//! | `material`  | `Material`, `Color`                          |
//! | `graph`     | `RobotGraph`, `Path`, `LinkId`, `JointId`    |
pub mod geometry;
pub mod graph;
pub mod joint;
pub mod link;
pub mod material;
pub mod robot;
pub mod robot_asset;

pub use geometry::{Box3D, Collision, CollisionGeometry, Cylinder, Geometry, Mesh, Sphere, Visual};
pub use graph::{JointId, LinkId, Path, RobotGraph};
pub use joint::{Joint, JointKind, JointLimits};
pub use link::{Inertial, Link};
pub use material::{Color, Material};
pub use robot::Robot;
pub use robot_asset::{AssetRole, RobotAsset};
