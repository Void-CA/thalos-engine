//! Consumer scenario **B8 — frames and poses**.
//!
//! # Intent
//! "Express a pose relative to a frame" without caring how frames are numbered.
//!
//! # What the consumer must know today
//! - `core::spatial::frame::{FrameId, FrameRegistry}` — frame identity + registry
//! - `core::spatial::pose::Pose` — the pose type
//! - `math::{Transform3D, Vector3}` — the transform vocabulary
//!
//! # Mechanism leaked
//! A `Pose` is expressed over `FrameId`s (`reference`, `target`). Even a simple
//! "10 cm along X" requires knowing `FrameId` and the registry — internal
//! plumbing. `FrameId::World` is a value the consumer must know to say
//! "in world coordinates".
//!
//! # Verdict
//! Tier 3 **mixta** (see `docs/API-ERGONOMICS.md` §1): the transform vocabulary
//! is domain; `FrameId` in pose construction is accidental.
//!
//! ```ignore
//! // Does not exist today:
//! let pose = Pose::from_xyz(0.1, 0.0, 0.2);
//! ```

use thalos_api::core::spatial::frame::{FrameId, FrameRegistry};
use thalos_api::core::spatial::pose::Pose;
use thalos_api::math::{Transform3D, Vector3};

fn main() {
    let mut frames = FrameRegistry::new();
    let base = frames.create("base");
    let tool = frames.create("tool0");

    // A pose is defined over frame identities, not over a point.
    let offset = Transform3D::from_translation(Vector3::new(0.1, 0.0, 0.2));
    let pose = Pose::new(base, tool, offset);

    // "In world coordinates" means knowing FrameId::World.
    let world_pose = Pose::new(FrameId::World, tool, Transform3D::identity());

    println!(
        "local pose {} → {} = {:?}",
        pose.reference_id(),
        pose.target_id(),
        pose.translation()
    );
    println!("world pose is global = {}", world_pose.is_global());
}
