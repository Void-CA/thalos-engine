use crate::spatial::frame::frame::FrameId;
use serde::{Deserialize, Serialize};
use thalos_math::Transform3D;
use thalos_math::Vector3;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    reference: FrameId,
    target: FrameId,
    transform: Transform3D,
}

impl Pose {
    pub fn new(reference: FrameId, target: FrameId, transform: Transform3D) -> Self {
        Self {
            reference,
            target,
            transform,
        }
    }

    /// World-referenced pose from a translation, identity rotation.
    ///
    /// Convenience for expressing an end-effector goal without constructing
    /// [`FrameId`]s by hand; `reference` and `target` are both [`FrameId::World`].
    pub fn from_xyz(x: f64, y: f64, z: f64) -> Self {
        Self::new(
            FrameId::World,
            FrameId::World,
            Transform3D::from_translation(Vector3::new(x, y, z)),
        )
    }

    /// World-referenced pose from an explicit translation and rotation.
    pub fn from_translation_rotation(
        translation: Vector3,
        rotation: thalos_math::UnitQuaternion,
    ) -> Self {
        Self::new(
            FrameId::World,
            FrameId::World,
            Transform3D::from_translation_rotation(translation, rotation),
        )
    }

    pub fn reference_id(&self) -> FrameId {
        self.reference
    }

    pub fn target_id(&self) -> FrameId {
        self.target
    }

    pub fn transform(&self) -> &Transform3D {
        &self.transform
    }

    pub fn is_global(&self) -> bool {
        self.reference_id() == FrameId::World
    }

    pub fn translation(&self) -> Vector3 {
        self.transform.translation
    }
}
