use crate::{UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

/// Transformación rígida 3D (traslación + rotación).
///
/// La rotación está representada con un [`UnitQuaternion`] para garantizar
/// que sea una rotación válida en SO(3) (norma = 1).
///
/// La traslación es un [`Vector3`] cualquiera.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Transform3D {
    pub translation: Vector3,
    pub rotation: UnitQuaternion,
}

impl Transform3D {
    pub fn from_translation_rotation(translation: Vector3, rotation: UnitQuaternion) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    pub fn from_translation(translation: Vector3) -> Self {
        Self {
            translation,
            rotation: UnitQuaternion::identity(),
        }
    }

    pub fn from_rotation(rotation: UnitQuaternion) -> Self {
        Self {
            translation: Vector3::zero(),
            rotation,
        }
    }

    pub fn identity() -> Self {
        Self {
            translation: Vector3::zero(),
            rotation: UnitQuaternion::identity(),
        }
    }

    pub fn compose(&self, other: &Self) -> Self {
        let translation = self.translation + self.rotation.rotate_vector(other.translation);
        let rotation = self.rotation * other.rotation;

        Self {
            translation,
            rotation,
        }
    }

    pub fn apply(&self, point: Vector3) -> Vector3 {
        self.rotation.rotate_vector(point) + self.translation
    }
}

impl std::fmt::Display for Transform3D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Traslation:(\n  {}],\n  {}\n)",
            self.translation, self.rotation
        )
    }
}
