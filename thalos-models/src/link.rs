use crate::{Collision, Visual};
use thalos_math::Transform3D;

/// Inertial properties of a link.
///
/// Corresponds to `<inertial>` in URDF.
#[derive(Debug, Clone, PartialEq)]
pub struct Inertial {
    /// Pose of the inertial frame relative to the link frame.
    pub origin: Transform3D,
    /// Mass in kg.
    pub mass: f64,
    /// 3×3 inertia tensor (symmetric) expressed at the CoM in the
    /// inertial frame.
    pub inertia: InertiaMatrix,
}

/// The six unique elements of a symmetric 3×3 inertia matrix.
///
/// | Axis | Moment       | Product     |
/// |------|-------------|-------------|
/// | x    | `ixx`       |             |
/// | y    | `iyy`       | `ixy`       |
/// | z    | `izz`       | `ixz`, `iyz` |
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertiaMatrix {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

/// A rigid body in the robot description.
///
/// A `Link` is purely a named body with optional inertial, visual, and
/// collision properties. **It does not carry a pose** — the spatial
/// relationship between links is defined by [`Joint`](crate::Joint).
///
/// Corresponds to `<link>` in URDF.
#[derive(Debug, Clone, PartialEq)]
pub struct Link {
    pub name: String,
    pub inertial: Option<Inertial>,
    pub visual: Vec<Visual>,
    pub collision: Vec<Collision>,
}

impl Link {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            inertial: None,
            visual: Vec::new(),
            collision: Vec::new(),
        }
    }
}
