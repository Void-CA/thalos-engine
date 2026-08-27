use crate::Material;
use thalos_math::{Transform3D, Vector3};

// ─── Primitive shapes ──────────────────────────────────────────

/// A sphere centred at the local origin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    pub radius: f64,
}

impl Sphere {
    pub const fn new(radius: f64) -> Self {
        Self { radius }
    }
}

/// An axis-aligned box defined by half-extents in the local frame.
///
/// Half-extents are half the width/height/depth along each axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Box3D {
    /// Half-width, half-height, half-depth in the local frame.
    pub half_extents: Vector3,
}

impl Box3D {
    /// Create a box from full width, height, and depth.
    pub fn new(width: f64, height: f64, depth: f64) -> Self {
        Self {
            half_extents: Vector3::new(width / 2.0, height / 2.0, depth / 2.0),
        }
    }

    /// Create a box directly from half-extents.
    pub fn from_half_extents(half_extents: Vector3) -> Self {
        Self { half_extents }
    }
}

/// A cylinder with its longitudinal axis aligned to the local Y axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cylinder {
    pub radius: f64,
    pub height: f64,
}

impl Cylinder {
    pub const fn new(radius: f64, height: f64) -> Self {
        Self { radius, height }
    }
}

/// A mesh loaded from an external file.
#[derive(Debug, Clone, PartialEq)]
pub struct Mesh {
    pub filename: String,
    pub scale: Option<Vector3>,
}

/// A shape used in both visual and collision elements.
///
/// Named-field variant — URDF-native representation.
#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    Sphere {
        radius: f64,
    },
    Box {
        width: f64,
        height: f64,
        depth: f64,
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Mesh {
        filename: String,
        scale: Option<Vector3>,
    },
}

/// Geometry type used by the collision-detection system.
///
/// Wraps primitive shapes for efficient overlap tests.
#[derive(Debug, Clone, PartialEq)]
pub enum CollisionGeometry {
    Sphere(Sphere),
    Box(Box3D),
    Cylinder(Cylinder),
}

// ─── Visual / Collision descriptions ───────────────────────────

/// Visual element of a link.
///
/// Corresponds to `<visual>` in URDF.
#[derive(Debug, Clone, PartialEq)]
pub struct Visual {
    pub origin: Transform3D,
    pub geometry: Geometry,
    pub material: Option<Material>,
}

impl Visual {
    pub fn new(origin: Transform3D, geometry: Geometry) -> Self {
        Self {
            origin,
            geometry,
            material: None,
        }
    }
}

/// Collision element of a link.
///
/// Corresponds to `<collision>` in URDF.
#[derive(Debug, Clone, PartialEq)]
pub struct Collision {
    pub origin: Transform3D,
    pub geometry: Geometry,
}

impl Collision {
    pub fn new(origin: Transform3D, geometry: Geometry) -> Self {
        Self { origin, geometry }
    }
}
