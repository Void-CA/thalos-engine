use thalos_math::Transform3D;

/// Junta fija (0 DOF).
///
/// Aporta un `origin` (transformación de montaje) y un `link` en el
/// segmento, pero **no** consume coordenadas generalizadas ni contribuye
/// al Jacobiano. Sirve para modelar bases, tool frames, sensores, etc.
#[derive(Debug, Clone)]
pub struct FixedJoint {
    /// Transformación fija de montaje (posición/orientación relativa al frame padre).
    pub origin: Transform3D,
}

impl FixedJoint {
    pub fn new(origin: Transform3D) -> Self {
        Self { origin }
    }

    /// Siempre devuelve identidad — un joint fijo no produce movimiento.
    pub fn motion(&self, _q: f64) -> Transform3D {
        Transform3D::identity()
    }

    pub fn origin(&self) -> &Transform3D {
        &self.origin
    }
}
