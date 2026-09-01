pub mod diff;
pub mod precision;

use serde::{Deserialize, Serialize};
use thalos_engine::core::robot::joint::JointId;
use thalos_engine::core::spatial::frame::FrameId;
use thalos_engine::math::Transform3D;

pub use diff::{ChangedFrame, SceneDiff};
pub use precision::VisualPrecision;

pub type VisualId = String;

// ── Geometrías primitivas ──────────────────────────────────────────────

/// Forma geométrica de un elemento visual primitivo.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum PrimitiveGeometry {
    Cylinder { radius: f64, height: f64 },
    Sphere { radius: f64 },
    Box { width: f64, height: f64, depth: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct VisualPrimitive {
    pub id: VisualId,
    /// ID visual del frame padre. La primitive es hija de este frame en el
    /// scene graph del frontend — mover el frame mueve la primitive.
    pub frame_id: VisualId,
    /// Transformación LOCAL (relativa al frame padre).
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    pub geometry: PrimitiveGeometry,
    /// RGBA color from URDF `<material>`, or `None` if unspecified.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<[f64; 4]>,
}

impl VisualPrimitive {
    pub fn cylinder(id: impl Into<VisualId>, frame_id: impl Into<VisualId>) -> Self {
        Self {
            id: id.into(),
            frame_id: frame_id.into(),
            translation: [0.0; 3],
            rotation: [1.0, 0.0, 0.0, 0.0],
            geometry: PrimitiveGeometry::Cylinder {
                radius: 0.01,
                height: 0.01,
            },
            color: None,
        }
    }

    pub fn sphere(id: impl Into<VisualId>, frame_id: impl Into<VisualId>) -> Self {
        Self {
            id: id.into(),
            frame_id: frame_id.into(),
            translation: [0.0; 3],
            rotation: [1.0, 0.0, 0.0, 0.0],
            geometry: PrimitiveGeometry::Sphere { radius: 0.01 },
            color: None,
        }
    }

    pub fn box_shape(id: impl Into<VisualId>, frame_id: impl Into<VisualId>) -> Self {
        Self {
            id: id.into(),
            frame_id: frame_id.into(),
            translation: [0.0; 3],
            rotation: [1.0, 0.0, 0.0, 0.0],
            geometry: PrimitiveGeometry::Box {
                width: 0.01,
                height: 0.01,
                depth: 0.01,
            },
            color: None,
        }
    }

    pub fn with_translation(mut self, translation: [f64; 3]) -> Self {
        self.translation = translation;
        self
    }

    pub fn with_rotation(mut self, rotation: [f64; 4]) -> Self {
        self.rotation = rotation;
        self
    }

    pub fn with_color(mut self, color: [f64; 4]) -> Self {
        self.color = Some(color);
        self
    }
}

/// A visual element extracted from a robot model, before world-space
/// resolution.
///
/// Carries a `frame_id` (link frame) and a local `origin` so the
/// `SceneBuilder` can resolve the world transform via FK:
///
/// ```text
/// world_pose = fk.pose(frame_id) * origin
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct VisualElement {
    pub id: VisualId,
    /// Frame of the link this visual belongs to (resolved once during
    /// mapping so that per-frame rendering doesn't need name lookups).
    pub frame_id: FrameId,
    /// Local origin of the visual element relative to the link frame.
    pub origin: Transform3D,
    /// Primitive geometry (Sphere, Cylinder, Box).
    pub geometry: PrimitiveGeometry,
    /// RGBA color from URDF `<material>`, or `None` if unspecified.
    pub color: Option<[f64; 4]>,
}

// ── Escala de referencia por defecto (1 m = robot "promedio") ──

fn default_ref_dim() -> f64 {
    1.0
}

// ── Escena visual ──────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct VisualScene {
    pub frames: Vec<VisualFrame>,
    pub links: Vec<VisualLink>,
    pub joint_axes: Vec<VisualJointAxis>,
    pub twists: Vec<VisualTwist>,
    pub primitives: Vec<VisualPrimitive>,
    /// Dimensión de referencia del robot: la mayor distancia desde el origen
    /// a cualquier frame en la configuración actual. El frontend la usa para
    /// escalar grid, gizmos y la cámara Fit Robot.
    ///
    /// Serde default para backward compat: robots cargados con versiones
    /// viejas del backend que no emiten este campo reciben 1.0 (comportamiento
    /// original: `GridHelper(4,10)` y `DEFAULT_FRAME_STYLE`).
    #[serde(default = "default_ref_dim")]
    pub reference_dimension: f64,
}

impl Default for VisualScene {
    fn default() -> Self {
        Self {
            frames: vec![],
            links: vec![],
            joint_axes: vec![],
            twists: vec![],
            primitives: vec![],
            reference_dimension: default_ref_dim(),
        }
    }
}

/// Estilo visual de un frame (cómo renderizar sus ejes y origen).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FrameStyle {
    /// Longitud de cada eje (desde el origen).
    pub axis_length: f64,
    /// Radio de los cilindros que representan cada eje (0 = línea).
    pub axis_radius: f64,
    /// Radio de la esfera en el origen (0 = sin esfera).
    pub origin_radius: f64,
    /// Si se deben mostrar etiquetas (X/Y/Z) cerca del extremo de cada eje.
    pub show_labels: bool,
    /// Color RGB del eje X (cada componente en 0..1).
    pub color_x: [f64; 3],
    /// Color RGB del eje Y.
    pub color_y: [f64; 3],
    /// Color RGB del eje Z.
    pub color_z: [f64; 3],
}

impl Default for FrameStyle {
    fn default() -> Self {
        Self {
            axis_length: 0.18,
            axis_radius: 0.006,
            origin_radius: 0.0,
            show_labels: false,
            color_x: [1.0, 0.5, 0.0], // naranja
            color_y: [0.0, 0.8, 0.0], // verde
            color_z: [0.0, 0.5, 1.0], // azul
        }
    }
}

impl FrameStyle {
    /// Crea un `FrameStyle` escalado proporcionalmente al tamaño del robot.
    ///
    /// `max_dim` es la dimensión máxima del robot (distancia desde el origen
    /// al frame más lejano). Los valores de `axis_length` y `axis_radius`
    /// se escalan al ~10 % del tamaño del robot, que es el estándar en tools
    /// como Rviz:
    ///
    /// - `axis_length = max_dim × 0.10` (mín. 5 mm)
    /// - `axis_radius = max_dim × 0.003` (mín. 0.5 mm)
    pub fn scaled_by(max_dim: f64) -> Self {
        let dim = max_dim.max(0.01);
        Self {
            axis_length: (dim * 0.10).max(0.005),
            axis_radius: (dim * 0.003).max(0.0005),
            ..Default::default()
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct VisualFrame {
    pub id: VisualId,
    pub parent: Option<VisualId>,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    /// Estilo visual opcional. `None` = usar defaults del frontend.
    #[serde(default)]
    pub style: Option<FrameStyle>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct VisualLink {
    pub id: JointId,
    pub start: [f64; 3],
    pub end: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct VisualJointAxis {
    pub origin: [f64; 3],
    pub axis: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct VisualTwist {
    pub origin: [f64; 3],
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}
