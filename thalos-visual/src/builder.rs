use thalos_engine::core::{
    kinematics::{forward::result::FKResult, jacobian::Jacobian},
    robot::serial_chain::SerialChain,
    spatial::frame::FrameId,
};
use thalos_engine::math::{Transform3D, UnitQuaternion, UnitVector3, Vector3};

use crate::scene::*;

// ── Helpers geométricos para construir primitives ──

/// Normaliza un vector 3D.
fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-15 {
        return [0.0, 1.0, 0.0];
    }
    [v[0] / len, v[1] / len, v[2] / len]
}

/// Devuelve un quaternion `[w, x, y, z]` que rota el eje Y (0,1,0) para alinearse
/// con `direction`. Útil para orientar cilindros cuyo eje default es Y (Three.js).
pub fn align_y_to(direction: [f64; 3]) -> [f64; 4] {
    let dir = normalize(direction);
    let y = [0.0, 1.0, 0.0];
    let dot = y[0] * dir[0] + y[1] * dir[1] + y[2] * dir[2];

    // Misma dirección → identidad
    if dot > 0.9999 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    // Dirección opuesta → 180° alrededor de Z
    if dot < -0.9999 {
        return [0.0, 0.0, 0.0, 1.0];
    }

    // Producto vectorial: eje de rotación
    let axis = [
        y[1] * dir[2] - y[2] * dir[1],
        y[2] * dir[0] - y[0] * dir[2],
        y[0] * dir[1] - y[1] * dir[0],
    ];
    let axis = normalize(axis);

    let half = dot.acos() / 2.0;
    let s = half.sin();
    [half.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
}

/// Construye un `VisualPrimitive::Cylinder` que va desde `from` hasta `to`.
/// El cilindro queda centrado en el punto medio, con la altura = distancia y
/// orientación alineada con la dirección del segmento.
pub fn cylinder_between(
    id: impl Into<String>,
    frame_id: impl Into<String>,
    from: [f64; 3],
    to: [f64; 3],
    radius: f64,
) -> VisualPrimitive {
    let dx = to[0] - from[0];
    let dy = to[1] - from[1];
    let dz = to[2] - from[2];
    let height = (dx * dx + dy * dy + dz * dz).sqrt();

    if height < 1e-10 {
        return VisualPrimitive::cylinder(id, frame_id);
    }

    let midpoint = [
        (from[0] + to[0]) / 2.0,
        (from[1] + to[1]) / 2.0,
        (from[2] + to[2]) / 2.0,
    ];
    let rotation = align_y_to([dx / height, dy / height, dz / height]);

    VisualPrimitive {
        id: id.into(),
        frame_id: frame_id.into(),
        translation: midpoint,
        rotation,
        geometry: PrimitiveGeometry::Cylinder { radius, height },
        color: None,
    }
}

// ── SceneBuilder ──

pub struct SceneBuilder {
    chain: SerialChain,
    precision: VisualPrecision,
    frame_style: Option<FrameStyle>,
}

impl SceneBuilder {
    pub fn new(chain: &SerialChain) -> Self {
        Self {
            chain: chain.clone(),
            precision: VisualPrecision::default(),
            frame_style: None,
        }
    }

    pub fn with_precision(mut self, precision: VisualPrecision) -> Self {
        self.precision = precision;
        self
    }

    /// Override the automatic per-frame style (computed from kinematic extent).
    ///
    /// When `None` (default), `from_fk` computes the robot's maximum kinematic
    /// extent from the chain structure (sum of link translation norms) and
    /// creates a proportional `FrameStyle` via [`FrameStyle::scaled_by`].
    /// The extent is STABLE across FK config changes — computed once from the
    /// chain topology, not from current FK pose, so the grid and gizmo sizes
    /// don't jump on every joint angle change.
    pub fn with_frame_style(mut self, style: FrameStyle) -> Self {
        self.frame_style = Some(style);
        self
    }

    pub fn from_fk(&self, fk: &FKResult) -> VisualScene {
        let mut frames = Vec::new();
        let mut links = Vec::new();
        let mut joint_axes = Vec::new();

        // 1. Compute reference dimension from chain structure — the canonical
        //    chain-side factor (thalos-core::robot::scale::scene_reference_dimension).
        //    This is the maximum possible reach when fully stretched — stable
        //    across FK config changes, so the grid/gizmo sizes don't jump on FK
        //    movement. The chain-side function sums BOTH link translations AND
        //    joint origins (the URDF adapter stores lengths on origins, links
        //    are identity) — fixing the URDF path that degenerated to 0.01.
        //    The 0.01 floor stays as a visual guardrail for degenerate chains.
        let ref_dim = thalos_engine::core::robot::scale::scene_reference_dimension(&self.chain).max(0.01);

        // 2. Determine frame style: explicit override or auto-scaled
        let style = self
            .frame_style
            .clone()
            .unwrap_or_else(|| FrameStyle::scaled_by(ref_dim));

        let world_pose = fk
            .pose(&FrameId::World)
            .expect("FKResult must contain world frame");
        frames.push(VisualFrame {
            id: self.resolve_visual_id(&FrameId::World),
            parent: None,
            translation: self.normalize_tx(world_pose.transform()),
            rotation: self.normalize_rot(world_pose.transform()),
            style: Some(style.clone()),
        });

        for segment in &self.chain.segments {
            let child_pose = fk.pose(&segment.child).expect("Child frame pose not found");
            let parent_pose = fk
                .pose(&segment.parent)
                .expect("Parent frame pose not found");

            frames.push(VisualFrame {
                id: self.resolve_visual_id(&segment.child),
                parent: Some(self.resolve_visual_id(&segment.parent)),
                translation: self.normalize_tx(child_pose.transform()),
                rotation: self.normalize_rot(child_pose.transform()),
                style: Some(style.clone()),
            });

            // Fixed joints no tienen link ni axis visual (se dibujan como primitiva aparte)
            if segment.joint.dof() > 0 {
                links.push(VisualLink {
                    id: segment.joint.id(),
                    start: self.normalize_point(&parent_pose.transform().translation),
                    end: self.normalize_point(&child_pose.transform().translation),
                });

                let joint_transform = parent_pose.transform().compose(segment.joint.origin());
                let axis = segment.joint.axis_world(&joint_transform);

                joint_axes.push(VisualJointAxis {
                    origin: self.normalize_point(&joint_transform.translation),
                    axis: self.normalize_point(&axis),
                });
            }
        }

        VisualScene {
            frames,
            links,
            joint_axes,
            twists: Vec::new(),
            primitives: Vec::new(),
            reference_dimension: ref_dim,
        }
    }

    /// Extend a scene with visual elements as children of their link frames.
    ///
    /// Cada elemento se emite con su `frame_id` y transformación LOCAL
    /// (relativa al frame padre). El frontend las cuelga como hijas del
    /// `THREE.Group` del frame, de modo que mover el frame mueve la primitive.
    ///
    /// La corrección de cilindros (Z_urdf → Y_threejs) se aplica a la
    /// transformación local ANTES de emitir, no a la transformación mundo.
    pub fn with_visual_elements(&self, fk: &FKResult, elements: &[VisualElement]) -> VisualScene {
        let mut scene = self.from_fk(fk);

        for element in elements {
            let frame_visual_id = self.resolve_visual_id(&element.frame_id);

            let local = if matches!(element.geometry, PrimitiveGeometry::Cylinder { .. }) {
                // Cylinder correction: URDF defines cylinder axis as Z,
                // Three.js CylinderGeometry defaults to Y.
                // Post-multiply by +90° around X to map Z_urdf → Y_threejs.
                let correction = Transform3D::from_rotation(UnitQuaternion::from_axis_angle(
                    UnitVector3::x_axis(),
                    std::f64::consts::FRAC_PI_2,
                ));
                element.origin.compose(&correction)
            } else {
                element.origin.clone()
            };

            scene.primitives.push(VisualPrimitive {
                id: element.id.clone(),
                frame_id: frame_visual_id,
                translation: self.normalize_tx(&local),
                rotation: self.normalize_rot(&local),
                geometry: element.geometry.clone(),
                color: element.color,
            });
        }

        scene
    }

    pub fn from_fk_with_jacobian(&self, fk: &FKResult, jacobian: &Jacobian) -> VisualScene {
        let mut scene = self.from_fk(fk);

        let mut col = 0;
        for segment in self.chain.segments.iter() {
            // Fixed joints no contribuyen al Jacobiano
            if segment.joint.dof() == 0 {
                continue;
            }

            let parent_pose = fk.pose(&segment.parent).expect("Parent pose not found");
            let joint_transform = parent_pose.transform().compose(segment.joint.origin());

            scene.twists.push(VisualTwist {
                origin: self.normalize_point(&joint_transform.translation),
                linear: [
                    self.precision.normalize(jacobian.linear()[(0, col)]),
                    self.precision.normalize(jacobian.linear()[(1, col)]),
                    self.precision.normalize(jacobian.linear()[(2, col)]),
                ],
                angular: [
                    self.precision.normalize(jacobian.angular()[(0, col)]),
                    self.precision.normalize(jacobian.angular()[(1, col)]),
                    self.precision.normalize(jacobian.angular()[(2, col)]),
                ],
            });
            col += 1;
        }

        scene
    }

    fn resolve_visual_id(&self, id: &FrameId) -> VisualId {
        match id {
            FrameId::World => "world".into(),
            id => {
                if let Some(frame) = self.chain.frames.get(id) {
                    frame.name().to_string()
                } else {
                    eprintln!("[SceneBuilder] WARNING: frame {:?} not found in registry, using fallback", id);
                    format!("missing_{}", id)
                }
            }
        }
    }

    fn normalize_tx(&self, transform: &Transform3D) -> [f64; 3] {
        let mut arr = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ];
        self.precision.normalize_3(&mut arr);
        arr
    }

    fn normalize_rot(&self, transform: &Transform3D) -> [f64; 4] {
        let q = transform.rotation.inner();
        let mut arr = [q.w, q.x, q.y, q.z];
        self.precision.normalize_4(&mut arr);

        let norm = (arr[0] * arr[0] + arr[1] * arr[1] + arr[2] * arr[2] + arr[3] * arr[3]).sqrt();
        if norm > 1e-15 {
            for v in arr.iter_mut() {
                *v /= norm;
            }
        }

        self.precision.normalize_4(&mut arr);
        arr
    }

    fn normalize_point(&self, v: &Vector3) -> [f64; 3] {
        let mut arr = [v.x, v.y, v.z];
        self.precision.normalize_3(&mut arr);
        arr
    }
}
