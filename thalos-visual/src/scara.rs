use thalos_engine::core::{kinematics::forward::result::FKResult, robot::serial_chain::SerialChain};

use crate::builder::{SceneBuilder, cylinder_between};
use crate::scene::VisualScene;

/// Builder visual específico para el robot SCARA.
///
/// Genera primitives geométricas que representan la estructura visual del SCARA:
/// - **base column**: cilindro vertical en el origen (soporte fijo)
/// - **link 1 body**: cilindro desde world hasta link_1 (primer brazo)
/// - **link 2 body**: cilindro desde link_1 hasta link_2 (segundo brazo)
///
/// Usa el `FKResult` para posicionar cada primitiva, por lo que sigue
/// correctamente los cambios de configuración articular (q).
pub struct ScaraVisualBuilder;

impl ScaraVisualBuilder {
    /// Construye una `VisualScene` completa con frames, links, axes y primitives.
    pub fn build(fk: &FKResult, chain: &SerialChain) -> VisualScene {
        let builder = SceneBuilder::new(chain);
        let mut scene = builder.from_fk(fk);

        // Árbol actual: segment[0] = base (Fixed), segment[1] = joint1 → link1, segment[2] = joint2 → link2
        let base_id = &chain.segments[0].child;
        let link1_id = &chain.segments[1].child;
        let link2_id = &chain.segments[2].child;

        let base_pose = fk.pose(base_id).expect("SCARA must have base frame");
        let link1_pose = fk.pose(link1_id).expect("SCARA must have link_1 frame");
        let link2_pose = fk.pose(link2_id).expect("SCARA must have link_2 frame");

        let t_base: [f64; 3] = [
            base_pose.transform().translation.x,
            base_pose.transform().translation.y,
            base_pose.transform().translation.z,
        ];
        let t_link1: [f64; 3] = [
            link1_pose.transform().translation.x,
            link1_pose.transform().translation.y,
            link1_pose.transform().translation.z,
        ];
        let t_link2: [f64; 3] = [
            link2_pose.transform().translation.x,
            link2_pose.transform().translation.y,
            link2_pose.transform().translation.z,
        ];

        // ADR-0001: Z is vertical. Base height comes from Z component.
        let base_height = (t_base[2] - 0.0).abs();
        if base_height > 1e-6 {
            scene.primitives.push(cylinder_between(
                "base_column",
                "world",
                [0.0, 0.0, 0.0],
                t_base,
                0.08,
            ));
        }

        // 2. Link 1 — cilindro desde base frame hasta link_1 frame
        scene.primitives.push(cylinder_between(
            "link_1_body",
            "world",
            t_base,
            t_link1,
            0.045,
        ));

        // 3. Link 2 — cilindro desde link_1 hasta link_2
        scene.primitives.push(cylinder_between(
            "link_2_body",
            "world",
            t_link1,
            t_link2,
            0.035,
        ));

        scene
    }
}
