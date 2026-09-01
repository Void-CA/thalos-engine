use thalos_engine::core::robot::serial_chain::SerialChain;
use thalos_engine::models::Robot;

use crate::scene::{PrimitiveGeometry, VisualElement};

fn material_color(material: &thalos_engine::models::Material) -> Option<[f64; 4]> {
    material.color.map(|c| [c.r, c.g, c.b, c.a])
}

fn to_primitive(geometry: &thalos_engine::models::geometry::Geometry) -> Option<PrimitiveGeometry> {
    match geometry {
        thalos_engine::models::geometry::Geometry::Sphere { radius } => {
            Some(PrimitiveGeometry::Sphere { radius: *radius })
        }
        thalos_engine::models::geometry::Geometry::Box {
            width,
            height,
            depth,
        } => Some(PrimitiveGeometry::Box {
            width: *width,
            height: *height,
            depth: *depth,
        }),
        thalos_engine::models::geometry::Geometry::Cylinder { radius, height } => {
            Some(PrimitiveGeometry::Cylinder {
                radius: *radius,
                height: *height,
            })
        }
        thalos_engine::models::geometry::Geometry::Mesh { .. } => None,
    }
}

/// Extract visual elements from a URDF [`Robot`].
///
/// Returns one [`VisualElement`] per `<visual>` entry, skipping meshes
/// and any link not found in the chain's frame registry. Each element
/// carries its link's `FrameId` so the [`SceneBuilder`] can resolve
/// world-space positions via FK without repeated name lookups.
pub fn map_visuals(robot: &Robot, chain: &SerialChain) -> Vec<VisualElement> {
    let mut elements = Vec::new();

    for (link_name, link) in &robot.links {
        if link.visual.is_empty() {
            continue;
        }

        let Some(frame_id) = chain.frames.resolve_by_name(link_name) else {
            continue;
        };

        for (idx, visual) in link.visual.iter().enumerate() {
            let Some(geometry) = to_primitive(&visual.geometry) else {
                continue;
            };

            elements.push(VisualElement {
                id: format!("{}_{}", link_name, idx),
                frame_id,
                origin: visual.origin.clone(),
                geometry,
                color: visual.material.as_ref().and_then(material_color),
            });
        }
    }

    elements
}

// ── Tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_conversion() {
        let g = thalos_engine::models::geometry::Geometry::Sphere { radius: 0.5 };
        assert_eq!(
            to_primitive(&g),
            Some(PrimitiveGeometry::Sphere { radius: 0.5 })
        );
    }

    #[test]
    fn box_conversion() {
        let g = thalos_engine::models::geometry::Geometry::Box {
            width: 1.0,
            height: 2.0,
            depth: 3.0,
        };
        assert_eq!(
            to_primitive(&g),
            Some(PrimitiveGeometry::Box {
                width: 1.0,
                height: 2.0,
                depth: 3.0,
            })
        );
    }

    #[test]
    fn cylinder_conversion() {
        let g = thalos_engine::models::geometry::Geometry::Cylinder {
            radius: 0.2,
            height: 1.0,
        };
        assert_eq!(
            to_primitive(&g),
            Some(PrimitiveGeometry::Cylinder {
                radius: 0.2,
                height: 1.0,
            })
        );
    }

    #[test]
    fn mesh_is_skipped() {
        let g = thalos_engine::models::geometry::Geometry::Mesh {
            filename: "foo.stl".into(),
            scale: None,
        };
        assert_eq!(to_primitive(&g), None);
    }

    #[test]
    fn map_scara_urdf() {
        let src = include_str!("../../thalos-models/tests/fixtures/scara.urdf");
        let robot = thalos_importer::import_urdf(src).unwrap();
        let chain = thalos_engine::core::robot::adapter::auto(&robot).unwrap();

        let elements = map_visuals(&robot, &chain);

        // SCARA fixture has 5 links with visuals:
        //   base_link  → 1 cylinder
        //   link_1     → 1 cylinder
        //   link_2     → 1 cylinder
        //   link_3     → 1 cylinder
        //   tool0      → 1 sphere
        assert_eq!(elements.len(), 5, "expected 5 visual elements");

        // Each element should reference a valid frame in the chain
        for el in &elements {
            assert!(
                chain.frames.get(&el.frame_id).is_some(),
                "element '{}' references unknown frame",
                el.id,
            );
        }
    }
}
