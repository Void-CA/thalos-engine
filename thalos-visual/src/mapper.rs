use thalos_engine::core::robot::serial_chain::SerialChain;
use thalos_engine::models::Robot;

use crate::scene::{PrimitiveGeometry, VisualElement};

use crate::asset_resolver::AssetResolver;
use crate::mesh_loader::{load_dae, load_stl};

fn material_color(material: &thalos_engine::models::Material) -> Option<[f64; 4]> {
    material.color.map(|c| [c.r, c.g, c.b, c.a])
}

fn to_primitive_with_resolver(
    geometry: &thalos_engine::models::geometry::Geometry,
    resolver: Option<&AssetResolver>,
) -> Option<PrimitiveGeometry> {
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
        thalos_engine::models::geometry::Geometry::Mesh { filename, scale } => {
            let mut vertices = Vec::new();
            let mut normals = Vec::new();

            if let Some(res) = resolver {
                if let Ok(resolved_path) = res.resolve(filename) {
                    let ext = resolved_path
                        .extension()
                        .and_then(|e| e.to_str())
                        .unwrap_or("")
                        .to_lowercase();
                    if ext == "stl" {
                        if let Ok(mesh_data) = load_stl(&resolved_path) {
                            vertices = mesh_data.vertices;
                            normals = mesh_data.normals;
                        }
                    } else if ext == "dae" {
                        if let Ok(mesh_data) = load_dae(&resolved_path) {
                            vertices = mesh_data.vertices;
                            normals = mesh_data.normals;
                        }
                    }
                }
            }

            Some(PrimitiveGeometry::Mesh {
                filename: filename.clone(),
                scale: scale.map(|s| [s.x, s.y, s.z]),
                vertices,
                normals,
            })
        }
    }
}

/// Extract visual elements from a URDF [`Robot`] without an asset resolver.
pub fn map_visuals(robot: &Robot, chain: &SerialChain) -> Vec<VisualElement> {
    map_visuals_with_resolver(robot, chain, None)
}

/// Extract visual elements from a URDF [`Robot`], resolving meshes via [`AssetResolver`].
pub fn map_visuals_with_resolver(
    robot: &Robot,
    chain: &SerialChain,
    resolver: Option<&AssetResolver>,
) -> Vec<VisualElement> {
    let mut elements = Vec::new();

    for (link_name, link) in &robot.links {
        if link.visual.is_empty() {
            continue;
        }

        let Some(frame_id) = chain.frames.resolve_by_name(link_name) else {
            continue;
        };

        for (idx, visual) in link.visual.iter().enumerate() {
            let Some(geometry) = to_primitive_with_resolver(&visual.geometry, resolver) else {
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
            to_primitive_with_resolver(&g, None),
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
            to_primitive_with_resolver(&g, None),
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
            to_primitive_with_resolver(&g, None),
            Some(PrimitiveGeometry::Cylinder {
                radius: 0.2,
                height: 1.0,
            })
        );
    }

    #[test]
    fn mesh_conversion_without_resolver() {
        let g = thalos_engine::models::geometry::Geometry::Mesh {
            filename: "foo.stl".into(),
            scale: None,
        };
        assert_eq!(
            to_primitive_with_resolver(&g, None),
            Some(PrimitiveGeometry::Mesh {
                filename: "foo.stl".into(),
                scale: None,
                vertices: vec![],
                normals: vec![],
            })
        );
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

    #[test]
    fn mesh_resolved_with_stl() {
        use std::io::Write;
        use tempfile::tempdir;

        let dir = tempdir().unwrap();
        let mesh_path = dir.path().join("link.stl");

        // Write a minimal 1-triangle binary STL file
        let mut f = std::fs::File::create(&mesh_path).unwrap();
        f.write_all(&[0u8; 80]).unwrap();
        f.write_all(&1u32.to_le_bytes()).unwrap();
        for val in [0.0f32, 0.0, 1.0,  0.0, 0.0, 0.0,  1.0, 0.0, 0.0,  0.0, 1.0, 0.0] {
            f.write_all(&val.to_le_bytes()).unwrap();
        }
        f.write_all(&0u16.to_le_bytes()).unwrap();
        f.flush().unwrap();

        let resolver = AssetResolver::new().with_base_dir(dir.path());
        let g = thalos_engine::models::geometry::Geometry::Mesh {
            filename: "link.stl".into(),
            scale: None,
        };

        let primitive = to_primitive_with_resolver(&g, Some(&resolver)).unwrap();
        if let PrimitiveGeometry::Mesh { vertices, normals, .. } = primitive {
            assert_eq!(vertices.len(), 9);
            assert_eq!(normals.len(), 9);
        } else {
            panic!("Expected PrimitiveGeometry::Mesh");
        }
    }
}
