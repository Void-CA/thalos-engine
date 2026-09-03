use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use quick_xml::events::Event;
use quick_xml::reader::Reader;

#[derive(Debug, Clone, PartialEq)]
pub struct Triangle {
    pub normal: [f32; 3],
    pub v1: [f32; 3],
    pub v2: [f32; 3],
    pub v3: [f32; 3],
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct MeshGeometryData {
    pub triangles: Vec<Triangle>,
    /// Flattened vertices: [x0, y0, z0, x1, y1, z1, ...]
    pub vertices: Vec<f32>,
    /// Flattened normals per vertex (matching vertex count)
    pub normals: Vec<f32>,
}

#[derive(Debug, thiserror::Error)]
pub enum MeshLoaderError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Failed to parse STL: {0}")]
    StlParse(String),
    #[error("Failed to parse DAE: {0}")]
    DaeParse(String),
    #[error("Unsupported mesh format: {0}")]
    UnsupportedFormat(String),
}

/// Load an STL mesh from a file path.
pub fn load_stl<P: AsRef<Path>>(path: P) -> Result<MeshGeometryData, MeshLoaderError> {
    let path = path.as_ref();
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    let stl_data = stl::read_stl(&mut reader)
        .map_err(|e| MeshLoaderError::StlParse(format!("{:?}", e)))?;

    let mut triangles = Vec::with_capacity(stl_data.triangles.len());
    let mut vertices = Vec::with_capacity(stl_data.triangles.len() * 9);
    let mut normals = Vec::with_capacity(stl_data.triangles.len() * 9);

    for tri in stl_data.triangles {
        let n = tri.normal;
        let v1 = tri.v1;
        let v2 = tri.v2;
        let v3 = tri.v3;

        triangles.push(Triangle { normal: n, v1, v2, v3 });

        // Vertices
        vertices.extend_from_slice(&v1);
        vertices.extend_from_slice(&v2);
        vertices.extend_from_slice(&v3);

        // Normals per vertex
        normals.extend_from_slice(&n);
        normals.extend_from_slice(&n);
        normals.extend_from_slice(&n);
    }

    Ok(MeshGeometryData {
        triangles,
        vertices,
        normals,
    })
}

/// Load a Collada DAE mesh from a file path.
pub fn load_dae<P: AsRef<Path>>(path: P) -> Result<MeshGeometryData, MeshLoaderError> {
    let content = std::fs::read_to_string(path)?;
    parse_dae_xml(&content)
}

/// Parse Collada XML string into [`MeshGeometryData`].
pub fn parse_dae_xml(xml: &str) -> Result<MeshGeometryData, MeshLoaderError> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut float_arrays: HashMap<String, Vec<f32>> = HashMap::new();
    let mut vertices_map: HashMap<String, String> = HashMap::new();

    struct MeshPrimitive {
        pos_src_id: String,
        norm_src_id: Option<String>,
        p_indices: Vec<usize>,
        stride: usize,
        pos_offset: usize,
        norm_offset: Option<usize>,
        vcounts: Vec<usize>,
    }

    let mut primitives: Vec<MeshPrimitive> = Vec::new();
    let mut buf = Vec::new();

    let mut current_element = String::new();
    let mut current_source_id = String::new();
    let mut current_vertices_id = String::new();

    let mut current_pos_src = String::new();
    let mut current_norm_src: Option<String> = None;
    let mut current_pos_offset = 0;
    let mut current_norm_offset: Option<usize> = None;
    let mut current_stride = 0;
    let mut in_p = false;
    let mut in_vcount = false;
    let mut current_p = Vec::new();
    let mut current_vcounts = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                match name.as_str() {
                    "source" => {
                        current_source_id = e
                            .attributes()
                            .filter_map(|a| a.ok())
                            .find(|a| a.key.as_ref() == b"id")
                            .map(|a| String::from_utf8_lossy(&a.value).to_string())
                            .unwrap_or_default();
                    }
                    "vertices" => {
                        current_vertices_id = e
                            .attributes()
                            .filter_map(|a| a.ok())
                            .find(|a| a.key.as_ref() == b"id")
                            .map(|a| String::from_utf8_lossy(&a.value).to_string())
                            .unwrap_or_default();
                    }
                    "input" => {
                        let semantic = e
                            .attributes()
                            .filter_map(|a| a.ok())
                            .find(|a| a.key.as_ref() == b"semantic")
                            .map(|a| String::from_utf8_lossy(&a.value).to_string())
                            .unwrap_or_default();
                        let src = e
                            .attributes()
                            .filter_map(|a| a.ok())
                            .find(|a| a.key.as_ref() == b"source")
                            .map(|a| {
                                String::from_utf8_lossy(&a.value)
                                    .trim_start_matches('#')
                                    .to_string()
                            })
                            .unwrap_or_default();
                        let offset = e
                            .attributes()
                            .filter_map(|a| a.ok())
                            .find(|a| a.key.as_ref() == b"offset")
                            .and_then(|a| String::from_utf8_lossy(&a.value).parse::<usize>().ok())
                            .unwrap_or(0);

                        if !current_vertices_id.is_empty() && semantic == "POSITION" {
                            vertices_map.insert(current_vertices_id.clone(), src);
                        } else if current_element == "triangles" || current_element == "polylist" {
                            if offset >= current_stride {
                                current_stride = offset + 1;
                            }
                            if semantic == "VERTEX" {
                                current_pos_src = src;
                                current_pos_offset = offset;
                            } else if semantic == "NORMAL" {
                                current_norm_src = Some(src);
                                current_norm_offset = Some(offset);
                            }
                        }
                    }
                    "triangles" | "polylist" => {
                        current_element = name.clone();
                        current_pos_src.clear();
                        current_norm_src = None;
                        current_pos_offset = 0;
                        current_norm_offset = None;
                        current_stride = 1;
                        current_p.clear();
                        current_vcounts.clear();
                    }
                    "p" => {
                        in_p = true;
                    }
                    "vcount" => {
                        in_vcount = true;
                    }
                    _ => {
                        current_element = name;
                    }
                }
            }
            Ok(Event::Text(e)) => {
                let text = e.unescape().map_err(|err| MeshLoaderError::DaeParse(err.to_string()))?;
                if current_element == "float_array" && !current_source_id.is_empty() {
                    let floats: Vec<f32> = text
                        .split_whitespace()
                        .filter_map(|s| s.parse::<f32>().ok())
                        .collect();
                    float_arrays.insert(current_source_id.clone(), floats);
                } else if in_p {
                    current_p = text
                        .split_whitespace()
                        .filter_map(|s| s.parse::<usize>().ok())
                        .collect();
                } else if in_vcount {
                    current_vcounts = text
                        .split_whitespace()
                        .filter_map(|s| s.parse::<usize>().ok())
                        .collect();
                }
            }
            Ok(Event::End(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                match name.as_str() {
                    "source" => {
                        current_source_id.clear();
                    }
                    "vertices" => {
                        current_vertices_id.clear();
                    }
                    "p" => {
                        in_p = false;
                    }
                    "vcount" => {
                        in_vcount = false;
                    }
                    "triangles" | "polylist" => {
                        let pos_src_id = vertices_map
                            .get(&current_pos_src)
                            .cloned()
                            .unwrap_or_else(|| current_pos_src.clone());

                        primitives.push(MeshPrimitive {
                            pos_src_id,
                            norm_src_id: current_norm_src.clone(),
                            p_indices: std::mem::take(&mut current_p),
                            stride: current_stride.max(1),
                            pos_offset: current_pos_offset,
                            norm_offset: current_norm_offset,
                            vcounts: std::mem::take(&mut current_vcounts),
                        });
                        current_element.clear();
                    }
                    _ => {}
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => return Err(MeshLoaderError::DaeParse(e.to_string())),
            _ => {}
        }
        buf.clear();
    }

    let mut triangles = Vec::new();
    let mut vertices = Vec::new();
    let mut normals = Vec::new();

    for prim in primitives {
        let Some(pos_array) = float_arrays.get(&prim.pos_src_id) else {
            continue;
        };
        let norm_array = prim.norm_src_id.as_ref().and_then(|id| float_arrays.get(id));

        let stride = prim.stride;
        let p = &prim.p_indices;

        let poly_counts: Vec<usize> = if prim.vcounts.is_empty() {
            if stride * 3 > 0 && p.len() >= stride * 3 {
                vec![3; p.len() / (3 * stride)]
            } else {
                vec![]
            }
        } else {
            prim.vcounts
        };

        let mut p_idx = 0;
        for vcount in poly_counts {
            if vcount < 3 {
                p_idx += vcount * stride;
                continue;
            }
            for i in 1..(vcount - 1) {
                let corner_indices = [0, i, i + 1];
                let mut tri_verts = [[0.0f32; 3]; 3];
                let mut tri_norms = [[0.0f32; 3]; 3];
                let mut has_norms = norm_array.is_some() && prim.norm_offset.is_some();

                for (c, &corner) in corner_indices.iter().enumerate() {
                    let vert_p_idx = p_idx + corner * stride;
                    if vert_p_idx + prim.pos_offset < p.len() {
                        let pos_idx = p[vert_p_idx + prim.pos_offset];
                        if pos_idx * 3 + 2 < pos_array.len() {
                            tri_verts[c] = [
                                pos_array[pos_idx * 3],
                                pos_array[pos_idx * 3 + 1],
                                pos_array[pos_idx * 3 + 2],
                            ];
                        }
                    }

                    if has_norms {
                        if let (Some(n_arr), Some(n_off)) = (norm_array, prim.norm_offset) {
                            let norm_p_idx = vert_p_idx + n_off;
                            if norm_p_idx < p.len() {
                                let norm_idx = p[norm_p_idx];
                                if norm_idx * 3 + 2 < n_arr.len() {
                                    tri_norms[c] = [
                                        n_arr[norm_idx * 3],
                                        n_arr[norm_idx * 3 + 1],
                                        n_arr[norm_idx * 3 + 2],
                                    ];
                                } else {
                                    has_norms = false;
                                }
                            } else {
                                has_norms = false;
                            }
                        }
                    }
                }

                if !has_norms {
                    let u = [
                        tri_verts[1][0] - tri_verts[0][0],
                        tri_verts[1][1] - tri_verts[0][1],
                        tri_verts[1][2] - tri_verts[0][2],
                    ];
                    let v = [
                        tri_verts[2][0] - tri_verts[0][0],
                        tri_verts[2][1] - tri_verts[0][1],
                        tri_verts[2][2] - tri_verts[0][2],
                    ];
                    let face_normal = [
                        u[1] * v[2] - u[2] * v[1],
                        u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0],
                    ];
                    let len = (face_normal[0] * face_normal[0]
                        + face_normal[1] * face_normal[1]
                        + face_normal[2] * face_normal[2])
                        .sqrt();
                    let norm = if len > 1e-6 {
                        [face_normal[0] / len, face_normal[1] / len, face_normal[2] / len]
                    } else {
                        [0.0, 0.0, 1.0]
                    };
                    tri_norms = [norm, norm, norm];
                }

                triangles.push(Triangle {
                    normal: tri_norms[0],
                    v1: tri_verts[0],
                    v2: tri_verts[1],
                    v3: tri_verts[2],
                });

                for c in 0..3 {
                    vertices.extend_from_slice(&tri_verts[c]);
                    normals.extend_from_slice(&tri_norms[c]);
                }
            }
            p_idx += vcount * stride;
        }
    }

    Ok(MeshGeometryData {
        triangles,
        vertices,
        normals,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn load_valid_stl_file() {
        let mut tmp = NamedTempFile::new().unwrap();
        tmp.write_all(&[0u8; 80]).unwrap();
        tmp.write_all(&1u32.to_le_bytes()).unwrap();
        let normal = [0.0f32, 0.0f32, 1.0f32];
        let v1 = [0.0f32, 0.0f32, 0.0f32];
        let v2 = [1.0f32, 0.0f32, 0.0f32];
        let v3 = [0.0f32, 1.0f32, 0.0f32];

        for val in normal.iter().chain(v1.iter()).chain(v2.iter()).chain(v3.iter()) {
            tmp.write_all(&val.to_le_bytes()).unwrap();
        }
        tmp.write_all(&0u16.to_le_bytes()).unwrap();
        tmp.flush().unwrap();

        let mesh = load_stl(tmp.path()).unwrap();
        assert_eq!(mesh.triangles.len(), 1);
        assert_eq!(mesh.vertices.len(), 9);
        assert_eq!(mesh.normals.len(), 9);
        assert_eq!(mesh.triangles[0].normal, [0.0, 0.0, 1.0]);
    }

    #[test]
    fn load_invalid_stl_file() {
        let mut tmp = NamedTempFile::new().unwrap();
        tmp.write_all(b"not an stl file").unwrap();
        tmp.flush().unwrap();

        let res = load_stl(tmp.path());
        assert!(res.is_err());
    }

    #[test]
    fn load_valid_dae_file() {
        let dae_content = r##"<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASpec" version="1.4.1">
  <library_geometries>
    <geometry id="test-mesh">
      <mesh>
        <source id="test-positions">
          <float_array id="test-positions-array" count="9">0 0 0 1 0 0 0 1 0</float_array>
        </source>
        <source id="test-normals">
          <float_array id="test-normals-array" count="9">0 0 1 0 0 1 0 0 1</float_array>
        </source>
        <vertices id="test-vertices">
          <input semantic="POSITION" source="#test-positions"/>
        </vertices>
        <triangles count="1">
          <input semantic="VERTEX" source="#test-vertices" offset="0"/>
          <input semantic="NORMAL" source="#test-normals" offset="1"/>
          <p>0 0 1 1 2 2</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
</COLLADA>"##;

        let mesh = parse_dae_xml(dae_content).unwrap();
        assert_eq!(mesh.triangles.len(), 1);
        assert_eq!(mesh.vertices.len(), 9);
        assert_eq!(mesh.normals.len(), 9);
        assert_eq!(mesh.triangles[0].normal, [0.0, 0.0, 1.0]);
    }
}
