use std::collections::HashMap;

use quick_xml::Reader;
use quick_xml::events::Event;

use crate::candidate::{CandidateBody, CandidateJoint, ImportedCandidate};
use crate::urdf::attr::{attr, parse_n_floats, required_attr};
use crate::urdf::elements::{parse_global_material, parse_link_body, skip_element};
use crate::urdf::error::UrdfError;

/// Parse a URDF XML string into an [`ImportedCandidate`].
///
/// This is a pure XML→candidate conversion. It does not validate robot
/// semantics (root detection, axis requirements, etc.) — that is the
/// normalizer's responsibility.
pub fn parse(source: &str) -> Result<ImportedCandidate, UrdfError> {
    let mut reader = Reader::from_reader(source.as_bytes());
    let mut buf = Vec::new();

    let mut bodies: Vec<CandidateBody> = Vec::new();
    let mut raw_joints: Vec<CandidateJoint> = Vec::new();
    let mut materials = HashMap::new();
    let mut robot_name: Option<String> = None;

    loop {
        buf.clear();
        match reader.read_event_into(&mut buf)? {
            Event::Start(start) => {
                let e = start.into_owned();
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"robot" => {
                        robot_name = Some(required_attr(&e, b"name", "robot")?);
                    }
                    b"link" => {
                        let name = required_attr(&e, b"name", "link")?;
                        let mut link = thalos_models::link::Link::new(&name);
                        parse_link_body(&mut reader, &mut buf, &mut link)?;
                        bodies.push(candidate_body_from_link(&link));
                    }
                    b"joint" => {
                        let name = required_attr(&e, b"name", "joint")?;
                        let type_str = required_attr(&e, b"type", "joint")?;
                        let candidate = parse_candidate_joint(&mut reader, &mut buf, &type_str, name)?;
                        raw_joints.push(candidate);
                    }
                    b"material" => {
                        let mat = parse_global_material(&mut reader, &mut buf, e)?;
                        materials.insert(mat.name.clone(), mat);
                    }
                    _ => {
                        skip_element(&mut reader, &mut buf)?;
                    }
                }
            }
            Event::End(end) => {
                let tag = end.name().as_ref().to_ascii_lowercase();
                if tag == b"robot" {
                    break;
                }
            }
            Event::Empty(empty) => {
                let e = empty.into_owned();
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"link" {
                    let name = required_attr(&e, b"name", "link")?;
                    bodies.push(CandidateBody {
                        name,
                        parent_hint: None,
                        inertial: None,
                        visual: Vec::new(),
                        collision: Vec::new(),
                        visual_sources: Vec::new(),
                        collision_sources: Vec::new(),
                    });
                } else if tag == b"joint" {
                    let name = required_attr(&e, b"name", "joint")?;
                    let type_str = required_attr(&e, b"type", "joint")?;
                    let candidate = parse_candidate_joint(&mut reader, &mut buf, &type_str, name)?;
                    raw_joints.push(candidate);
                } else if tag == b"material" {
                    let mat = parse_global_material(&mut reader, &mut buf, e)?;
                    materials.insert(mat.name.clone(), mat);
                }
            }
            Event::Eof => {
                break;
            }
            _ => {}
        }
    }

    Ok(ImportedCandidate {
        name: robot_name.unwrap_or_else(|| "robot".to_string()),
        raw_bodies: bodies,
        raw_joints,
        materials,
        metadata: HashMap::new(),
    })
}

fn candidate_body_from_link(link: &thalos_models::link::Link) -> CandidateBody {
    let visual_sources = link
        .visual
        .iter()
        .filter_map(|v| match &v.geometry {
            thalos_models::geometry::Geometry::Mesh { filename, .. } => Some(filename.clone()),
            _ => None,
        })
        .collect();

    let collision_sources = link
        .collision
        .iter()
        .filter_map(|c| match &c.geometry {
            thalos_models::geometry::Geometry::Mesh { filename, .. } => Some(filename.clone()),
            _ => None,
        })
        .collect();

    CandidateBody {
        name: link.name.clone(),
        parent_hint: None,
        inertial: link.inertial.clone(),
        visual: link.visual.clone(),
        collision: link.collision.clone(),
        visual_sources,
        collision_sources,
    }
}

fn parse_candidate_joint<R: std::io::BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
    type_str: &str,
    name: String,
) -> Result<CandidateJoint, UrdfError> {
    let mut parent = None;
    let mut child = None;
    let mut origin_xyz: Option<[f64; 3]> = None;
    let mut origin_rpy: Option<[f64; 3]> = None;
    let mut axis: Option<[f64; 3]> = None;
    let mut lower_limit: Option<f64> = None;
    let mut upper_limit: Option<f64> = None;

    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"parent" => {
                        parent = Some(required_attr(&e, b"link", "parent")?);
                    }
                    b"child" => {
                        child = Some(required_attr(&e, b"link", "child")?);
                    }
                    b"origin" => {
                        if let Some(xyz_str) = attr(&e, b"xyz")? {
                            let v = parse_n_floats(&xyz_str, 3, "origin xyz")?;
                            origin_xyz = Some([v[0], v[1], v[2]]);
                        }
                        if let Some(rpy_str) = attr(&e, b"rpy")? {
                            let v = parse_n_floats(&rpy_str, 3, "origin rpy")?;
                            origin_rpy = Some([v[0], v[1], v[2]]);
                        }
                    }
                    b"axis" => {
                        if let Some(xyz_str) = attr(&e, b"xyz")? {
                            let v = parse_n_floats(&xyz_str, 3, "axis xyz")?;
                            axis = Some([v[0], v[1], v[2]]);
                        }
                    }
                    b"limit" => {
                        lower_limit = attr(&e, b"lower")?
                            .and_then(|s| s.parse::<f64>().ok());
                        upper_limit = attr(&e, b"upper")?
                            .and_then(|s| s.parse::<f64>().ok());
                    }
                    _ => {}
                }
            }
            Event::End(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"joint" {
                    break;
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <joint>".into()));
            }
            _ => {}
        }
    }

    let parent = parent.ok_or_else(|| UrdfError::MissingElement {
        parent: "joint".into(),
        child: "parent".into(),
    })?;
    let child = child.ok_or_else(|| UrdfError::MissingElement {
        parent: "joint".into(),
        child: "child".into(),
    })?;

    Ok(CandidateJoint {
        name,
        parent,
        child,
        joint_type: type_str.to_string(),
        axis,
        origin_xyz,
        origin_rpy,
        lower_limit,
        upper_limit,
    })
}
