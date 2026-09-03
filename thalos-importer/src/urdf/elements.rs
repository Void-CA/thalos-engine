use std::io::BufRead;

use quick_xml::Reader;
use quick_xml::events::Event;

use crate::urdf::attr::{attr, parse_origin, parse_rgba, parse_xyz, required_attr};
use crate::urdf::error::UrdfError;
use thalos_math::{Transform3D, UnitVector3};
use thalos_models::geometry::{Collision, Geometry, Visual};
use thalos_models::joint::{Joint, JointKind, JointLimits};
use thalos_models::link::{InertiaMatrix, Inertial, Link};
use thalos_models::material::Material;

pub fn skip_element<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
) -> Result<(), UrdfError> {
    let mut depth: usize = 1;
    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(_) => depth += 1,
            Event::End(_) => {
                depth -= 1;
                if depth == 0 {
                    return Ok(());
                }
            }
            Event::Empty(_) => {}
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF during skip".into()));
            }
            _ => {}
        }
    }
}

pub fn parse_link_body<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
    link: &mut Link,
) -> Result<(), UrdfError> {
    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"inertial" => {
                        link.inertial = Some(parse_inertial(reader, buf)?);
                    }
                    b"visual" => {
                        link.visual.push(parse_visual(reader, buf)?);
                    }
                    b"collision" => {
                        link.collision.push(parse_collision(reader, buf)?);
                    }
                    _ => {
                        skip_element(reader, buf)?;
                    }
                }
            }
            Event::Empty(_) => {}
            Event::End(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"link" {
                    return Ok(());
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <link>".into()));
            }
            _ => {}
        }
    }
}

fn parse_inertial<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
) -> Result<Inertial, UrdfError> {
    let mut origin = Transform3D::identity();
    let mut mass = None;
    let mut inertia = None;

    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"origin" => {
                        origin = parse_origin(&e)?;
                    }
                    b"mass" => {
                        let s = required_attr(&e, b"value", "mass")?;
                        mass = Some(s.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
                            value: s,
                            source: e2.to_string(),
                        })?);
                    }
                    b"inertia" => {
                        let ixx = required_attr(&e, b"ixx", "inertia")?;
                        let ixy = required_attr(&e, b"ixy", "inertia")?;
                        let ixz = required_attr(&e, b"ixz", "inertia")?;
                        let iyy = required_attr(&e, b"iyy", "inertia")?;
                        let iyz = required_attr(&e, b"iyz", "inertia")?;
                        let izz = required_attr(&e, b"izz", "inertia")?;
                        let parse = |s: &str| {
                            s.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
                                value: s.to_string(),
                                source: e2.to_string(),
                            })
                        };
                        inertia = Some(InertiaMatrix {
                            ixx: parse(&ixx)?,
                            ixy: parse(&ixy)?,
                            ixz: parse(&ixz)?,
                            iyy: parse(&iyy)?,
                            iyz: parse(&iyz)?,
                            izz: parse(&izz)?,
                        });
                    }
                    _ => {}
                }
            }
            Event::End(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"inertial" {
                    break;
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <inertial>".into()));
            }
            _ => {}
        }
    }

    let mass = mass.ok_or_else(|| UrdfError::MissingElement {
        parent: "inertial".into(),
        child: "mass".into(),
    })?;
    let inertia = inertia.ok_or_else(|| UrdfError::MissingElement {
        parent: "inertial".into(),
        child: "inertia".into(),
    })?;

    Ok(Inertial {
        origin,
        mass,
        inertia,
    })
}

fn parse_geometry_body<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
) -> Result<Geometry, UrdfError> {
    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"sphere" => {
                        let r = required_attr(&e, b"radius", "sphere")?;
                        let radius = r.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
                            value: r,
                            source: e2.to_string(),
                        })?;
                        return Ok(Geometry::Sphere { radius });
                    }
                    b"box" => {
                        let s = required_attr(&e, b"size", "box")?;
                        let dims = parse_xyz(&s, "box")?;
                        return Ok(Geometry::Box {
                            width: dims.x,
                            height: dims.y,
                            depth: dims.z,
                        });
                    }
                    b"cylinder" => {
                        let r = required_attr(&e, b"radius", "cylinder")?;
                        let h = required_attr(&e, b"length", "cylinder")?;
                        let radius = r.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
                            value: r,
                            source: e2.to_string(),
                        })?;
                        let height = h.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
                            value: h,
                            source: e2.to_string(),
                        })?;
                        return Ok(Geometry::Cylinder { radius, height });
                    }
                    b"mesh" => {
                        let filename = required_attr(&e, b"filename", "mesh")?;
                        let scale = match attr(&e, b"scale")? {
                            Some(s) => Some(parse_xyz(&s, "mesh")?),
                            None => None,
                        };
                        return Ok(Geometry::Mesh { filename, scale });
                    }
                    _ => {}
                }
            }
            Event::End(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"geometry" {
                    return Err(UrdfError::MissingElement {
                        parent: "geometry".into(),
                        child: "sphere|box|cylinder|mesh".into(),
                    });
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <geometry>".into()));
            }
            _ => {}
        }
    }
}

fn parse_visual<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
) -> Result<Visual, UrdfError> {
    let mut origin = Transform3D::identity();
    let mut geometry = None;
    let mut material = None;

    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"origin" => {
                        origin = parse_origin(&e)?;
                    }
                    b"material" => {
                        let name = attr(&e, b"name")?.unwrap_or_default();
                        material = Some(Material { name, color: None, texture: None });
                    }
                    _ => {}
                }
            }
            Event::Start(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"origin" => {
                        origin = parse_origin(&e)?;
                    }
                    b"geometry" => {
                        geometry = Some(parse_geometry_body(reader, buf)?);
                    }
                    b"material" => {
                        let e = e.into_owned();
                        material = Some(parse_visual_material(reader, buf, e)?);
                    }
                    _ => {}
                }
            }
            Event::End(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"visual" {
                    break;
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <visual>".into()));
            }
            _ => {}
        }
    }

    let geometry = geometry.ok_or_else(|| UrdfError::MissingElement {
        parent: "visual".into(),
        child: "geometry".into(),
    })?;

    Ok(Visual {
        origin,
        geometry,
        material,
    })
}

fn parse_visual_material<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
    start: quick_xml::events::BytesStart<'_>,
) -> Result<Material, UrdfError> {
    let name = required_attr(&start, b"name", "material").unwrap_or_default();
    let mut color = None;

    // Self-closing <material name="..."/> — no children to parse
    if start.is_empty() {
        return Ok(Material {
            name,
            color,
            texture: None,
        });
    }

    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"color" {
                    let s = required_attr(&e, b"rgba", "color")?;
                    color = Some(parse_rgba(&s, "color")?);
                }
            }
            Event::End(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"material" {
                    break;
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <material>".into()));
            }
            _ => {}
        }
    }

    Ok(Material {
        name,
        color,
        texture: None,
    })
}

fn parse_collision<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
) -> Result<Collision, UrdfError> {
    let mut origin = Transform3D::identity();
    let mut geometry = None;

    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"origin" => {
                        origin = parse_origin(&e)?;
                    }
                    b"geometry" => {
                        geometry = Some(parse_geometry_body(reader, buf)?);
                    }
                    _ => {}
                }
            }
            Event::End(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"collision" {
                    break;
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <collision>".into()));
            }
            _ => {}
        }
    }

    let geometry = geometry.ok_or_else(|| UrdfError::MissingElement {
        parent: "collision".into(),
        child: "geometry".into(),
    })?;

    Ok(Collision { origin, geometry })
}

pub fn parse_joint_body<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
    kind: JointKind,
    name: String,
) -> Result<Joint, UrdfError> {
    let mut origin = Transform3D::identity();
    let mut parent = None;
    let mut child = None;
    let mut axis = None;
    let mut limits = None;

    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                match tag.as_slice() {
                    b"origin" => {
                        origin = parse_origin(&e)?;
                    }
                    b"parent" => {
                        parent = Some(required_attr(&e, b"link", "parent")?);
                    }
                    b"child" => {
                        child = Some(required_attr(&e, b"link", "child")?);
                    }
                    b"axis" => {
                        let s = required_attr(&e, b"xyz", "axis")?;
                        let v = parse_xyz(&s, "axis")?;
                        if v.norm() < 1e-12 {
                            return Err(UrdfError::ZeroAxis);
                        }
                        axis = Some(UnitVector3::new(v).map_err(|_| UrdfError::ZeroAxis)?);
                    }
                    b"limit" => {
                        limits = Some(parse_limit(&e)?);
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

    Ok(Joint {
        name,
        kind,
        parent,
        child,
        origin,
        axis,
        limits,
    })
}

fn parse_limit(elem: &quick_xml::events::BytesStart<'_>) -> Result<JointLimits, UrdfError> {
    let lower = match attr(elem, b"lower")? {
        Some(s) => s.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
            value: s,
            source: e2.to_string(),
        })?,
        None => 0.0,
    };
    let upper = match attr(elem, b"upper")? {
        Some(s) => s.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
            value: s,
            source: e2.to_string(),
        })?,
        None => 0.0,
    };
    let velocity = match attr(elem, b"velocity")? {
        Some(s) => Some(s.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
            value: s,
            source: e2.to_string(),
        })?),
        None => None,
    };
    let effort = match attr(elem, b"effort")? {
        Some(s) => Some(s.parse::<f64>().map_err(|e2| UrdfError::ParseFloat {
            value: s,
            source: e2.to_string(),
        })?),
        None => None,
    };

    Ok(JointLimits {
        min: lower,
        max: upper,
        velocity,
        effort,
        enabled: true,
    })
}

pub fn parse_global_material<R: BufRead>(
    reader: &mut Reader<R>,
    buf: &mut Vec<u8>,
    start: quick_xml::events::BytesStart<'_>,
) -> Result<Material, UrdfError> {
    let name = required_attr(&start, b"name", "material")?;
    let mut color = None;

    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let tag = e.name().as_ref().to_ascii_lowercase();
                if tag == b"color" {
                    let s = required_attr(&e, b"rgba", "color")?;
                    color = Some(parse_rgba(&s, "color")?);
                }
            }
            Event::End(e) => {
                if e.name().as_ref().eq_ignore_ascii_case(b"material") {
                    break;
                }
            }
            Event::Eof => {
                return Err(UrdfError::Xml("unexpected EOF inside <material>".into()));
            }
            _ => {}
        }
    }

    Ok(Material {
        name,
        color,
        texture: None,
    })
}

pub fn parse_joint_type(s: &str) -> Result<JointKind, UrdfError> {
    match s {
        "revolute" => Ok(JointKind::Revolute),
        "continuous" => Ok(JointKind::Continuous),
        "prismatic" => Ok(JointKind::Prismatic),
        "fixed" => Ok(JointKind::Fixed),
        "floating" => Ok(JointKind::Floating),
        "planar" => Ok(JointKind::Planar),
        other => Err(UrdfError::UnknownJointType(other.to_string())),
    }
}
