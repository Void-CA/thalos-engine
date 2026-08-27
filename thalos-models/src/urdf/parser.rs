//! URDF → [`Robot`](crate::Robot) parser.
//!
//! ## Usage
//!
//! ```rust
//! # use thalos_models::urdf::parser::parse_robot;
//! let source = r#"
//!   <robot name="my_bot">
//!     <link name="base_link"/>
//!     <link name="head">
//!       <visual>
//!         <geometry><sphere radius="0.1"/></geometry>
//!       </visual>
//!     </link>
//!     <joint name="neck" type="revolute">
//!       <parent link="base_link"/>
//!       <child  link="head"/>
//!       <origin xyz="0 0 0.5" rpy="0 0 0"/>
//!       <axis  xyz="0 0 1"/>
//!     </joint>
//!   </robot>
//! "#;
//!
//! let robot = parse_robot(source).unwrap();
//! assert_eq!(robot.name, "my_bot");
//! assert_eq!(robot.links.len(), 2);
//! assert_eq!(robot.joints.len(), 1);
//! ```

use std::collections::HashMap;

use quick_xml::Reader;
use quick_xml::events::Event;

use crate::link::Link;
use crate::material::Material;
use crate::robot::Robot;
use crate::urdf::attr::required_attr;
use crate::urdf::elements::{
    parse_global_material, parse_joint_body, parse_joint_type, parse_link_body, skip_element,
};

// Re-exported for backward compatibility with downstream code that
// imports via `thalos_models::urdf::parser::UrdfError`.
pub use crate::urdf::error::UrdfError;

/// Parse a URDF document into a [`Robot`](crate::Robot).
///
/// The root link is determined automatically: the link that is never
/// the child of a joint. If every link appears as a child (cyclic
/// graph) or there are no links, the first link is used.
pub fn parse_robot(source: &str) -> Result<Robot, UrdfError> {
    let mut reader = Reader::from_reader(source.as_bytes());

    let mut buf = Vec::new();

    let mut links: Vec<Link> = Vec::new();
    let mut joints: Vec<crate::joint::Joint> = Vec::new();
    let mut materials: HashMap<String, Material> = HashMap::new();
    let mut robot_name: Option<String> = None;
    let mut child_links: Vec<String> = Vec::new();

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
                        let mut link = Link::new(&name);
                        parse_link_body(&mut reader, &mut buf, &mut link)?;
                        links.push(link);
                    }
                    b"joint" => {
                        let name = required_attr(&e, b"name", "joint")?;
                        let type_str = required_attr(&e, b"type", "joint")?;
                        let kind = parse_joint_type(&type_str)?;
                        let joint = parse_joint_body(&mut reader, &mut buf, kind, name)?;
                        child_links.push(joint.child.clone());
                        joints.push(joint);
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
                    links.push(Link::new(&name));
                } else if tag == b"joint" {
                    let name = required_attr(&e, b"name", "joint")?;
                    let type_str = required_attr(&e, b"type", "joint")?;
                    let kind = parse_joint_type(&type_str)?;
                    // Self-closing <joint/> is invalid URDF (no parent/child),
                    // but parse_joint_body will error with a clear message.
                    let joint = parse_joint_body(&mut reader, &mut buf, kind, name)?;
                    child_links.push(joint.child.clone());
                    joints.push(joint);
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

    // Determine root link: the link that is NOT a child in any joint.
    let root_link = if links.is_empty() {
        "world".to_string()
    } else {
        links
            .iter()
            .find(|l| !child_links.contains(&l.name))
            .map(|l| l.name.clone())
            .unwrap_or_else(|| links[0].name.clone())
    };

    let mut robot = Robot::new(robot_name.unwrap_or_else(|| "robot".to_string()), root_link);

    for link in links {
        robot.add_link(link);
    }
    for joint in joints {
        robot.add_joint(joint);
    }
    robot.materials = materials;

    Ok(robot)
}
