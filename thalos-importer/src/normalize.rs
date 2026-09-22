use std::collections::{HashMap, HashSet};
use thalos_math::{Transform3D, UnitQuaternion, UnitVector3, Vector3};
use thalos_models::{Color, Joint, JointKind, JointLimits, Link, Robot};

use crate::candidate::ImportedCandidate;
use crate::diagnostic::{DiagnosticCode, ImportDiagnostic};
use crate::error::ImportError;

/// Output of normalizing an [`ImportedCandidate`] into a canonical [`Robot`].
#[derive(Debug, Clone)]
pub struct NormalizedRobotResult {
    pub robot: Robot,
    pub diagnostics: Vec<ImportDiagnostic>,
}

/// Normalizer contract that resolves candidate assertions into domain [`Robot`] models.
pub trait Normalizer {
    fn normalize(&self, candidate: &ImportedCandidate) -> Result<NormalizedRobotResult, ImportError>;
}

/// Default normalizer implementation for [`ImportedCandidate`].
#[derive(Debug, Default, Clone)]
pub struct CandidateNormalizer;

impl CandidateNormalizer {
    pub fn new() -> Self {
        Self
    }
}

impl Normalizer for CandidateNormalizer {
    fn normalize(&self, candidate: &ImportedCandidate) -> Result<NormalizedRobotResult, ImportError> {
        let mut diagnostics = Vec::new();

        // 1. Find root link (a body that is not a child in any joint)
        let child_links: HashSet<&str> = candidate
            .raw_joints
            .iter()
            .map(|j| j.child.as_str())
            .collect();

        let root_link_name = candidate
            .raw_bodies
            .iter()
            .map(|b| b.name.as_str())
            .find(|name| !child_links.contains(name))
            .unwrap_or_else(|| {
                diagnostics.push(ImportDiagnostic::warning(
                    DiagnosticCode::UnresolvedParentLink,
                    "Could not unambiguously determine root link, selecting first body",
                ));
                candidate
                    .raw_bodies
                    .first()
                    .map(|b| b.name.as_str())
                    .unwrap_or("world")
            });

        let mut robot = Robot::new(&candidate.name, root_link_name);

        // 2. Add Links
        for raw_body in &candidate.raw_bodies {
            let mut link = Link::new(&raw_body.name);
            link.inertial = raw_body.inertial.clone();
            link.visual = raw_body.visual.clone();
            link.collision = raw_body.collision.clone();
            robot.add_link(link);
        }

        // 3. Add Materials
        robot.materials = candidate.materials.clone();

        // 4. Resolve named material references on visuals.
        resolve_visual_materials(&mut robot);

        // 5. Add Joints
        for raw_joint in &candidate.raw_joints {
            let origin_translation = raw_joint
                .origin_xyz
                .map(|[x, y, z]| Vector3::new(x, y, z))
                .unwrap_or(Vector3::zero());

            let origin_rotation = raw_joint
                .origin_rpy
                .map(|[r, p, y]| {
                    UnitQuaternion::from_euler_angles(r, p, y)
                })
                .unwrap_or(UnitQuaternion::identity());

            let origin = Transform3D::from_translation_rotation(origin_translation, origin_rotation);

            let kind = match raw_joint.joint_type.to_lowercase().as_str() {
                "revolute" => JointKind::Revolute,
                "continuous" => JointKind::Continuous,
                "prismatic" => JointKind::Prismatic,
                "fixed" => JointKind::Fixed,
                other => {
                    return Err(ImportError::Urdf(format!(
                        "Unsupported joint type '{other}' — Thalos supports revolute, continuous, prismatic, and fixed"
                    )));
                }
            };

            let axis = raw_joint.axis.map(|[x, y, z]| {
                let vec = Vector3::new(x, y, z);
                UnitVector3::new_normalize(vec)
            });

            let limits = match (raw_joint.lower_limit, raw_joint.upper_limit) {
                (Some(min), Some(max)) => Some(JointLimits::new(min, max)),
                _ => {
                    if kind == JointKind::Revolute {
                        diagnostics.push(ImportDiagnostic::warning(
                            DiagnosticCode::MissingJointLimit,
                            format!("Revolute joint '{}' missing limits", raw_joint.name),
                        ));
                    }
                    None
                }
            };

            let mut joint = Joint::new(&raw_joint.name, kind, &raw_joint.parent, &raw_joint.child, origin);
            joint.axis = axis;
            joint.limits = limits;

            robot.add_joint(joint);
        }

        Ok(NormalizedRobotResult { robot, diagnostics })
    }
}

/// Resolve named `<material name="..."/>` references on a link's visuals.
///
/// URDF allows a visual to reference a material by name only. The definition
/// usually lives in the robot-level table, but some exporters (the ABB IRB140
/// xacro output) define the color inline on the first link and leave the rest
/// as bare references — technically invalid, yet widespread. Without
/// resolution the visual keeps `color: None` and the viewport falls back to a
/// default grey.
///
/// Resolution order: robot-level definitions are authoritative, then the first
/// inline visual definition seen wins. A visual that already carries its own
/// color is never overwritten.
fn resolve_visual_materials(robot: &mut Robot) {
    let mut palette: HashMap<String, Color> = HashMap::new();

    for material in robot.materials.values() {
        if let Some(color) = material.color {
            palette.insert(material.name.clone(), color);
        }
    }

    for link in robot.links.values() {
        for visual in &link.visual {
            if let Some(material) = &visual.material {
                if !material.name.is_empty()
                    && let Some(color) = material.color
                {
                    palette.entry(material.name.clone()).or_insert(color);
                }
            }
        }
    }

    for link in robot.links.values_mut() {
        for visual in &mut link.visual {
            let Some(material) = visual.material.as_mut() else {
                continue;
            };
            if material.color.is_some() || material.name.is_empty() {
                continue;
            }
            if let Some(color) = palette.get(&material.name) {
                material.color = Some(*color);
            }
        }
    }
}
