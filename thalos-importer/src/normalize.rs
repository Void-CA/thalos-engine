use std::collections::HashSet;
use thalos_math::{Transform3D, UnitQuaternion, UnitVector3, Vector3};
use thalos_models::{Joint, JointKind, JointLimits, Link, Robot};

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

        // 4. Add Joints
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
