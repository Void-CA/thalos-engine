use std::collections::HashMap;

use thalos_models::geometry::{Collision, Visual};
use thalos_models::link::Inertial;
use thalos_models::Material;

/// Raw imported candidate assertions prior to domain normalization.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ImportedCandidate {
    pub name: String,
    pub raw_bodies: Vec<CandidateBody>,
    pub raw_joints: Vec<CandidateJoint>,
    pub materials: HashMap<String, Material>,
    pub metadata: HashMap<String, String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CandidateBody {
    pub name: String,
    pub parent_hint: Option<String>,
    pub inertial: Option<Inertial>,
    pub visual: Vec<Visual>,
    pub collision: Vec<Collision>,
    pub visual_sources: Vec<String>,
    pub collision_sources: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CandidateJoint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub joint_type: String,
    pub axis: Option<[f64; 3]>,
    pub origin_xyz: Option<[f64; 3]>,
    pub origin_rpy: Option<[f64; 3]>,
    pub lower_limit: Option<f64>,
    pub upper_limit: Option<f64>,
}
