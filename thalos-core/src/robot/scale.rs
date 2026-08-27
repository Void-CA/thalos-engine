//! Chain-side reference dimension (`L_ref`) — TWO distinct concepts.
//!
//! Decision (remediation, user): the reference dimension must separate the
//! physical/visual scale from the effective kinematic scale of the mechanism
//! that generates the Jacobian:
//!
//! - [`scene_reference_dimension`] — ALL segments (fixed included, e.g. a
//!   terminal TCP/tool joint). Physical/visual scale. Consumed by the
//!   viewport (`SceneBuilder`) and the scene wire contract.
//! - [`manipulability_reference_dimension`] — ONLY moving segments
//!   (`dof > 0`). Effective kinematic scale of the mechanism generating J.
//!   Consumed by the manipulability normalization.
//!
//! Both definitions converge across catalog and URDF sources (spec
//! reference-dimension-fix "Catalog-URDF Equivalence"): catalog robots carry
//! lengths on link transforms, URDF adapters carry them on joint origins
//! (links are identity), and both functions sum
//! `‖link.transform.translation‖ + ‖joint.origin.translation‖` — over all
//! segments for the scene, over moving segments only for manipulability.
//! `L_ref` is a canonical robot-scale normalization factor, NOT a physical
//! robot length.

use super::serial_chain::SerialChain;

/// Floor applied to a degenerate/empty chain so the canonical factor is
/// always strictly positive (`l_ref > ε` — no NaN/Inf downstream when a
/// broken URDF produces identity translations everywhere).
pub const REFERENCE_DIMENSION_EPS: f64 = 1e-9;

/// Scene reference dimension — ALL segments (fixed joints included).
///
/// `Σ_segments (‖link.transform.translation‖ + ‖joint.origin.translation‖)`
/// over the full kinematic chain. Physical/visual scale: the viewport uses it
/// to size grids, gizmos and overlays (`SceneBuilder::from_fk`). A fixed
/// terminal joint (e.g. TCP/tool) IS part of the physical robot and counts
/// here. Floored at [`REFERENCE_DIMENSION_EPS`].
pub fn scene_reference_dimension(chain: &SerialChain) -> f64 {
    sum_segment_norms(chain, |_| true)
}

/// Manipulability reference dimension — ONLY moving segments (`dof > 0`).
///
/// Same formula, restricted to the segments whose joints generate Jacobian
/// columns: `Σ_{segments with dof > 0} (‖link.transform.translation‖ +
/// ‖joint.origin.translation‖)`. This is the effective kinematic scale of
/// the mechanism that produces J. Fixed joints (bases, terminal TCP/tool)
/// contribute NO columns — including their lengths in the divisor would
/// penalize the normalized measure with "dead weight" (fixed-terminal-joint
/// independence invariant). Floored at [`REFERENCE_DIMENSION_EPS`].
pub fn manipulability_reference_dimension(chain: &SerialChain) -> f64 {
    sum_segment_norms(chain, |segment| segment.joint.dof() > 0)
}

fn sum_segment_norms(
    chain: &SerialChain,
    include: impl Fn(&crate::robot::segment::Segment) -> bool,
) -> f64 {
    let sum: f64 = chain
        .segments
        .iter()
        .filter(|segment| include(segment))
        .map(|segment| {
            segment.link.transform.translation.norm() + segment.joint.origin().translation.norm()
        })
        .sum();
    sum.max(REFERENCE_DIMENSION_EPS)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::scara::ScaraSpec;
    use crate::prelude::*;
    use thalos_math::*;

    fn identity_translation_chain() -> SerialChain {
        // A valid chain (end effector defined) whose segments carry ONLY
        // identity translations — the URDF-adapter degenerate shape.
        let mut builder = SerialChainBuilder::new();
        let base = builder.create_frame("base");
        let ee = builder.create_frame("ee");
        builder.add_segment(Segment::new(
            FrameId::World,
            base.clone(),
            JointType::Fixed(FixedJoint::new(Transform3D::identity())),
            Link::new(0, Transform3D::identity()),
        ));
        builder.add_segment(Segment::new(
            base,
            ee.clone(),
            JointType::Fixed(FixedJoint::new(Transform3D::identity())),
            Link::new(1, Transform3D::identity()),
        ));
        builder.set_end_effector(ee);
        builder.build().expect("identity chain")
    }

    #[test]
    fn scene_catalog_scara_matches_formula() {
        // Canonical SCARA: base origin (0,0,0.5) → 0.5; link1 (1,0,0) → 1.0;
        // link2 (0.8,0,0) → 0.8; remaining segments contribute 0.
        // Scene L_ref = 0.5 + 1.0 + 0.8 = 2.3 (all segments, fixed base in).
        let chain = ScaraSpec::canonical().build();
        assert!(
            (scene_reference_dimension(&chain) - 2.3).abs() < 1e-12,
            "scene SCARA L_ref must be 2.3, got {}",
            scene_reference_dimension(&chain)
        );
    }

    #[test]
    fn manipulability_scara_excludes_fixed_segments() {
        // Moving-only SCARA: the fixed base segment (origin 0.5) is EXCLUDED;
        // link1 1.0 + link2 0.8 = 1.8. The moving mechanism that generates J
        // does not include the base mount.
        let chain = ScaraSpec::canonical().build();
        assert!(
            (manipulability_reference_dimension(&chain) - 1.8).abs() < 1e-12,
            "moving SCARA L_ref must be 1.8 (fixed base excluded), got {}",
            manipulability_reference_dimension(&chain)
        );
    }

    #[test]
    fn identity_only_chain_floors_to_eps() {
        // A chain whose segments carry only identity translations (the URDF
        // adapter degenerate case) must still produce a strictly positive
        // factor — the guardrail, not a 0.01 magic floor. Both definitions.
        let chain = identity_translation_chain();
        assert_eq!(scene_reference_dimension(&chain), REFERENCE_DIMENSION_EPS);
        assert_eq!(
            manipulability_reference_dimension(&chain),
            REFERENCE_DIMENSION_EPS
        );
    }

    #[test]
    fn scene_joint_origin_translations_count_toward_l_ref() {
        // Spec: the formula sums BOTH link translations AND joint origins.
        // A segment with an identity link but a translated joint origin must
        // contribute its origin norm (this is exactly what the old visual
        // inline ref_dim missed for URDF robots).
        let mut builder = SerialChainBuilder::new();
        let base = builder.create_frame("base");
        let ee = builder.create_frame("ee");
        builder.add_segment(Segment::new(
            FrameId::World,
            base.clone(),
            JointType::Fixed(FixedJoint::new(Transform3D::from_translation(
                Vector3::new(0.0, 0.0, 0.5),
            ))),
            Link::new(0, Transform3D::identity()),
        ));
        builder.add_segment(Segment::new(
            base,
            ee.clone(),
            JointType::Revolute(RevoluteJoint::new(
                0,
                UnitVector3::z_axis(),
                JointLimits::new(-1.0, 1.0),
                Transform3D::identity(),
            )),
            Link::new(
                1,
                Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
            ),
        ));
        builder.set_end_effector(ee);
        let chain = builder.build().expect("two-segment chain");
        assert!(
            (scene_reference_dimension(&chain) - 1.5).abs() < 1e-12,
            "scene: origin 0.5 + link 1.0 = 1.5, got {}",
            scene_reference_dimension(&chain)
        );
        // Moving-only: the FIXED origin is excluded → only the moving link.
        assert!(
            (manipulability_reference_dimension(&chain) - 1.0).abs() < 1e-12,
            "moving: fixed origin excluded → link 1.0, got {}",
            manipulability_reference_dimension(&chain)
        );
    }
}
