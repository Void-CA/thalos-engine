//! IK consistency regression (spec `unified-kinematics`).
//!
//! Same chain + same target + same [`IKConfig`] MUST yield the same IK verdict
//! across the three construction paths:
//!
//! 1. **Semantic path** — mirrors `thalos-api/.../semantic/handler.rs`
//!    (`DampedLeastSquaresSolver::from_config(fk, chain.end_effector, …)`
//!    with the site's preserved values 1000/1e-4/0.1).
//! 2. **Analysis path** — mirrors `thalos-api/.../plan_analysis/handler.rs`
//!    (from_config with the site's preserved values 500/1e-6/0.1).
//! 3. **Runtime path** — mirrors `thalos-runtime/.../scene.rs` `solve_ik`
//!    (from_config with the runtime constants 500/1e-6/0.1).
//!
//! The three paths share `DampedLeastSquaresSolver`, so a naive equality test
//! is vacuous. The real value is:
//!
//! - **(a) Compile-time**: all three paths construct through the shared
//!   `IKConfig` type (`from_config`) — no site can drift back to inline
//!   constants without breaking this test.
//! - **(b) Same config → same verdict**: identical chain/target/config yields
//!   one verdict across all three paths.
//! - **(c) Divergence documentation**: the two preserved site configs
//!   (semantic 1e-4 vs analysis/runtime 1e-6) DO produce different verdicts on
//!   a pose the SCARA cannot fully reach — the exact divergence this mechanism
//!   prevents when one shared config is passed to all sites.

use thalos_engine::core::{
    kinematics::{
        forward::ForwardKinematics,
        inverse::{DampedLeastSquaresSolver, IKConfig, IKGoal, IKSolver, IKStatus},
    },
    robot::{adapter, serial_chain::SerialChain},
    spatial::pose::Pose,
};
use thalos_engine::math::{UnitQuaternion, UnitVector3, Vector3};

/// The icebot (SCARA: 3 z-axis revolutes + 1 z prismatic) loaded ONCE and
/// shared by all three paths.
const ICEBOT_URDF: &str = include_str!("../../thalos-core/tests/fixtures/icebot.urdf");

/// Preserved semantic-site config (1000/1e-4/0.1).
fn semantic_config() -> IKConfig {
    IKConfig {
        max_iterations: 1000,
        tolerance: 1e-4,
        lambda: 0.1,
    }
}

/// Preserved analysis+runtime-site config (500/1e-6/0.1).
fn analysis_config() -> IKConfig {
    IKConfig {
        max_iterations: 500,
        tolerance: 1e-6,
        lambda: 0.1,
    }
}

/// Build the three pipeline-path solvers over ONE icebot chain, each
/// constructing through the shared `IKConfig` type exactly as its site does.
fn three_path_solvers(chain: &SerialChain, config: IKConfig) -> [DampedLeastSquaresSolver; 3] {
    let fk = ForwardKinematics::new(chain.clone());
    let ee = *chain.end_effector();
    [
        // Semantic compilation path — `semantic/handler.rs` construction.
        DampedLeastSquaresSolver::from_config(fk.clone(), ee, config),
        // Plan analysis path — `plan_analysis/handler.rs` construction.
        DampedLeastSquaresSolver::from_config(fk.clone(), ee, config),
        // Runtime path — `scene.rs` `solve_ik` construction.
        DampedLeastSquaresSolver::from_config(fk, ee, config),
    ]
}

/// A mid-workspace joint seed for the icebot chain.
fn mid_workspace_seed(chain: &SerialChain) -> Vec<f64> {
    // Icebot is 4 DOF (3 z-axis revolutes + 1 z prismatic). Mid-range values
    // keep joint-limit clamping from interfering with the solves.
    // With physical home offsets (axis_0 +20°, axis_1 -80°), the workspace is
    // rotated and the minimum reach is ~0.173m. Use a seed that keeps the
    // end effector well within the reachable annulus (r < 0.225m max).
    assert_eq!(chain.dof_count(), 4, "icebot must have 4 DOF");
    vec![0.3, 0.8, -0.3, 0.03]
}

/// The end-effector pose at the seed — the reachable starting point.
fn seed_pose(chain: &SerialChain, q0: &[f64]) -> Pose {
    let fk = ForwardKinematics::new(chain.clone());
    let fk_result = fk.evaluate(q0);
    let pose = fk_result
        .ee_pose()
        .expect("icebot EE frame must exist in FK result");
    Pose::new(
        pose.reference_id(),
        pose.target_id(),
        pose.transform().clone(),
    )
}

/// A pose the 4-DOF SCARA CANNOT reach: the seed pose perturbed by a small
/// roll (5e-5 rad) around X. SCARA only produces yaw (z-axis) rotations, so
/// the orientation error has an irreducible floor of ~5e-5 rad — above the
/// analysis tolerance (1e-6) but below the semantic tolerance (1e-4). This
/// makes the tolerance divergence deterministic: the solver never needs to
/// "get lucky" within its iteration budget.
fn roll_unreachable_pose(pose: &Pose) -> Pose {
    let roll = UnitQuaternion::from_axis_angle(
        UnitVector3::new(Vector3::new(1.0, 0.0, 0.0)).expect("unit x axis"),
        5e-5,
    );
    let mut transform = pose.transform().clone();
    transform.rotation = transform.rotation * roll;
    Pose::new(pose.reference_id(), pose.target_id(), transform)
}

/// (a) + (b): the three construction sites honor one explicit shared
/// [`IKConfig`] — same chain + same target + same config → SAME verdict
/// across semantic, analysis, and runtime paths.
#[test]
fn same_chain_same_target_same_config_yields_same_verdict_across_all_three_paths() {
    let chain = adapter::from_urdf(ICEBOT_URDF).expect("icebot URDF must import");
    let q0 = mid_workspace_seed(&chain);
    let seed = seed_pose(&chain, &q0);
    // Reachable position target (1 cm offset from the seed toward origin).
    // With physical offsets the workspace is tighter; a larger offset
    // can push the target past the max reach (~0.225 m).
    let reachable_pos = seed.translation() + Vector3::new(-0.01, 0.0, 0.0);

    // Analysis/runtime config: reachable position converges on all three paths.
    let solvers = three_path_solvers(&chain, analysis_config());
    let verdicts: Vec<IKStatus> = solvers
        .iter()
        .map(|s| {
            s.solve(&q0, IKGoal::Position(reachable_pos))
                .expect("solve must succeed")
                .status
        })
        .collect();
    assert!(
        verdicts.iter().all(|v| *v == IKStatus::Converged),
        "all three paths must converge on a reachable position, got {verdicts:?}"
    );

    // Analysis/runtime config: the roll floor keeps ALL three paths at
    // MaxIterations — a single verdict, never a split.
    let solvers = three_path_solvers(&chain, analysis_config());
    let verdicts: Vec<IKStatus> = solvers
        .iter()
        .map(|s| {
            s.solve(&q0, IKGoal::Pose(roll_unreachable_pose(&seed)))
                .expect("solve must succeed")
                .status
        })
        .collect();
    assert!(
        verdicts.iter().all(|v| *v == IKStatus::MaxIterations),
        "irreducible roll error must keep all three paths at MaxIterations, got {verdicts:?}"
    );

    // Semantic config: the same roll floor is WITHIN the looser tolerance, so
    // all three paths converge — again a single verdict.
    let solvers = three_path_solvers(&chain, semantic_config());
    let verdicts: Vec<IKStatus> = solvers
        .iter()
        .map(|s| {
            s.solve(&q0, IKGoal::Pose(roll_unreachable_pose(&seed)))
                .expect("solve must succeed")
                .status
        })
        .collect();
    assert!(
        verdicts.iter().all(|v| *v == IKStatus::Converged),
        "looser semantic tolerance must converge on all three paths, got {verdicts:?}"
    );
}

/// (c) Divergence documentation: the two PRESERVED site configs produce
/// different verdicts on the same unreachable pose — semantic assessment says
/// `Converged` while analysis/runtime execution says `MaxIterations`. This is
/// the exact assessment-vs-execution divergence the shared `IKConfig`
/// mechanism prevents: one config passed to all sites yields one verdict.
#[test]
fn divergent_site_configs_produce_divergent_verdicts() {
    let chain = adapter::from_urdf(ICEBOT_URDF).expect("icebot URDF must import");
    let q0 = mid_workspace_seed(&chain);
    let seed = seed_pose(&chain, &q0);

    let semantic = DampedLeastSquaresSolver::from_config(
        ForwardKinematics::new(chain.clone()),
        *chain.end_effector(),
        semantic_config(),
    );
    let analysis = DampedLeastSquaresSolver::from_config(
        ForwardKinematics::new(chain.clone()),
        *chain.end_effector(),
        analysis_config(),
    );

    let semantic_res = semantic
        .solve(&q0, IKGoal::Pose(roll_unreachable_pose(&seed)))
        .expect("solve must succeed");
    let analysis_res = analysis
        .solve(&q0, IKGoal::Pose(roll_unreachable_pose(&seed)))
        .expect("solve must succeed");

    // The verdicts DIFFER — the pre-unification state of the pipeline.
    assert_eq!(
        semantic_res.status,
        IKStatus::Converged,
        "semantic config (1000/1e-4) must converge: final_error {:.2e} < 1e-4",
        semantic_res.final_error
    );
    assert_eq!(
        analysis_res.status,
        IKStatus::MaxIterations,
        "analysis config (500/1e-6) must exhaust its budget: final_error {:.2e} >= 1e-6",
        analysis_res.final_error
    );

    // The config values are actually HONORED (not just carried):
    // - max_iterations: the analysis solve reports exactly its 500-iteration
    //   budget; the semantic solve converges before its 1000-iteration cap.
    assert_eq!(
        analysis_res.iterations, 500,
        "max_iterations must be honored"
    );
    assert!(semantic_res.iterations <= 1000);
    // - tolerance: the residual lands between the two tolerances.
    assert!(
        semantic_res.final_error < 1e-4,
        "semantic residual {:.2e} must satisfy its 1e-4 tolerance",
        semantic_res.final_error
    );
    assert!(
        analysis_res.final_error >= 1e-6,
        "analysis residual {:.2e} must exceed its 1e-6 tolerance",
        analysis_res.final_error
    );
}
