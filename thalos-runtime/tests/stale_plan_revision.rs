//! E06 — Runtime invariant: Stale plan revision & fingerprint rejection.
//!
//! Asserts that plans carrying program provenance (program_id, program_revision,
//! source_fingerprint) are correctly checked against the active program state,
//! refusing execution when the program revision or fingerprint has diverged.

use sha2::{Digest, Sha256};
use thalos_engine::core::execution::plan::ExecutionPlan;
use thalos_runtime::error::RuntimeError;

fn compute_fingerprint(source: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(source.as_bytes());
    format!("{:x}", hasher.finalize())
}

#[test]
fn e06_stale_plan_revision_and_fingerprint_invariants() {
    let source_v1 = "movej(joints(0deg, 0deg, 0deg, 0deg))";
    let source_v2 = "movej(joints(10deg, 0deg, 0deg, 0deg))";

    let fp_v1 = compute_fingerprint(source_v1);
    let fp_v2 = compute_fingerprint(source_v2);

    let plan = ExecutionPlan {
        waypoints: vec![],
        segments: vec![],
        duration: 1.0,
        repeat_count: 1,
        program_id: Some("prog-main".to_string()),
        program_revision: Some(1),
        source_fingerprint: Some(fp_v1.clone()),
        robot_id: Some("planar_2r".to_string()),
    };

    // 1. Fresh plan matching revision 1 and fp_v1 is valid (not stale)
    assert!(
        !plan.is_stale_for(1, &fp_v1),
        "fresh plan matching revision and fingerprint must NOT be stale"
    );

    // 2. Incrementing program revision to 2 renders plan stale
    assert!(
        plan.is_stale_for(2, &fp_v1),
        "plan with revision 1 must be STALE for program revision 2"
    );

    // 3. Changing source code to v2 renders plan stale
    assert!(
        plan.is_stale_for(1, &fp_v2),
        "plan with fingerprint v1 must be STALE for source fingerprint v2"
    );

    // 4. Validate runtime error mapping
    let err_rev: RuntimeError = RuntimeError::StalePlanRevision {
        expected: 2,
        actual: 1,
    };
    assert_eq!(err_rev.error_code(), "stale_plan_revision");

    let err_fp: RuntimeError = RuntimeError::StalePlanFingerprint {
        expected: fp_v2,
        actual: fp_v1,
    };
    assert_eq!(err_fp.error_code(), "stale_plan_fingerprint");
}
