//! Corpus of real-world Thalos DSL (.thls) programs.
//!
//! Validates language ergonomics, type checking, purity constraints,
//! and semantic resolution across 15 realistic robotic programming scenarios.

use thalos_lang::parse_source;
use thalos_semantic::compiler::SemanticCompiler;
use thalos_semantic::resolver::SemanticResolver;

fn parse_and_compile(source: &str) -> Result<thalos_semantic::model::SemanticProgram, Vec<String>> {
    let ast = parse_source(source).map_err(|errs| errs.into_iter().map(|e| format!("{:?}", e)).collect::<Vec<_>>())?;
    SemanticCompiler::compile(&ast)
}

#[test]
fn test_corpus_01_pick_and_place() {
    let source = "
    const APPROACH_OFFSET = [0mm, 0mm, 150mm]

    target home = position([1800mm, 0mm, 500mm])
    target pick_pos = position([1500mm, 300mm, 350mm])
    target place_pos = position([1200mm, -400mm, 350mm])

    fn approach(p: Position) -> Position {
        p + APPROACH_OFFSET
    }

    fn pick(p: Position) {
        movel(approach(p));
        movel(p);
        set_output(\"gripper\", true);
        wait(200ms);
        movel(approach(p));
    }

    fn place(p: Position) {
        movel(approach(p));
        movel(p);
        set_output(\"gripper\", false);
        wait(150ms);
        movel(approach(p));
    }

    fn main() {
        movej(home);
        pick(pick_pos);
        place(place_pos);
        movej(home);
    }
    ";

    let sem = parse_and_compile(source).expect("Pick & Place scenario MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Pick & Place scenario MUST resolve");
    assert_eq!(resolved.statements.len(), 12); // 2 movej, 6 movel, 2 wait, 2 set_output
}

#[test]
fn test_corpus_02_spatial_vector_offsets() {
    let source = "
    const X_STEP = [100mm, 0mm, 0mm]
    const Y_STEP = [0mm, 100mm, 0mm]
    const Z_LIFT = [0mm, 0mm, 50mm]

    target base_corner = position([1000mm, 200mm, 300mm])

    fn calc_cell(base: Position) -> Position {
        base + X_STEP + Y_STEP + Z_LIFT
    }

    fn main() {
        movel(calc_cell(base_corner));
    }
    ";

    let sem = parse_and_compile(source).expect("Spatial vector offsets MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Spatial vector offsets MUST resolve");
    assert_eq!(resolved.statements.len(), 1);
}

#[test]
fn test_corpus_03_joint_space_motion() {
    let source = "
    target home = joints(0deg, -45deg, 90deg, 0deg, 45deg, 0deg)
    target maintenance = joints(0deg, 0deg, 0deg, 0deg, 0deg, 0deg)

    fn main() {
        movej(home);
        wait(1s);
        movej(maintenance);
    }
    ";

    let sem = parse_and_compile(source).expect("Joint space motion MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Joint space motion MUST resolve");
    assert_eq!(resolved.statements.len(), 3);
}

#[test]
fn test_corpus_04_cartesian_linear_path() {
    let source = "
    target p1 = position([1200mm, 0mm, 400mm])
    target p2 = position([1200mm, 200mm, 400mm])
    target p3 = position([1400mm, 200mm, 400mm])

    fn main() {
        movel(p1);
        movel(p2);
        movel(p3);
    }
    ";

    let sem = parse_and_compile(source).expect("Cartesian path MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Cartesian path MUST resolve");
    assert_eq!(resolved.statements.len(), 3);
}

#[test]
fn test_corpus_05_nested_pure_functions() {
    let source = "
    const UP = [0mm, 0mm, 100mm]
    const FORWARD = [50mm, 0mm, 0mm]

    target p0 = position([1000mm, 100mm, 200mm])

    fn lift(p: Position) -> Position {
        p + UP
    }

    fn advance_and_lift(p: Position) -> Position {
        lift(p + FORWARD)
    }

    fn main() {
        movel(advance_and_lift(p0));
    }
    ";

    let sem = parse_and_compile(source).expect("Nested pure functions MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Nested pure functions MUST resolve");
    assert_eq!(resolved.statements.len(), 1);
}

#[test]
fn test_corpus_06_io_sequencing_and_delays() {
    let source = "
    target inspect_station = position([1500mm, 100mm, 400mm])

    fn trigger_camera() {
        set_output(\"camera_trigger\", true);
        wait(50ms);
        set_output(\"camera_trigger\", false);
    }

    fn main() {
        movej(inspect_station);
        trigger_camera();
        wait(500ms);
    }
    ";

    let sem = parse_and_compile(source).expect("IO sequencing MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("IO sequencing MUST resolve");
    assert_eq!(resolved.statements.len(), 5);
}

#[test]
fn test_corpus_07_calibration_routine() {
    let source = "
    target calib_p1 = position([1000mm, -100mm, 300mm])
    target calib_p2 = position([1000mm, 100mm, 300mm])
    target calib_p3 = position([1000mm, 0mm, 400mm])

    fn record_point(p: Position) {
        movel(p);
        wait(100ms);
        set_output(\"sync_sensor\", true);
        wait(50ms);
        set_output(\"sync_sensor\", false);
    }

    fn main() {
        record_point(calib_p1);
        record_point(calib_p2);
        record_point(calib_p3);
    }
    ";

    let sem = parse_and_compile(source).expect("Calibration routine MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Calibration routine MUST resolve");
    assert_eq!(resolved.statements.len(), 15);
}

#[test]
fn test_corpus_08_reusable_approach_retract() {
    let source = "
    const CLEARANCE = [0mm, 0mm, 200mm]

    target work_target = position([1200mm, 150mm, 250mm])

    fn safe_above(p: Position) -> Position {
        p + CLEARANCE
    }

    fn execute_work(target_pos: Position) {
        movej(safe_above(target_pos));
        movel(target_pos);
        wait(300ms);
        movel(safe_above(target_pos));
    }

    fn main() {
        execute_work(work_target);
    }
    ";

    let sem = parse_and_compile(source).expect("Reusable approach/retract MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Reusable approach/retract MUST resolve");
    assert_eq!(resolved.statements.len(), 4);
}

#[test]
fn test_corpus_09_multiple_routine_invocations() {
    let source = "
    target slot1 = position([1000mm, -200mm, 100mm])
    target slot2 = position([1000mm, 0mm, 100mm])
    target slot3 = position([1000mm, 200mm, 100mm])

    fn inspect_slot(p: Position) {
        movej(p);
        set_output(\"led\", true);
        wait(100ms);
        set_output(\"led\", false);
    }

    fn main() {
        inspect_slot(slot1);
        inspect_slot(slot2);
        inspect_slot(slot3);
    }
    ";

    let sem = parse_and_compile(source).expect("Multiple routine invocations MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Multiple routine invocations MUST resolve");
    assert_eq!(resolved.statements.len(), 12);
}

#[test]
fn test_corpus_10_derived_targets_with_const() {
    let source = "
    const DROP_OFFSET = [0mm, 0mm, -80mm]

    target stage_1 = position([1400mm, 100mm, 500mm])

    fn compute_drop(p: Position) -> Position {
        p + DROP_OFFSET
    }

    fn main() {
        let final_pos = compute_drop(stage_1);
        movel(final_pos);
    }
    ";

    let sem = parse_and_compile(source).expect("Derived targets with const MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Derived targets with const MUST resolve");
    assert_eq!(resolved.statements.len(), 1);
}

#[test]
fn test_corpus_11_pure_fn_effect_rejection_movel() {
    let source = "
    fn illegal_pure(p: Position) -> Position {
        movel(p);
        p
    }
    ";

    let errs = parse_and_compile(source).expect_err("Pure function with movel MUST be rejected");
    assert!(errs.iter().any(|e| e.contains("cannot produce robotic/IO effects")));
}

#[test]
fn test_corpus_12_pure_fn_effect_rejection_set_output() {
    let source = "
    fn illegal_io(p: Position) -> Position {
        set_output(\"val\", true);
        p
    }
    ";

    let errs = parse_and_compile(source).expect_err("Pure function with set_output MUST be rejected");
    assert!(errs.iter().any(|e| e.contains("cannot produce robotic/IO effects")));
}

#[test]
fn test_corpus_13_type_mismatch_rejection() {
    let source = "
    target home = position([1000mm, 0mm, 500mm])

    fn add_coords(p: Position, val: Position) -> Position {
        p + val
    }

    fn main() {
        movej(home);
    }
    ";

    let errs = parse_and_compile(source).expect_err("Position + Position MUST be rejected by type checker");
    assert!(errs.iter().any(|e| e.contains("Invalid binary operation")));
}

#[test]
fn test_corpus_14_unbound_variable_resolution_error() {
    let source = "
    fn main() {
        movej(undefined_target);
    }
    ";

    let errs = parse_and_compile(source).expect_err("Unbound target MUST fail compilation");
    assert!(errs.iter().any(|e| e.contains("Unknown identifier")));
}

#[test]
fn test_corpus_15_full_multi_target_cycle() {
    let source = "
    const LIFT = [0mm, 0mm, 200mm]

    target t1 = position([1500mm, 0mm, 400mm])
    target t2 = position([1500mm, 500mm, 400mm])

    fn offset_up(p: Position) -> Position {
        p + LIFT
    }

    fn cycle(a: Position, b: Position) {
        movej(offset_up(a));
        movel(a);
        wait(100ms);
        movel(offset_up(a));
        movej(offset_up(b));
        movel(b);
        wait(100ms);
        movel(offset_up(b));
    }

    fn main() {
        cycle(t1, t2);
    }
    ";

    let sem = parse_and_compile(source).expect("Full multi-target cycle MUST compile");
    let resolved = SemanticResolver::resolve(&sem).expect("Full multi-target cycle MUST resolve");
    assert_eq!(resolved.statements.len(), 8);
}
