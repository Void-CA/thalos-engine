//! Industrial Programs Validation Corpus for Thalos DSL (.thls).
//!
//! Evaluates language ergonomics, orientation math (Pose/Euler/Quaternion),
//! spatial clearance, multi-target routines, and industrial workflow patterns.

use thalos_lang::parse_source;
use thalos_semantic::compiler::SemanticCompiler;
use thalos_semantic::resolver::SemanticResolver;

fn compile_and_resolve(source: &str) -> Result<thalos_semantic::model::ResolvedProgram, Vec<String>> {
    let ast = parse_source(source).map_err(|errs| errs.into_iter().map(|e| format!("{:?}", e)).collect::<Vec<_>>())?;
    let sem = SemanticCompiler::compile(&ast)?;
    SemanticResolver::resolve(&sem)
}

#[test]
fn test_industrial_01_pick_and_place_with_clearance() {
    let source = "
    const PICK_CLEARANCE = [0mm, 0mm, 150mm]
    const PLACE_CLEARANCE = [0mm, 0mm, 200mm]

    target home = position([1800mm, 0mm, 600mm])
    target pick_station = position([1500mm, 300mm, 200mm])
    target place_station = position([1000mm, -500mm, 250mm])

    fn above_pick(p: Position) -> Position {
        p + PICK_CLEARANCE
    }

    fn above_place(p: Position) -> Position {
        p + PLACE_CLEARANCE
    }

    fn pick(p: Position) {
        movej(above_pick(p));
        movel(p);
        set_output(\"gripper\", true);
        wait(200ms);
        movel(above_pick(p));
    }

    fn place(p: Position) {
        movej(above_place(p));
        movel(p);
        set_output(\"gripper\", false);
        wait(200ms);
        movel(above_place(p));
    }

    fn main() {
        movej(home);
        pick(pick_station);
        place(place_station);
        movej(home);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Pick & Place with Clearance MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 12);
}

#[test]
fn test_industrial_02_multi_angle_inspection() {
    let source = "
    const INSPECTION_HEIGHT = [0mm, 0mm, 300mm]

    target part_center = position([1200mm, 0mm, 200mm])

    target view_top = pose(
        part_center + INSPECTION_HEIGHT,
        euler(0deg, 180deg, 0deg)
    )

    target view_side_a = pose(
        part_center + [200mm, 0mm, 100mm],
        euler(0deg, 135deg, 0deg)
    )

    target view_side_b = pose(
        part_center + [-200mm, 0mm, 100mm],
        euler(0deg, 225deg, 0deg)
    )

    fn capture_snapshot(view: Pose) {
        movej(view);
        wait(100ms);
        set_output(\"trigger_camera\", true);
        wait(50ms);
        set_output(\"trigger_camera\", false);
    }

    fn main() {
        capture_snapshot(view_top);
        capture_snapshot(view_side_a);
        capture_snapshot(view_side_b);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Multi-angle inspection MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 15); // 3 * (1 movej + 2 wait + 2 set_output)
}

#[test]
fn test_industrial_03_seam_welding_routine() {
    let source = "
    const APPROACH_VEC = [0mm, 0mm, 80mm]

    target seam_start = position([1400mm, -200mm, 300mm])
    target seam_end = position([1400mm, 200mm, 300mm])

    fn approach(p: Position) -> Position {
        p + APPROACH_VEC
    }

    fn execute_weld(start: Position, end: Position) {
        movej(approach(start));
        movel(start);
        set_output(\"arc_enable\", true);
        wait(100ms);
        movel(end);
        set_output(\"arc_enable\", false);
        movel(approach(end));
    }

    fn main() {
        execute_weld(seam_start, seam_end);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Seam welding routine MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 7); // movej, movel, set_output, wait, movel, set_output, movel
}

#[test]
fn test_industrial_04_3d_palletizing_layer() {
    let source = "
    const X_DELTA = [120mm, 0mm, 0mm]
    const Y_DELTA = [0mm, 120mm, 0mm]
    const LAYER_HEIGHT = [0mm, 0mm, 100mm]

    target pallet_origin = position([1000mm, -300mm, 150mm])

    fn calc_cell(origin: Position, col: Vector3) -> Position {
        origin + col
    }

    fn drop_box(target_p: Position) {
        movel(target_p + LAYER_HEIGHT);
        movel(target_p);
        set_output(\"suction_pad\", false);
        wait(150ms);
        movel(target_p + LAYER_HEIGHT);
    }

    fn main() {
        let p0 = calc_cell(pallet_origin, X_DELTA);
        let p1 = calc_cell(pallet_origin, Y_DELTA);
        drop_box(p0);
        drop_box(p1);
    }
    ";

    let resolved = compile_and_resolve(source).expect("3D Palletizing layer MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 10);
}

#[test]
fn test_industrial_05_dispensing_continuous_path() {
    let source = "
    target p_start = position([1100mm, -100mm, 250mm])
    target p_corner1 = position([1100mm, 100mm, 250mm])
    target p_corner2 = position([1300mm, 100mm, 250mm])
    target p_end = position([1300mm, -100mm, 250mm])

    fn main() {
        movej(p_start + [0mm, 0mm, 50mm]);
        movel(p_start);
        set_output(\"glue_valve\", true);
        movel(p_corner1);
        movel(p_corner2);
        movel(p_end);
        set_output(\"glue_valve\", false);
        movel(p_end + [0mm, 0mm, 50mm]);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Continuous dispensing path MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 8);
}

#[test]
fn test_industrial_06_tcp_orientation_calibration() {
    let source = "
    target calib_tip = position([1000mm, 0mm, 400mm])

    target pose_0deg = pose(calib_tip, euler(0deg, 0deg, 0deg))
    target pose_45deg = pose(calib_tip, euler(0deg, 45deg, 0deg))
    target pose_90deg = pose(calib_tip, euler(0deg, 90deg, 0deg))

    fn touch_probe(target_pose: Pose) {
        movej(target_pose);
        wait(200ms);
        set_output(\"probe_latch\", true);
        wait(50ms);
        set_output(\"probe_latch\", false);
    }

    fn main() {
        touch_probe(pose_0deg);
        touch_probe(pose_45deg);
        touch_probe(pose_90deg);
    }
    ";

    let resolved = compile_and_resolve(source).expect("TCP orientation calibration MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 15);
}

#[test]
fn test_industrial_07_conveyor_part_pick() {
    let source = "
    target conveyor_pick_point = position([1500mm, 0mm, 100mm])
    target staging_area = position([1200mm, 400mm, 300mm])

    fn main() {
        movej(staging_area);
        wait(500ms);
        movej(conveyor_pick_point + [0mm, 0mm, 100mm]);
        movel(conveyor_pick_point);
        set_output(\"conveyor_stop\", true);
        set_output(\"gripper\", true);
        wait(200ms);
        movel(conveyor_pick_point + [0mm, 0mm, 100mm]);
        set_output(\"conveyor_stop\", false);
        movej(staging_area);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Conveyor part pick MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 10);
}

#[test]
fn test_industrial_08_tool_change_sequence() {
    let source = "
    target tool_rack_gripper = position([800mm, 500mm, 400mm])
    target tool_rack_welder = position([800mm, 700mm, 400mm])

    fn drop_tool(rack: Position) {
        movej(rack + [0mm, 0mm, 100mm]);
        movel(rack);
        set_output(\"tool_lock\", false);
        wait(300ms);
        movel(rack + [0mm, 0mm, 100mm]);
    }

    fn pick_tool(rack: Position) {
        movej(rack + [0mm, 0mm, 100mm]);
        movel(rack);
        set_output(\"tool_lock\", true);
        wait(300ms);
        movel(rack + [0mm, 0mm, 100mm]);
    }

    fn main() {
        drop_tool(tool_rack_gripper);
        pick_tool(tool_rack_welder);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Tool change sequence MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 10);
}

#[test]
fn test_industrial_09_multi_pass_welding() {
    let source = "
    const ROOT_CLEARANCE = [0mm, 0mm, 50mm]
    const CAP_OFFSET = [0mm, 2mm, 5mm]

    target seam_start = pose(
        position([1200mm, -100mm, 300mm]),
        euler(0deg, 180deg, 45deg)
    )
    target seam_end = pose(
        position([1200mm, 100mm, 300mm]),
        euler(0deg, 180deg, 45deg)
    )

    fn weld_pass(start_p: Pose, end_p: Pose) {
        movej(start_p);
        set_output(\"arc_enable\", true);
        wait(50ms);
        movel(end_p);
        set_output(\"arc_enable\", false);
    }

    fn main() {
        weld_pass(seam_start, seam_end);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Multi-pass welding MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 5);
}

#[test]
fn test_industrial_10_surface_scan_grid() {
    let source = "
    const SCAN_ALTITUDE = [0mm, 0mm, 100mm]
    const STEP_X = [50mm, 0mm, 0mm]

    target p0 = position([1000mm, 0mm, 200mm])

    fn scan_point(p: Position) {
        movel(p);
        wait(20ms);
        set_output(\"laser_trigger\", true);
        wait(10ms);
        set_output(\"laser_trigger\", false);
    }

    fn main() {
        movej(p0 + SCAN_ALTITUDE);
        scan_point(p0);
        scan_point(p0 + STEP_X);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Surface scan grid MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 11);
}

#[test]
fn test_industrial_11_multi_layer_palletizing_stack() {
    let source = "
    const LAYER_STEP = [0mm, 0mm, 150mm]
    const BOX_APPROACH = [0mm, 0mm, 200mm]

    target stack_base = position([1400mm, -400mm, 100mm])

    fn layer_target(base: Position, layer: Vector3) -> Position {
        base + layer
    }

    fn stack_box(target_pos: Position) {
        movej(target_pos + BOX_APPROACH);
        movel(target_pos);
        set_output(\"gripper\", false);
        wait(100ms);
        movel(target_pos + BOX_APPROACH);
    }

    fn main() {
        let layer1 = layer_target(stack_base, LAYER_STEP);
        stack_box(stack_base);
        stack_box(layer1);
    }
    ";

    let resolved = compile_and_resolve(source).expect("Multi-layer palletizing stack MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 10);
}

#[test]
fn test_industrial_12_tcp_orientation_pose_approach() {
    let source = "
    const RETRACT_VEC = [0mm, 0mm, 120mm]

    target work_pose = pose(
        position([1500mm, 200mm, 350mm]),
        euler(0deg, 135deg, 90deg)
    )

    fn main() {
        movej(work_pose);
        set_output(\"tool_active\", true);
        wait(250ms);
        set_output(\"tool_active\", false);
    }
    ";

    let resolved = compile_and_resolve(source).expect("TCP orientation pose approach MUST compile and resolve");
    assert_eq!(resolved.statements.len(), 4);
}
