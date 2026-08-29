use std::fs;
use thalos_semantic::{
    knowledge::MockKnowledgeProvider,
    lowering::{LoweringContext, SemanticLowering},
    profile::RobotProfileLoader,
    script::parse,
};

#[test]
fn e2e_persisted_profile_loading_and_multi_robot_compilation() {
    let temp_dir = std::env::temp_dir().join("thalos_profile_e2e");
    let profile_01_dir = temp_dir.join("scara-01");
    let profile_02_dir = temp_dir.join("scara-02");

    fs::create_dir_all(profile_01_dir.join("skills")).unwrap();
    fs::create_dir_all(profile_02_dir.join("skills")).unwrap();

    // 1. Write SCARA-01 config and skill script to disk
    let scara_01_toml = r#"
        id = "scara_01"
        name = "SCARA Robot 01"
        capabilities = ["pick"]

        [[skills]]
        skill = "pick"
        source_type = "script"
        path = "skills/pick.thalos"
    "#;
    let scara_01_pick_script = r#"
        wait(100ms)
    "#;

    fs::write(profile_01_dir.join("robot.toml"), scara_01_toml).unwrap();
    fs::write(profile_01_dir.join("skills/pick.thalos"), scara_01_pick_script).unwrap();

    // 2. Write SCARA-02 config and skill script to disk
    let scara_02_toml = r#"
        id = "scara_02"
        name = "SCARA Robot 02"
        capabilities = ["pick"]

        [[skills]]
        skill = "pick"
        source_type = "script"
        path = "skills/pick.thalos"
    "#;
    let scara_02_pick_script = r#"
        wait(500ms)
    "#;

    fs::write(profile_02_dir.join("robot.toml"), scara_02_toml).unwrap();
    fs::write(profile_02_dir.join("skills/pick.thalos"), scara_02_pick_script).unwrap();

    // 3. Load RobotProfile and materialize SkillRegistry from disk
    let profile_a = RobotProfileLoader::load_from_dir(&profile_01_dir).expect("load scara-01");
    let registry_a = RobotProfileLoader::materialize_skills(&profile_a, &profile_01_dir).expect("materialize scara-01");

    let profile_b = RobotProfileLoader::load_from_dir(&profile_02_dir).expect("load scara-02");
    let registry_b = RobotProfileLoader::materialize_skills(&profile_b, &profile_02_dir).expect("materialize scara-02");

    // 4. Parse single source DSL program
    let dsl = r#"
        program pick_cell(robot = scara) {
            target approach = cartesian(x = 100mm, y = 50mm, z = 10mm)
            pick(part_01)
        }
    "#;
    let program = parse(dsl).expect("parse program");
    let ir = thalos_semantic::ir::normalize(&program).expect("normalize IR");

    let provider = MockKnowledgeProvider::new();

    // 5. Lower for SCARA-01
    let ctx_a = LoweringContext::new(&provider)
        .with_robot(&profile_a.definition)
        .with_skills(&registry_a);
    let plan_a = SemanticLowering::lower(&ir, &ctx_a).expect("lower SCARA-01");

    // 6. Lower for SCARA-02
    let ctx_b = LoweringContext::new(&provider)
        .with_robot(&profile_b.definition)
        .with_skills(&registry_b);
    let plan_b = SemanticLowering::lower(&ir, &ctx_b).expect("lower SCARA-02");

    // 7. Verify architectural invariants across disk lifecycle
    assert_eq!(profile_a.id.as_str(), "scara_01");
    assert_eq!(profile_b.id.as_str(), "scara_02");

    // Both execution programs originated from identical source, but differ according to persisted skills!
    assert_ne!(plan_a.instructions, plan_b.instructions);

    // Clean up temp dir
    fs::remove_dir_all(temp_dir).ok();
}
