//! End-to-end read-only accessors: `PTT.x`, `JTT.j2`.

use thalos_lang::parse_source;
use thalos_language_service::compiler::SemanticCompiler;

fn compile_ok(source: &str) {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect("must compile");
}

fn compile_err(source: &str) -> Vec<String> {
    let ast = parse_source(source).expect("source must parse");
    SemanticCompiler::compile(&ast).expect_err("must fail")
}

#[test]
fn accessor_on_target_folds_at_compile_time() {
    let source = r#"
target PTT = position([1mm, 2mm, 3mm])
target JTT = joints(10deg, 20deg, 30deg)

fn main() {
    let x = PTT.x
    let j = JTT.j2
    movel(PTT + [PTT.x, 0mm, 0mm])
}
"#;
    compile_ok(source);
}

#[test]
fn accessor_on_local_resolves_through_the_resolver() {
    let source = r#"
fn main() {
    let p = position([1mm, 2mm, 3mm])
    let x = p.x
}
"#;
    compile_ok(source);
}

#[test]
fn unknown_member_is_a_compile_error() {
    let source = "target PTT = position([1mm, 2mm, 3mm])\nfn main() { let bad = PTT.j2 }";
    let errors = compile_err(source);
    assert!(
        errors.iter().any(|e| e.contains("Unknown member 'j2'")),
        "unexpected errors: {errors:?}"
    );
}

#[test]
fn namespace_style_access_is_not_a_value_member() {
    let source = "fn main() { let t = module.channel }";
    let errors = compile_err(source);
    assert!(
        errors.iter().any(|e| e.contains("Unknown identifier 'module'")),
        "unexpected errors: {errors:?}"
    );
}
