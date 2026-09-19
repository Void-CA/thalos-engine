use thalos_language_service::intelligence::analyze_intelligence;
use thalos_language_service::intelligence::{HintOrigin, SemanticTokenKind};
use thalos_language_service::types::Type;

fn slice<'a>(source: &'a str, span: &thalos_language_service::SourceSpan) -> &'a str {
    &source[span.start as usize..span.end as usize]
}

const PROGRAM: &str = r#"target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)
target ptt = position([2.152, 0.783, 1.882])

fn main() {
    movej(jtt)
    movel(ptt)

    let offset1 = [1, 0, 0]
    movel(ptt - offset1)
}
"#;

#[test]
fn infers_declaration_types_as_inlay_hints() {
    let intelligence = analyze_intelligence(PROGRAM, 7);
    assert_eq!(intelligence.revision, 7);
    assert!(
        intelligence.diagnostics.is_empty(),
        "program must be semantically valid, got {:?}",
        intelligence.diagnostics
    );

    let hint = |name: &str| {
        intelligence
            .hints
            .iter()
            .find(|h| h.name == name)
            .unwrap_or_else(|| panic!("missing hint for {name}"))
    };

    assert_eq!(hint("jtt").ty, Type::Joints { dimension: Some(6) });
    assert_eq!(hint("ptt").ty, Type::Position);
    assert_eq!(hint("offset1").ty, Type::Vector3);
    assert_eq!(slice(PROGRAM, &hint("jtt").span), "jtt");
}

#[test]
fn inlay_hints_distinguish_declared_from_inferred_types() {
    let source = r#"const D : Length = 5mm
const I = 5mm
target t = position([1mm, 2mm, 3mm])

fn f(p : Position, q, side : Length) {
    let a : Length = side
    let b = side
    movel(p)
}

fn main() {}
"#;
    let intelligence = analyze_intelligence(source, 1);
    assert!(
        intelligence.diagnostics.is_empty(),
        "program must be valid, got {:?}",
        intelligence.diagnostics
    );

    let origin = |name: &str| {
        intelligence
            .hints
            .iter()
            .find(|h| h.name == name)
            .unwrap_or_else(|| panic!("missing hint for {name}"))
            .origin
    };

    // Explicit annotations must not be repeated as inlay hints.
    assert_eq!(origin("D"), HintOrigin::Declared);
    assert_eq!(origin("p"), HintOrigin::Declared);
    assert_eq!(origin("side"), HintOrigin::Declared);
    assert_eq!(origin("a"), HintOrigin::Declared);

    // Inferred declarations keep their hint.
    assert_eq!(origin("I"), HintOrigin::Inferred);
    assert_eq!(origin("q"), HintOrigin::Inferred);
    assert_eq!(origin("b"), HintOrigin::Inferred);
    // `target` has no annotation syntax, so it is always inferred.
    assert_eq!(origin("t"), HintOrigin::Inferred);
}

#[test]
fn classifies_semantic_tokens_by_role() {
    let source = r#"target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)
target ptt = position([1mm, 2mm, 3mm])
const LIMIT : Length = 5mm

fn draw_square(start_point : Position, side : Length) {
    let p1 = start_point.offset(x = side)
    let dx = start_point.x
    movel(p1)
}

fn main() {
    movej(jtt)
    draw_square(ptt, LIMIT)
}
"#;
    let intelligence = analyze_intelligence(source, 1);

    let has = |kind: SemanticTokenKind, text: &str| {
        intelligence
            .tokens
            .iter()
            .any(|token| token.kind == kind && slice(source, &token.span) == text)
    };

    // Explicit type annotations.
    assert!(has(SemanticTokenKind::Type, "Position"));
    assert!(has(SemanticTokenKind::Type, "Length"));
    // Function declarations and calls (builtins included).
    assert!(has(SemanticTokenKind::Function, "draw_square"));
    assert!(has(SemanticTokenKind::Function, "joints"));
    assert!(has(SemanticTokenKind::Function, "position"));
    // Member call vs member access.
    assert!(has(SemanticTokenKind::Method, "offset"));
    assert!(has(SemanticTokenKind::Property, "x"));
    // Symbol references.
    assert!(has(SemanticTokenKind::Parameter, "start_point"));
    assert!(has(SemanticTokenKind::Parameter, "side"));
    assert!(has(SemanticTokenKind::Variable, "p1"));
    assert!(has(SemanticTokenKind::Target, "jtt"));
    assert!(has(SemanticTokenKind::Target, "ptt"));
    assert!(has(SemanticTokenKind::Const, "LIMIT"));
}

#[test]
fn exposes_expression_types_with_precise_spans() {
    let intelligence = analyze_intelligence(PROGRAM, 1);

    // `ptt` appears in `movel(ptt)` and in `ptt - offset1`; both must be found
    // and typed Position, with distinct spans.
    let ptt_exprs: Vec<_> = intelligence
        .expressions
        .iter()
        .filter(|e| slice(PROGRAM, &e.span) == "ptt")
        .collect();
    assert_eq!(ptt_exprs.len(), 2, "both ptt references must be collected");
    assert!(ptt_exprs.iter().all(|e| e.ty == Type::Position));

    // The whole binary expression resolves to a Position.
    let binary = intelligence
        .expressions
        .iter()
        .find(|e| slice(PROGRAM, &e.span) == "ptt - offset1")
        .expect("binary expression span");
    assert_eq!(binary.ty, Type::Position);
}

#[test]
fn reports_semantic_diagnostics_with_real_source_spans() {
    let source = r#"target ptt = position([2.152, 0.783, 1.882])

fn main() {
    movel(ptt2 - ptt)
}
"#;
    let intelligence = analyze_intelligence(source, 3);

    assert_eq!(intelligence.diagnostics.len(), 1);
    let diagnostic = &intelligence.diagnostics[0];
    assert_eq!(diagnostic.message, "Unknown identifier 'ptt2'");
    // The span is the engine-owned target expression, not a fabricated location.
    assert_eq!(slice(source, &diagnostic.span), "ptt2 - ptt");
}

#[test]
fn motion_type_error_points_at_the_target_node() {
    let source = r#"target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)

fn main() {
    movel(jtt)
}
"#;
    let intelligence = analyze_intelligence(source, 1);

    let diagnostic = intelligence
        .diagnostics
        .iter()
        .find(|d| d.message.contains("movel expected a spatial target"))
        .expect("expected a movel target diagnostic");
    // Points at `jtt`, not at the whole `movel(jtt)` statement.
    assert_eq!(slice(source, &diagnostic.span), "jtt");
}

#[test]
fn joints_constructor_is_known_inside_function_bodies() {
    let source = "fn main() {\n    movel(joints([0deg, 0deg, 0deg]))\n}";
    let intelligence = analyze_intelligence(source, 1);

    // `joints` must resolve to Joints, so movel is rejected on the spatial
    // target rule (not on an unknown-function error).
    assert!(
        intelligence
            .diagnostics
            .iter()
            .any(|d| d.message.contains("movel expected a spatial target")),
        "got {:?}",
        intelligence.diagnostics
    );
}

#[test]
fn exposes_builtin_and_overloaded_signatures() {
    let intelligence = analyze_intelligence("fn main() {}", 1);

    let position = intelligence
        .signatures
        .iter()
        .find(|s| s.name == "position")
        .expect("position signature");
    assert_eq!(position.overloads.len(), 1);
    assert_eq!(position.overloads[0].params, vec![Type::Vector3]);
    assert_eq!(*position.overloads[0].return_type, Type::Position);

    let movej = intelligence
        .signatures
        .iter()
        .find(|s| s.name == "movej")
        .expect("movej signature");
    assert_eq!(movej.overloads.len(), 3, "movej is overloaded");
}

#[test]
fn parse_errors_are_reported_and_snapshot_is_empty() {
    let source = "target INVALID = ;";
    let intelligence = analyze_intelligence(source, 4);

    assert_eq!(intelligence.revision, 4);
    assert!(intelligence.symbols.is_empty());
    assert!(intelligence.hints.is_empty());
    assert_eq!(intelligence.diagnostics.len(), 1);
    assert_eq!(
        intelligence.diagnostics[0].code.as_deref(),
        Some("THL_PARSER_ERROR")
    );
}
