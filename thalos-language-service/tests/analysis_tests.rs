use thalos_language_service::{
    analyze_document, char_range_to_byte_span, DiagnosticSeverity, SymbolKind, ProvenanceKind,
};

#[test]
fn test_analyze_valid_thalos_document() {
    let source = r#"
const CLEARANCE = 150mm;
target PICK = [320mm, 140mm, 80mm];

fn main() {
    movej(PICK);
    wait(200ms);
}
"#;

    let analysis = analyze_document(source, 10);
    assert_eq!(analysis.revision, 10);
    assert!(analysis.diagnostics.is_empty());

    // Symbols check
    let symbol_names: Vec<&str> = analysis.symbols.iter().map(|s| s.name.as_str()).collect();
    assert!(symbol_names.contains(&"CLEARANCE"));
    assert!(symbol_names.contains(&"PICK"));
    assert!(symbol_names.contains(&"main"));

    let pick_sym = analysis.symbols.iter().find(|s| s.name == "PICK").unwrap();
    assert_eq!(pick_sym.kind, SymbolKind::Target);
    assert!(pick_sym.span.end > pick_sym.span.start);

    // Provenance check
    assert!(!analysis.provenance.is_empty());
    assert!(analysis.provenance.iter().any(|p| p.kind == ProvenanceKind::Instruction));
}

#[test]
fn test_analyze_invalid_syntax_returns_diagnostic() {
    let source = r#"
target INVALID = ;
"#;

    let analysis = analyze_document(source, 42);
    assert_eq!(analysis.revision, 42);
    assert!(!analysis.diagnostics.is_empty());

    let diag = &analysis.diagnostics[0];
    assert_eq!(diag.severity, DiagnosticSeverity::Error);
    assert_eq!(diag.code, Some("THL_PARSER_ERROR".to_string()));
    assert!(diag.span.end >= diag.span.start);
}

#[test]
fn test_unicode_utf8_byte_offset_conversion() {
    let source = "const POS = 100mm; target PICK = [100mm, 200mm, 300mm];";
    
    let target_byte_offset = source.find("target").unwrap();
    let target_char_idx = source[..target_byte_offset].chars().count();
    
    let byte_span = char_range_to_byte_span(source, target_char_idx..target_char_idx + 6);
    let slice_by_bytes = &source[byte_span.start as usize..byte_span.end as usize];
    assert_eq!(slice_by_bytes, "target");
    assert_eq!(byte_span.start as usize, target_byte_offset);

    let analysis = analyze_document(source, 100);
    assert_eq!(analysis.revision, 100);
    assert!(analysis.diagnostics.is_empty());
    let pick_sym = analysis.symbols.iter().find(|s| s.name == "PICK").unwrap();
    let pick_slice = &source[pick_sym.span.start as usize..pick_sym.span.end as usize];
    assert_eq!(pick_slice, "PICK");
    assert_eq!(pick_sym.span.start as usize, source.find("PICK").unwrap());
}

#[test]
fn test_pure_language_program_without_scene() {
    let source = "const SPEED = 100mm;";
    let analysis = analyze_document(source, 1);
    assert_eq!(analysis.revision, 1);
    assert!(analysis.diagnostics.is_empty());
    assert_eq!(analysis.symbols.len(), 1);
    assert_eq!(analysis.symbols[0].name, "SPEED");
    assert_eq!(analysis.symbols[0].kind, SymbolKind::Const);
}

