use serde::{Deserialize, Serialize};
use thalos_lang::ast::{Item, Program, Statement};
use thalos_lang::parser::parse_source;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SourceSpan {
    /// Starting UTF-8 byte offset in source string (0-indexed, inclusive)
    pub start: u32,
    /// Ending UTF-8 byte offset in source string (0-indexed, exclusive)
    pub end: u32,
}

impl SourceSpan {
    pub fn new(start: u32, end: u32) -> Self {
        Self { start, end }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DiagnosticSeverity {
    Error,
    Warning,
    Info,
    Hint,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Diagnostic {
    pub severity: DiagnosticSeverity,
    pub code: Option<String>,
    pub message: String,
    pub span: SourceSpan,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SymbolKind {
    Target,
    Const,
    Function,
    Variable,
    Parameter,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Symbol {
    pub id: u64,
    pub name: String,
    pub kind: SymbolKind,
    pub span: SourceSpan,
    pub selection_span: SourceSpan,
    pub detail: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProvenanceKind {
    Declaration,
    Instruction,
    Expression,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProvenanceEntry {
    pub id: u64,
    pub span: SourceSpan,
    pub kind: ProvenanceKind,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DocumentAnalysis {
    pub revision: u64,
    pub diagnostics: Vec<Diagnostic>,
    pub symbols: Vec<Symbol>,
    pub provenance: Vec<ProvenanceEntry>,
}

pub fn analyze_document(source: &str, revision: u64) -> DocumentAnalysis {
    let mut diagnostics = Vec::new();
    let mut symbols = Vec::new();
    let mut provenance = Vec::new();

    match parse_source(source) {
        Ok(program) => {
            extract_analysis_from_program(source, &program, &mut symbols, &mut provenance);
        }
        Err(parse_errors) => {
            for err in parse_errors {
                let char_span = err.span();
                let byte_span = char_range_to_byte_span(source, char_span);
                diagnostics.push(Diagnostic {
                    severity: DiagnosticSeverity::Error,
                    code: Some("THL_PARSER_ERROR".to_string()),
                    message: format!("{}", err),
                    span: byte_span,
                });
            }
        }
    }

    DocumentAnalysis {
        revision,
        diagnostics,
        symbols,
        provenance,
    }
}

pub fn char_range_to_byte_span(source: &str, char_range: std::ops::Range<usize>) -> SourceSpan {
    let mut byte_start = 0;
    let mut byte_end = source.len();

    let mut current_char_idx = 0;
    let mut char_indices = source.char_indices().peekable();

    while let Some((byte_idx, _)) = char_indices.next() {
        if current_char_idx == char_range.start {
            byte_start = byte_idx;
        }
        if current_char_idx == char_range.end {
            byte_end = byte_idx;
            break;
        }
        current_char_idx += 1;
    }

    if current_char_idx < char_range.end {
        byte_end = source.len();
    }
    if char_range.start >= current_char_idx {
        byte_start = source.len();
    }

    SourceSpan::new(byte_start as u32, byte_end as u32)
}

fn extract_analysis_from_program(
    source: &str,
    program: &Program,
    symbols: &mut Vec<Symbol>,
    provenance: &mut Vec<ProvenanceEntry>,
) {
    let mut symbol_id_counter = 1u64;
    let mut provenance_id_counter = 100u64;

    for item in &program.items {
        match item {
            Item::Target(decl) => {
                let name_span = find_identifier_span(source, &decl.name);
                let full_span = name_span;

                let id = symbol_id_counter;
                symbol_id_counter += 1;

                symbols.push(Symbol {
                    id,
                    name: decl.name.clone(),
                    kind: SymbolKind::Target,
                    span: full_span,
                    selection_span: name_span,
                    detail: Some("Target pose declaration".to_string()),
                });

                let prov_id = provenance_id_counter;
                provenance_id_counter += 1;
                provenance.push(ProvenanceEntry {
                    id: prov_id,
                    span: full_span,
                    kind: ProvenanceKind::Declaration,
                });
            }
            Item::Const(decl) => {
                let name_span = find_identifier_span(source, &decl.name);
                let full_span = name_span;

                let id = symbol_id_counter;
                symbol_id_counter += 1;

                symbols.push(Symbol {
                    id,
                    name: decl.name.clone(),
                    kind: SymbolKind::Const,
                    span: full_span,
                    selection_span: name_span,
                    detail: Some("Constant value declaration".to_string()),
                });

                let prov_id = provenance_id_counter;
                provenance_id_counter += 1;
                provenance.push(ProvenanceEntry {
                    id: prov_id,
                    span: full_span,
                    kind: ProvenanceKind::Declaration,
                });
            }
            Item::Function(decl) => {
                let name_span = find_identifier_span(source, &decl.name);
                let full_span = name_span;

                let id = symbol_id_counter;
                symbol_id_counter += 1;

                symbols.push(Symbol {
                    id,
                    name: decl.name.clone(),
                    kind: SymbolKind::Function,
                    span: full_span,
                    selection_span: name_span,
                    detail: Some(format!("Function ({} params)", decl.params.len())),
                });

                let prov_id = provenance_id_counter;
                provenance_id_counter += 1;
                provenance.push(ProvenanceEntry {
                    id: prov_id,
                    span: full_span,
                    kind: ProvenanceKind::Declaration,
                });

                for stmt in &decl.body {
                    extract_statement_provenance(source, stmt, &mut provenance_id_counter, provenance);
                }
            }
            Item::Use(decl) => {
                let span = find_identifier_span(source, &decl.path);
                let prov_id = provenance_id_counter;
                provenance_id_counter += 1;
                provenance.push(ProvenanceEntry {
                    id: prov_id,
                    span,
                    kind: ProvenanceKind::Declaration,
                });
            }
        }
    }
}

fn extract_statement_provenance(
    source: &str,
    stmt: &Statement,
    provenance_id_counter: &mut u64,
    provenance: &mut Vec<ProvenanceEntry>,
) {
    let prov_id = *provenance_id_counter;
    *provenance_id_counter += 1;

    match stmt {
        Statement::MoveJ { target: _ } => {
            let span = find_keyword_span(source, "movej");
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Instruction,
            });
        }
        Statement::MoveL { target: _ } => {
            let span = find_keyword_span(source, "movel");
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Instruction,
            });
        }
        Statement::MoveC { via: _, target: _ } => {
            let span = find_keyword_span(source, "movec");
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Instruction,
            });
        }
        Statement::Wait(_) => {
            let span = find_keyword_span(source, "wait");
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Instruction,
            });
        }
        Statement::SetOutput { output, value: _ } => {
            let span = find_identifier_span(source, output);
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Instruction,
            });
        }
        Statement::Let { name, .. } => {
            let span = find_identifier_span(source, name);
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Declaration,
            });
        }
        Statement::If { then_branch, else_branch, .. } => {
            let span = find_keyword_span(source, "if");
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Instruction,
            });
            for s in then_branch {
                extract_statement_provenance(source, s, provenance_id_counter, provenance);
            }
            if let Some(else_stmts) = else_branch {
                for s in else_stmts {
                    extract_statement_provenance(source, s, provenance_id_counter, provenance);
                }
            }
        }
        Statement::Expr(_) => {
            let span = SourceSpan::new(0, 0);
            provenance.push(ProvenanceEntry {
                id: prov_id,
                span,
                kind: ProvenanceKind::Expression,
            });
        }
    }
}

fn find_identifier_span(source: &str, ident: &str) -> SourceSpan {
    if let Some(pos) = source.find(ident) {
        SourceSpan::new(pos as u32, (pos + ident.len()) as u32)
    } else {
        SourceSpan::new(0, 0)
    }
}

fn find_keyword_span(source: &str, keyword: &str) -> SourceSpan {
    if let Some(pos) = source.find(keyword) {
        SourceSpan::new(pos as u32, (pos + keyword.len()) as u32)
    } else {
        SourceSpan::new(0, 0)
    }
}
