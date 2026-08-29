//! Shared golden corpus parity — drift guard between the Rust parser
//! (`script::parse`) and the TS mirror
//! (`web/src/features/semantic/script/parser.ts`, design P6).
//!
//! Both suites iterate the SAME `test-fixtures/script-golden.json` array:
//! the TS side imports it in `golden.test.ts`, this side embeds it with
//! `include_str!`. A grammar change must land in BOTH parsers or one of the
//! two suites goes red.
//!
//! The corpus has 22 cases: 17 positive (expected_ops) + 5 negative
//! (expected_ops: null, expected_errors with {line, contains}). The negative
//! "parse_multi_error_accumulation" case subsumes the standalone
//! `parse_pick_empty_errors` inline test (line 2 of that input IS `pick`).
//!
//! Origins are intentionally NOT part of the corpus (design P6 format): they
//! are derived `<command>-{line}` metadata asserted separately by each
//! parser's unit tests. Op fields (type/object/destination/tool/duration_ms)
//! are the parity contract.

use serde_json::Value;
use thalos_semantic::operation::SemanticOperation;
use thalos_semantic::resource::ToolId;
use thalos_semantic::script;

const CORPUS: &str = include_str!("../../../../test-fixtures/script-golden.json");

#[test]
fn golden_corpus_parity() {
    let corpus: Vec<Value> =
        serde_json::from_str(CORPUS).expect("test-fixtures/script-golden.json must be valid JSON");
    assert!(!corpus.is_empty(), "golden corpus must not be empty");

    for entry in &corpus {
        let name = entry["name"].as_str().expect("case has a name");
        let input = entry["input"].as_str().expect("case has input");

        match script::parse(input) {
            Ok(program) => {
                let ir = thalos_semantic::ir::normalize(&program).expect("normalize parsed program");
                // Positive case: must not expect errors, ops must match.
                let expected_errors = entry["expected_errors"]
                    .as_array()
                    .unwrap_or_else(|| panic!("{name}: expected_errors must be an array"));
                assert!(
                    expected_errors.is_empty(),
                    "{name}: positive case must not expect errors"
                );

                let expected_ops = entry["expected_ops"]
                    .as_array()
                    .unwrap_or_else(|| panic!("{name}: expected_ops must be an array"));
                assert_eq!(
                    ir.operations.len(),
                    expected_ops.len(),
                    "{name}: operation count"
                );
                for (op, expected) in ir.operations.iter().zip(expected_ops) {
                    assert_op_matches(op, expected, name);
                }
            }
            Err(errors) => {
                // Negative case: no ops, accumulated errors at the same lines.
                assert!(
                    entry["expected_ops"].is_null(),
                    "{name}: negative case must have expected_ops: null"
                );
                let expected_errors = entry["expected_errors"]
                    .as_array()
                    .unwrap_or_else(|| panic!("{name}: expected_errors must be an array"));
                assert_eq!(errors.len(), expected_errors.len(), "{name}: error count");
                for (error, expected) in errors.iter().zip(expected_errors) {
                    let line = expected["line"]
                        .as_u64()
                        .unwrap_or_else(|| panic!("{name}: expected error line"))
                        as usize;
                    let contains = expected["contains"]
                        .as_str()
                        .unwrap_or_else(|| panic!("{name}: expected error contains"));
                    assert_eq!(error.line, line, "{name}: error line");
                    assert!(
                        error.message.contains(contains),
                        "{name}: message '{}' should contain '{contains}'",
                        error.message
                    );
                }
            }
        }
    }
}

fn assert_op_matches(op: &SemanticOperation, expected: &Value, name: &str) {
    let expected_type = expected["type"]
        .as_str()
        .unwrap_or_else(|| panic!("{name}: expected op has a type"));

    match (op, expected_type) {
        (SemanticOperation::Pick(pick), "pick") => {
            assert_eq!(
                pick.object.0,
                expected["object"]
                    .as_str()
                    .unwrap_or_else(|| panic!("{name}: pick object")),
                "{name}: pick object"
            );
            assert_tool(&pick.tool, expected, name);
        }
        (SemanticOperation::Place(place), "place") => {
            assert_eq!(
                place.object.0,
                expected["object"]
                    .as_str()
                    .unwrap_or_else(|| panic!("{name}: place object")),
                "{name}: place object"
            );
            assert_eq!(
                place.destination.0,
                expected["destination"]
                    .as_str()
                    .unwrap_or_else(|| panic!("{name}: place destination")),
                "{name}: place destination"
            );
            assert_tool(&place.tool, expected, name);
        }
        (SemanticOperation::MoveTo(move_to), "move_to") => {
            assert_eq!(
                move_to.destination.0,
                expected["destination"]
                    .as_str()
                    .unwrap_or_else(|| panic!("{name}: move_to destination")),
                "{name}: move_to destination"
            );
            assert_tool(&move_to.tool, expected, name);
        }
        (SemanticOperation::Wait(wait), "wait") => {
            let expected_ms = expected["duration_ms"]
                .as_u64()
                .unwrap_or_else(|| panic!("{name}: wait duration_ms"))
                as u128;
            assert_eq!(
                wait.duration.as_millis(),
                expected_ms,
                "{name}: wait duration in ms"
            );
        }
        (SemanticOperation::Home(_), "home") => {}
        (other, type_name) => {
            panic!("{name}: parsed op {other:?} does not match expected type '{type_name}'")
        }
    }
}

fn assert_tool(tool: &Option<ToolId>, expected: &Value, name: &str) {
    match (tool.as_ref(), expected["tool"].as_str()) {
        (Some(actual), Some(wanted)) => {
            assert_eq!(actual.0, wanted, "{name}: tool");
        }
        (None, None) => {}
        (Some(actual), None) => panic!("{name}: unexpected tool '{}'", actual.0),
        (None, Some(wanted)) => panic!("{name}: expected tool '{wanted}', got none"),
    }
}
