//! Task Script — a human-readable DSL for constructing `SemanticProgram`s.
//!
//! # Format
//!
//! ```text
//! # Comment
//! pick <object> [tool=<name>]
//! place <object> at <location> [tool=<name>]
//! move_to <location> [tool=<name>]
//! wait <duration>          (e.g., 500ms, 2s, 1.5s)
//! home
//! ```
//!
//! Each line is one operation. Empty lines and comments are ignored.
//! Named arguments use `key=value` syntax.

use std::time::Duration;

use crate::operation::{HomeOp, MoveToOp, PickOp, PlaceOp, SemanticOperation, WaitOp};
use crate::program::SemanticProgram;
use crate::resource::{LocationId, ObjectId, ToolId};
use thalos_core::ids::OperationId;

/// Error returned when parsing a Task Script line fails.
#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}

/// Parse a Task Script string into a `SemanticProgram`.
///
/// Each line is parsed as one operation. Lines starting with `#` are comments.
/// Empty lines are ignored.
pub fn parse(input: &str) -> Result<SemanticProgram, Vec<ParseError>> {
    let mut operations = Vec::new();
    let mut errors = Vec::new();

    for (line_idx, line) in input.lines().enumerate() {
        let line = line.trim();
        let line_num = line_idx + 1;

        // Skip empty lines and comments
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        match parse_line(line, line_num) {
            Ok(op) => operations.push(op),
            Err(e) => errors.push(e),
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(SemanticProgram::new(operations))
}

fn parse_line(line: &str, line_num: usize) -> Result<SemanticOperation, ParseError> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.is_empty() {
        return Err(ParseError {
            line: line_num,
            message: "empty line".into(),
        });
    }

    let command = parts[0];
    let args = &parts[1..];

    match command {
        "pick" => parse_pick(args, line_num),
        "place" => parse_place(args, line_num),
        "move_to" => parse_move_to(args, line_num),
        "wait" => parse_wait(args, line_num),
        "home" => {
            if !args.is_empty() {
                return Err(ParseError {
                    line: line_num,
                    message: format!("'home' takes no arguments, got: {}", args.join(" ")),
                });
            }
            Ok(SemanticOperation::Home(HomeOp {
                origin: OperationId(format!("home-{line_num}")),
            }))
        }
        other => Err(ParseError {
            line: line_num,
            message: format!("unknown command '{other}'"),
        }),
    }
}

/// Extract named arguments (key=value) from the tail of an argument list.
fn extract_named_args<'a>(args: &[&'a str]) -> (Vec<&'a str>, Vec<(&'a str, &'a str)>) {
    let mut positional = Vec::new();
    let mut named = Vec::new();

    for arg in args {
        if let Some(eq_pos) = arg.find('=') {
            let key = &arg[..eq_pos];
            let value = &arg[eq_pos + 1..];
            named.push((key, value));
        } else {
            positional.push(*arg);
        }
    }

    (positional, named)
}

fn extract_tool(named: &[(&str, &str)]) -> Option<ToolId> {
    named
        .iter()
        .find(|(k, _)| *k == "tool")
        .map(|(_, v)| ToolId(v.to_string()))
}

fn parse_pick(args: &[&str], line_num: usize) -> Result<SemanticOperation, ParseError> {
    let (pos, named) = extract_named_args(args);

    if pos.is_empty() {
        return Err(ParseError {
            line: line_num,
            message: "'pick' requires at least an object name".into(),
        });
    }

    let object = ObjectId(pos[0].to_string());
    let tool = extract_tool(&named);

    Ok(SemanticOperation::Pick(PickOp {
        origin: OperationId(format!("pick-{line_num}")),
        object,
        tool,
    }))
}

fn parse_place(args: &[&str], line_num: usize) -> Result<SemanticOperation, ParseError> {
    let (pos, named) = extract_named_args(args);

    // Expected: place <object> at <location>
    if pos.len() < 3 || pos[1] != "at" {
        return Err(ParseError {
            line: line_num,
            message: "'place' requires format: place <object> at <location>".into(),
        });
    }

    let object = ObjectId(pos[0].to_string());
    let destination = LocationId(pos[2].to_string());
    let tool = extract_tool(&named);

    Ok(SemanticOperation::Place(PlaceOp {
        origin: OperationId(format!("place-{line_num}")),
        object,
        destination,
        tool,
    }))
}

fn parse_move_to(args: &[&str], line_num: usize) -> Result<SemanticOperation, ParseError> {
    let (pos, named) = extract_named_args(args);

    if pos.is_empty() {
        return Err(ParseError {
            line: line_num,
            message: "'move_to' requires a location name".into(),
        });
    }

    let destination = LocationId(pos[0].to_string());
    let tool = extract_tool(&named);

    Ok(SemanticOperation::MoveTo(MoveToOp {
        origin: OperationId(format!("move_to-{line_num}")),
        destination,
        tool,
    }))
}

fn parse_duration(s: &str, line_num: usize) -> Result<Duration, ParseError> {
    let s = s.trim();
    if s.ends_with("ms") {
        let val: f64 = s[..s.len() - 2].parse().map_err(|_| ParseError {
            line: line_num,
            message: format!("invalid duration '{s}'"),
        })?;
        Ok(Duration::from_secs_f64(val / 1000.0))
    } else if s.ends_with('s') {
        let val: f64 = s[..s.len() - 1].parse().map_err(|_| ParseError {
            line: line_num,
            message: format!("invalid duration '{s}'"),
        })?;
        Ok(Duration::from_secs_f64(val))
    } else {
        Err(ParseError {
            line: line_num,
            message: format!("invalid duration '{s}': expected format like 500ms, 2s, or 1.5s"),
        })
    }
}

fn parse_wait(args: &[&str], line_num: usize) -> Result<SemanticOperation, ParseError> {
    if args.is_empty() {
        return Err(ParseError {
            line: line_num,
            message: "'wait' requires a duration (e.g., 500ms, 2s)".into(),
        });
    }

    let duration = parse_duration(args[0], line_num)?;

    Ok(SemanticOperation::Wait(WaitOp {
        origin: OperationId(format!("wait-{line_num}")),
        duration,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_empty() {
        let program = parse("").unwrap();
        assert!(program.operations.is_empty());
    }

    #[test]
    fn parse_comments_only() {
        let program = parse("# comment\n  # another\n").unwrap();
        assert!(program.operations.is_empty());
    }

    #[test]
    fn parse_pick() {
        let program = parse("pick bolt").unwrap();
        assert_eq!(program.operations.len(), 1);
        match &program.operations[0] {
            SemanticOperation::Pick(PickOp { object, tool, .. }) => {
                assert_eq!(object.0, "bolt");
                assert!(tool.is_none());
            }
            _ => panic!("expected Pick"),
        }
    }

    #[test]
    fn parse_pick_with_tool() {
        let program = parse("pick bolt tool=gripper-1").unwrap();
        assert_eq!(program.operations.len(), 1);
        match &program.operations[0] {
            SemanticOperation::Pick(PickOp { object, tool, .. }) => {
                assert_eq!(object.0, "bolt");
                assert_eq!(tool.as_ref().unwrap().0, "gripper-1");
            }
            _ => panic!("expected Pick"),
        }
    }

    #[test]
    fn parse_place() {
        let program = parse("place bolt at tray").unwrap();
        assert_eq!(program.operations.len(), 1);
        match &program.operations[0] {
            SemanticOperation::Place(PlaceOp {
                object,
                destination,
                ..
            }) => {
                assert_eq!(object.0, "bolt");
                assert_eq!(destination.0, "tray");
            }
            _ => panic!("expected Place"),
        }
    }

    #[test]
    fn parse_move_to() {
        let program = parse("move_to station-2").unwrap();
        assert_eq!(program.operations.len(), 1);
        match &program.operations[0] {
            SemanticOperation::MoveTo(MoveToOp { destination, .. }) => {
                assert_eq!(destination.0, "station-2");
            }
            _ => panic!("expected MoveTo"),
        }
    }

    #[test]
    fn parse_wait_ms() {
        let program = parse("wait 500ms").unwrap();
        match &program.operations[0] {
            SemanticOperation::Wait(WaitOp { duration, .. }) => {
                assert_eq!(*duration, Duration::from_millis(500));
            }
            _ => panic!("expected Wait"),
        }
    }

    #[test]
    fn parse_wait_seconds() {
        let program = parse("wait 2s").unwrap();
        match &program.operations[0] {
            SemanticOperation::Wait(WaitOp { duration, .. }) => {
                assert_eq!(*duration, Duration::from_secs(2));
            }
            _ => panic!("expected Wait"),
        }
    }

    #[test]
    fn parse_wait_fractional() {
        let program = parse("wait 1.5s").unwrap();
        match &program.operations[0] {
            SemanticOperation::Wait(WaitOp { duration, .. }) => {
                assert_eq!(*duration, Duration::from_millis(1500));
            }
            _ => panic!("expected Wait"),
        }
    }

    #[test]
    fn parse_home() {
        let program = parse("home").unwrap();
        assert_eq!(program.operations.len(), 1);
        assert!(matches!(program.operations[0], SemanticOperation::Home(_)));
    }

    #[test]
    fn parse_full_program() {
        let script = "\
# Assemble bolt
pick bolt
wait 500ms
place bolt at tray
home";
        let program = parse(script).unwrap();
        assert_eq!(program.operations.len(), 4);
        assert!(matches!(program.operations[0], SemanticOperation::Pick(_)));
        assert!(matches!(program.operations[1], SemanticOperation::Wait(_)));
        assert!(matches!(program.operations[2], SemanticOperation::Place(_)));
        assert!(matches!(program.operations[3], SemanticOperation::Home(_)));
    }

    #[test]
    fn parse_unknown_command_errors() {
        let result = parse("jump 10");
        assert!(result.is_err());
    }

    #[test]
    fn parse_home_with_args_errors() {
        let result = parse("home somewhere");
        assert!(result.is_err());
    }

    #[test]
    fn parse_place_missing_at_errors() {
        let result = parse("place bolt tray");
        assert!(result.is_err());
    }

    #[test]
    fn parse_invalid_duration_errors() {
        let result = parse("wait forever");
        assert!(result.is_err());
    }

    #[test]
    fn parse_pick_empty_errors() {
        let result = parse("pick");
        assert!(result.is_err());
    }
}
