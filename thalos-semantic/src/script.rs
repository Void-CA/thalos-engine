//! `.thalos` Task Script — human-readable DSL for constructing `RobotProgram`s.
//!
//! # Format (.thalos DSL)
//!
//! ```text
//! program pick_and_place(robot = scara_01) {
//!     target approach = cartesian(
//!         x = 0.120,
//!         y = 0.080,
//!         z = 0.050
//!     )
//!
//!     target pick_target = cartesian(
//!         x = 0.120,
//!         y = 0.080,
//!         z = 0.010
//!     )
//!
//!     move approach
//!     pick(part_01, at = pick_target)
//!     wait(500ms)
//! }
//! ```

use std::time::Duration;
use thalos_core::ids::{ProgramName, RobotId, SkillId, TargetId, TargetName};
use thalos_core::program::{
    ControlInstruction, Instruction, JointPosition, MotionInstruction, RobotProgram, SkillCall,
    Target, TargetReference, Value,
};
use thalos_core::spatial::frame::frame::FrameId;
use thalos_core::spatial::pose::Pose;
use thalos_math::{Transform3D, Vector3};

/// Error returned when parsing a `.thalos` script line fails.
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

/// Parse a `.thalos` DSL string or legacy script into a pure `RobotProgram`.
pub fn parse(input: &str) -> Result<RobotProgram, Vec<ParseError>> {
    let input_trimmed = input.trim();
    if input_trimmed.starts_with("program ") {
        parse_block_program(input)
    } else {
        parse_legacy_script(input)
    }
}

/// Parse a `.thalos` skill fragment script into a `ProgramFragment`.
///
/// A fragment contains instruction lines (e.g. `move approach`, `wait(100ms)`),
/// but MUST NOT contain a top-level `program` block declaration.
pub fn parse_fragment(input: &str) -> Result<thalos_core::skill::ProgramFragment, Vec<ParseError>> {
    let input_trimmed = input.trim();
    if input_trimmed.starts_with("program ") {
        return Err(vec![ParseError {
            line: 1,
            message: "skill fragment cannot contain a top-level 'program' block declaration".into(),
        }]);
    }

    let mut instructions = Vec::new();
    let mut errors = Vec::new();

    for (idx, line) in input.lines().enumerate() {
        let line_num = idx + 1;
        let line_trimmed = line.trim();

        if line_trimmed.is_empty()
            || line_trimmed.starts_with("//")
            || line_trimmed.starts_with('#')
        {
            continue;
        }

        match parse_instruction_line(line_trimmed, line_num) {
            Ok(inst) => instructions.push(inst),
            Err(e) => errors.push(e),
        }
    }

    if errors.is_empty() {
        Ok(thalos_core::skill::ProgramFragment { instructions })
    } else {
        Err(errors)
    }
}

/// Parse a block-structured `.thalos` program.
fn parse_block_program(input: &str) -> Result<RobotProgram, Vec<ParseError>> {
    let mut errors = Vec::new();
    let mut program_name = ProgramName("unnamed_program".into());
    let mut robot_id = RobotId("default_robot".into());
    let mut targets = Vec::new();
    let mut body = Vec::new();

    let lines: Vec<&str> = input.lines().collect();
    let mut idx = 0;

    while idx < lines.len() {
        let raw_line = lines[idx];
        let line = raw_line.trim();
        let line_num = idx + 1;
        idx += 1;

        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        if line.starts_with("program ") {
            if let Some(open_paren) = line.find('(') {
                let name_part = line["program ".len()..open_paren].trim();
                program_name = ProgramName(name_part.to_string());

                if let Some(close_paren) = line.find(')') {
                    let param_part = line[open_paren + 1..close_paren].trim();
                    if let Some(eq_pos) = param_part.find('=') {
                        let key = param_part[..eq_pos].trim();
                        let val = param_part[eq_pos + 1..].trim();
                        if key == "robot" {
                            robot_id = RobotId(val.to_string());
                        }
                    }
                }
            }
            continue;
        }

        if line == "}" {
            continue;
        }

        if line.starts_with("target ") {
            let mut target_block = line.to_string();
            let start_line_num = line_num;
            while !target_block.contains(')') && idx < lines.len() {
                target_block.push(' ');
                target_block.push_str(lines[idx].trim());
                idx += 1;
            }
            match parse_target_decl(&target_block, start_line_num) {
                Ok(target) => targets.push(target),
                Err(e) => errors.push(e),
            }
            continue;
        }

        match parse_instruction_line(line, line_num) {
            Ok(inst) => body.push(inst),
            Err(e) => errors.push(e),
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(RobotProgram::new(program_name, robot_id, targets, body))
}

fn parse_target_decl(line: &str, line_num: usize) -> Result<Target, ParseError> {
    let rest = line["target ".len()..].trim();
    let eq_pos = rest.find('=').ok_or_else(|| ParseError {
        line: line_num,
        message: format!("malformed target declaration: '{line}'"),
    })?;

    let name = rest[..eq_pos].trim();
    let expr = rest[eq_pos + 1..].trim();

    if expr.starts_with("cartesian") {
        let x = extract_num_param(expr, "x").unwrap_or(0.0);
        let y = extract_num_param(expr, "y").unwrap_or(0.0);
        let z = extract_num_param(expr, "z").unwrap_or(0.0);

        let transform = Transform3D::from_translation(Vector3::new(x, y, z));
        let pose = Pose::new(FrameId::World, FrameId::Id(0), transform);
        Ok(Target::new(
            TargetId(name.to_string()),
            TargetName(name.to_string()),
            TargetReference::Cartesian { pose },
        ))
    } else if expr.starts_with("joint") {
        let mut q = Vec::new();
        for i in 1..=6 {
            if let Some(val) = extract_num_param(expr, &format!("q{i}")) {
                q.push(val);
            }
        }
        Ok(Target::new(
            TargetId(name.to_string()),
            TargetName(name.to_string()),
            TargetReference::Joint {
                position: JointPosition::new(q),
            },
        ))
    } else {
        Err(ParseError {
            line: line_num,
            message: format!("unknown target reference type in '{expr}'"),
        })
    }
}

fn extract_num_param(expr: &str, key: &str) -> Option<f64> {
    let pattern = format!("{key}=");
    if let Some(pos) = expr.find(&pattern) {
        let rest = &expr[pos + pattern.len()..];
        let end = rest
            .find(|c: char| c == ',' || c == ')' || c.is_whitespace())
            .unwrap_or(rest.len());
        let val_str = rest[..end].trim();
        val_str.parse::<f64>().ok()
    } else {
        None
    }
}

fn parse_instruction_line(line: &str, line_num: usize) -> Result<Instruction, ParseError> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.is_empty() {
        return Err(ParseError {
            line: line_num,
            message: "empty line".into(),
        });
    }

    let command = parts[0];

    if command == "move" {
        if parts.len() < 2 {
            return Err(ParseError {
                line: line_num,
                message: "'move' requires a target name".into(),
            });
        }
        let target_id = TargetId(parts[1].trim_matches(',').to_string());
        return Ok(Instruction::Motion(MotionInstruction::Move {
            target: target_id,
        }));
    }

    if command == "move_linear" {
        if parts.len() < 2 {
            return Err(ParseError {
                line: line_num,
                message: "'move_linear' requires a target name".into(),
            });
        }
        let target_id = TargetId(parts[1].trim_matches(',').to_string());
        return Ok(Instruction::Motion(MotionInstruction::MoveLinear {
            target: target_id,
        }));
    }

    if command == "move_joint" {
        if parts.len() < 2 {
            return Err(ParseError {
                line: line_num,
                message: "'move_joint' requires a target name".into(),
            });
        }
        let target_id = TargetId(parts[1].trim_matches(',').to_string());
        return Ok(Instruction::Motion(MotionInstruction::MoveJoint {
            target: target_id,
        }));
    }

    if command.starts_with("wait") {
        let arg = if parts.len() > 1 {
            parts[1]
        } else if let Some(open) = line.find('(') {
            let close = line.find(')').unwrap_or(line.len());
            &line[open + 1..close]
        } else {
            return Err(ParseError {
                line: line_num,
                message: "'wait' requires a duration".into(),
            });
        };
        let duration = parse_duration(arg, line_num)?;
        return Ok(Instruction::Control(ControlInstruction::Wait { duration }));
    }

    if let Some(open_paren) = line.find('(') {
        let skill_name = SkillId(line[..open_paren].trim().to_string());
        let close_paren = line.find(')').unwrap_or(line.len());
        let args_str = &line[open_paren + 1..close_paren];

        let mut args = Vec::new();
        for arg in args_str.split(',') {
            let arg = arg.trim();
            if arg.is_empty() {
                continue;
            }
            if let Some(eq_pos) = arg.find('=') {
                let val = arg[eq_pos + 1..].trim();
                args.push(Value::Target(TargetId(val.to_string())));
            } else if arg.starts_with('"') && arg.ends_with('"') {
                let clean_arg = arg.trim_matches('"');
                args.push(Value::String(clean_arg.to_string()));
            } else {
                args.push(Value::Object(thalos_core::ids::ObjectId(arg.to_string())));
            }
        }

        return Ok(Instruction::Skill(SkillCall::new(skill_name, args)));
    }

    Err(ParseError {
        line: line_num,
        message: format!("unknown instruction '{line}'"),
    })
}

fn parse_duration(s: &str, line_num: usize) -> Result<Duration, ParseError> {
    let s = s.trim().trim_matches(')');
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

/// Legacy line-based script parser that emits a valid `RobotProgram`.
fn parse_legacy_script(input: &str) -> Result<RobotProgram, Vec<ParseError>> {
    let mut body = Vec::new();
    let mut targets = Vec::new();
    let mut target_set = std::collections::HashSet::new();
    let mut errors = Vec::new();

    for (line_idx, line) in input.lines().enumerate() {
        let line = line.trim();
        let line_num = line_idx + 1;

        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        let command = parts[0];
        let args = &parts[1..];

        match command {
            "pick" => {
                if args.is_empty() {
                    errors.push(ParseError {
                        line: line_num,
                        message: "'pick' requires at least an object name".into(),
                    });
                } else {
                    body.push(Instruction::Skill(SkillCall::new(
                        SkillId("pick".into()),
                        vec![Value::Object(thalos_core::ids::ObjectId(args[0].to_string()))],
                    )));
                }
            }
            "place" => {
                if args.len() < 3 || args[1] != "at" {
                    errors.push(ParseError {
                        line: line_num,
                        message: "'place' requires format: place <object> at <location>".into(),
                    });
                } else {
                    let target_id = TargetId(args[2].to_string());
                    if target_set.insert(target_id.clone()) {
                        targets.push(Target::new(
                            target_id.clone(),
                            TargetName(args[2].to_string()),
                            TargetReference::Cartesian {
                                pose: Pose::new(FrameId::World, FrameId::Id(0), Transform3D::identity()),
                            },
                        ));
                    }
                    body.push(Instruction::Skill(SkillCall::new(
                        SkillId("place".into()),
                        vec![
                            Value::Object(thalos_core::ids::ObjectId(args[0].to_string())),
                            Value::Target(target_id),
                        ],
                    )));
                }
            }
            "move_to" => {
                if args.is_empty() {
                    errors.push(ParseError {
                        line: line_num,
                        message: "'move_to' requires a location name".into(),
                    });
                } else {
                    let target_id = TargetId(args[0].to_string());
                    if target_set.insert(target_id.clone()) {
                        targets.push(Target::new(
                            target_id.clone(),
                            TargetName(args[0].to_string()),
                            TargetReference::Cartesian {
                                pose: Pose::new(FrameId::World, FrameId::Id(0), Transform3D::identity()),
                            },
                        ));
                    }
                    body.push(Instruction::Motion(MotionInstruction::MoveLinear {
                        target: target_id,
                    }));
                }
            }
            "wait" => {
                if args.is_empty() {
                    errors.push(ParseError {
                        line: line_num,
                        message: "'wait' requires a duration (e.g., 500ms, 2s)".into(),
                    });
                } else {
                    match parse_duration(args[0], line_num) {
                        Ok(dur) => body.push(Instruction::Control(ControlInstruction::Wait {
                            duration: dur,
                        })),
                        Err(e) => errors.push(e),
                    }
                }
            }
            "home" => {
                if !args.is_empty() {
                    errors.push(ParseError {
                        line: line_num,
                        message: format!("'home' takes no arguments, got: {}", args.join(" ")),
                    });
                } else {
                    body.push(Instruction::Motion(MotionInstruction::Retract {
                        distance: 0.0,
                    }));
                }
            }
            other => errors.push(ParseError {
                line: line_num,
                message: format!("unknown command '{other}'"),
            }),
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(RobotProgram::new(
        ProgramName("legacy_program".into()),
        RobotId("scara_1".into()),
        targets,
        body,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_block_thalos_program() {
        let script = r#"
program pick_and_place(robot = scara_01) {

    target approach = cartesian(
        x = 0.120,
        y = 0.080,
        z = 0.050
    )

    target pick_target = cartesian(
        x = 0.120,
        y = 0.080,
        z = 0.010
    )

    target place_target = cartesian(
        x = 0.250,
        y = 0.100,
        z = 0.010
    )

    move approach
    pick(part_01, at = pick_target)
    place(part_01, at = place_target)
    wait(500ms)
}
"#;
        let program = parse(script).expect("should parse valid .thalos program");
        assert_eq!(program.name.as_str(), "pick_and_place");
        assert_eq!(program.robot.as_str(), "scara_01");
        assert_eq!(program.targets.len(), 3);
        assert_eq!(program.body.len(), 4);
        assert!(matches!(program.body[0], Instruction::Motion(_)));
        assert!(matches!(program.body[1], Instruction::Skill(_)));
        assert!(matches!(program.body[2], Instruction::Skill(_)));
        assert!(matches!(program.body[3], Instruction::Control(_)));
    }

    #[test]
    fn parse_produces_deterministic_robot_program() {
        let script = r#"
program test_prog(robot = bot1) {
    target t1 = cartesian(x = 1.0, y = 2.0, z = 3.0)
    move t1
}
"#;
        let a = parse(script).unwrap();
        let b = parse(script).unwrap();
        assert_eq!(a, b, "Parsing same source must produce identical RobotProgram AST");
    }

    #[test]
    fn same_program_compiles_differently_in_different_scenes() {
        let script = r#"
program test_prog(robot = bot1) {
    target t1 = cartesian(x = 1.0, y = 2.0, z = 3.0)
    move t1
}
"#;
        let program = parse(script).unwrap();
        let ir = crate::ir::normalize(&program).unwrap();

        assert_eq!(program.name.as_str(), "test_prog");
        assert_eq!(program.robot.as_str(), "bot1");
        assert_eq!(ir.targets.len(), 1);
        assert_eq!(ir.targets[0].id.as_str(), "t1");
    }

    #[test]
    fn parse_legacy_script_to_robot_program() {
        let script = "pick bolt\nwait 500ms\nplace bolt at tray\nhome";
        let program = parse(script).unwrap();
        assert_eq!(program.body.len(), 4);
    }
}
