# ADR-002: Robot Capability Model & Skill Resolution

## Status
Accepted

## Context
In Thalos, program intention (`RobotProgram`) must remain decoupled from specific hardware kinematics, cell layouts, and robot skill implementations. Previously, skill resolution was monolithic or relied on string matching inside IR passes.

## Architectural Invariants
1. **`RobotDefinition` declares capabilities**: A robot definition specifies what skills the robot supports (`Vec<SkillCapability>`). It does NOT own the skill implementation code.
2. **`SkillRegistry` resolves implementations**: Concrete `RobotSkill` implementations are dynamically indexed and resolved by `(RobotId, SkillId)` during lowering.
3. **`RobotProgram` AST is pure**: Source `.thalos` code and the resulting `RobotProgram` AST never contain skill implementations or robot-specific configuration. `program_a == program_b` holds true across target robots.

## Target Execution Pipeline
```text
RobotProgram AST + LoweringContext(RobotDefinition, SkillRegistry)
                         │
                         ▼
                  SemanticIR (normalized)
                         │
                         ▼
          Skill Lowering via (RobotId, SkillId)
                         │
                         ▼
                  ExecutionProgram
```

## Next Phase Target (`RobotProfile`)
- Persist robot definitions and skill bindings in modular format (e.g. `scara-01/robot.toml`, `scara-01/skills/pick.toml`).
- Load `RobotProfile` -> construct `RobotDefinition` & populate `SkillRegistry` -> compile `RobotProgram` into `ExecutionProgram`.
