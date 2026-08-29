# Thalos Language Specification v0.1

**Status**: Frozen (v0.1)  
**Date**: August 2026  
**Crate**: `thalos-lang`, `thalos-semantic`, `thalos-planning`

---

## 1. Core Invariant & Architectural Philosophy

Thalos DSL (`.thls`) is a domain-specific language designed exclusively for industrial offline robot programming and spatial motion composition.

> **Fundamental Invariant**:  
> *The DSL describes explicit robotic intent. The compiler verifies semantic and mathematical validity. The planner determines physical feasibility.*

```text
       .thls Source Code
               │
               ▼
          thalos-lang (Parser)
               │  AST
               ▼
        thalos-semantic (Typecheck + Purity + Resolution)
               │  ResolvedProgram
               ▼
        thalos-planning (IK + Trajectory + Limits + Collisions)
               │  PlannedProgram
               ▼
        Execution System / Simulation
```

---

## 2. Type System & Physical Units

Thalos enforces strict physical dimensions at compile-time to prevent unit-mismatch errors in spatial calculations.

### 2.1 Primitive Physical Types
* `Length`: Scalar distance with explicit unit (`mm`, `cm`, `m`). Evaluates to standard meters (`m`).
* `Angle`: Scalar rotation with explicit unit (`deg`, `rad`). Evaluates to standard radians (`rad`).
* `Duration`: Time delta with explicit unit (`ms`, `s`). Evaluates to standard seconds (`s`).
* `Float` / `Int`: Pure numeric scalars for grid counters and scaling.
* `String`: Text literals for I/O channel identifiers (`"gripper"`, `"arc_enable"`).
* `Bool`: Truth values (`true`, `false`).

### 2.2 Composite Spatial Types
* `Vector3`: 3D displacement `[Length, Length, Length]`.
* `Position`: 3D Cartesian point `position([x, y, z])` or `position(Vector3)`.
* `Quaternion`: Orientation quaternion `quaternion(x, y, z, w)`.
* `Pose`: 6DOF transform `pose(Position, Quaternion)` or `pose(Position, Euler)`.
* `JointConfiguration`: Multi-axis joint vector `joints(angle1, angle2, ...)`.

---

## 3. Operations & Type Combinations

| Operator | Left Type | Right Type | Result Type | Semantic Meaning |
| :--- | :--- | :--- | :--- | :--- |
| `+` | `Position` | `Vector3` | `Position` | Offset a 3D position by a displacement vector |
| `+` | `Vector3` | `Vector3` | `Vector3` | Vector addition |
| `-` | `Position` | `Position` | `Vector3` | Relative vector displacement between positions |
| `-` | `Position` | `Vector3` | `Position` | Negative offset |
| `*` | `Vector3` | `Float` | `Vector3` | Vector scaling |
| `/` | `Vector3` | `Float` | `Vector3` | Vector division |

---

## 4. Entity Declarations & Scope Rules

### 4.1 `const`
Mandatory compile-time constant. Must evaluate to a static `CompileTimeValue`.
```thalos
const APPROACH_HEIGHT = [0mm, 0mm, 150mm]
```

### 4.2 `target`
Named spatial target (Position, Pose, or JointConfiguration). Immutable once declared.
```thalos
target home = joints(0deg, -45deg, 90deg, 0deg, 45deg, 0deg)
target pick_station = position([1500mm, 300mm, 200mm])
```

### 4.3 `let`
Local value binding within function scope. Immutable in v0.1.
```thalos
let target_p = origin + col;
```

---

## 5. Composition & Purity Contract

Functions (`fn`) are categorized by their return type into **Pure Calculations** or **Robotic Procedures**:

```text
fn name(params...) -> ReturnType
       │
       ├── ReturnType != Unit  ⇒  Pure Spatial Calculation (Effect-Free)
       └── ReturnType == Unit  ⇒  Robotic Procedure (Effects Allowed)
```

### 5.1 Pure Spatial Calculations
Must return a value (`Position`, `Pose`, `Vector3`, etc.). Any call to motion (`movej`, `movel`, `movec`), delay (`wait`), or I/O (`set_output`) results in a **compile-time semantic error**.
```thalos
fn above(p: Position) -> Position {
    p + [0mm, 0mm, 150mm]
}
```

### 5.2 Robotic Procedures
Functions returning `Unit` (implicit no return type) represent sequence routines. They can issue motion commands, I/O signals, and call other routines.
```thalos
fn pick(p: Position) {
    movej(above(p));
    movel(p);
    set_output("gripper", true);
    wait(200ms);
    movel(above(p));
}
```

---

## 6. Built-in Effect Statements

* **`movej(target)`**: Joint-space interpolated motion to target.
* **`movel(target)`**: Linear Cartesian path interpolated motion to target.
* **`movec(via, target)`**: Circular Cartesian path motion through `via` to `target`.
* **`wait(duration)`**: Program execution dwell.
* **`set_output(channel, state)`**: Digital I/O signal assertion.

---

## 7. Compiler IR & Intermediate Representation Pipeline

1. **`AST` (`thalos-lang`)**: Untyped syntax tree parsed via Chumsky combinators.
2. **`SemanticProgram` (`thalos-semantic`)**: Type-checked intermediate representation with constant folding and purity verification.
3. **`ResolvedProgram` (`thalos-semantic`)**: Monomorphized, call-expanded statement list preserving `Provenance` tracking.
4. **`PlanningInput` (`thalos-planning`)**: Lowered physical targets ready for IK solver and trajectory generation.

---

## 8. Taxonomy of Compiler & Runtime Errors

```text
                       Thalos Error Taxonomy
                                 │
   ┌──────────────┬──────────────┼──────────────┬──────────────┐
   │              │              │              │              │
SyntaxError   SemanticError  SceneError    PlanningError  RuntimeFault
 (Parser)     (Type/Purity) (Kinematics)    (IK/Collision) (Hardware)
```

1. **SyntaxError**: Unmatched tokens, invalid unit suffixes, malformed expressions.
2. **SemanticError**: Type mismatch, purity violation (motion inside pure function), unresolved identifier.
3. **SceneError**: Target missing kinematic frame in active robot model.
4. **PlanningError**: IK unreachable position, joint limit violation, collision along trajectory.
5. **RuntimeFault**: Emergency stop, actuator disconnect, execution drift.

---

## 9. Validation Corpus & Test Inventory

The v0.1 specification is validated by three comprehensive test suites:

* **Compiler Semantics Suite (`thls_corpus_tests.rs`)**: 15 tests covering AST lowering, purity rejection, scope resolution, and binary operator rules.
* **Industrial Programs Suite (`industrial_programs.rs`)**: 12 real-world industrial routines covering Pick & Place clearance, multi-angle Pose/Euler inspection, seam welding, continuous dispensing, 3D palletizing, conveyor handshake, tool change, and multi-pass welding.
* **Golden Corpus Parity (`golden_corpus.rs`)**: Ensures TypeScript frontend parser and Rust compiler parser produce identical AST structures.

---

## 10. v0.2 Architectural Preview

Future iterations beyond v0.1 will focus on cell-level semantics rather than general control flow:

* **Spatial Frames**: WorkObject / UserFrame / StationFrame scoping (`target p = position(...) in workpiece`).
* **Tool Specifications**: Dynamic TCP definition (`in tool(my_gripper)`).
* **Cell Device APIs**: Evolving `set_output` into structured device abstractions (`gripper.close()`, `camera.trigger()`).
