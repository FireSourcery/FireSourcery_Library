# CLAUDE.md — FireSourcery_Library

## Project Overview
**FireSourcery_Library** is a modular, layered embedded systems library written in **C** for motor control applications (primarily targeting **ARM Cortex-M** microcontrollers NXP S32K / Kinetis KE0x families). It implements a full motor controller stack including FOC (Field-Oriented Control), sensor feedback, protocol communication, and peripheral abstraction.

## Core Principles
- **Expression through code**: Use code as the language to express what its doing. Favor code that expresses itself over extensive comments.
- **Declarative functions**. Favor less local variables, particularly in cases that do not impact optimization. Function body should look like the equation in its docstring, not reconstruct it.
- **Simplicity First**: Make every change as simple as possible. Impact minimal code. Favor regularity.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.

### Demand Elegance
- For non-trivial changes: pause and ask "is there a more elegant way?"
- Consider established or published design patterns first to eliminate code duplication
- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"
- Skip this for simple, obvious fixes – don't over-engineer
- Challenge your own work before presenting it

### Documentation
- Keep comments concise. Comments are reserved for points that the code cannot express, not what the code is doing.
- Rather than describing what the code is doing using comments, use visualizations i.e mermaid diagrams in a separate markdown file.
- When referring to a concept that is already modeled by a type or struct in code, refer to that type.
- Markdown symbol notation:
  - `[Name]` — anything defined in this codebase, spelled exactly as in code: `[Motor_StateId_T]`, `[Motor_Enable]`, `[MOTOR_STATE_INIT]`.
  - `` `code` `` — literal code that is not one of our names: expressions, calls with arguments, keywords, standard-library names, paths, commands.
  - Bare CAPS — a member name (state, input, mode) once its section has named the type, e.g. INIT under `[Motor_StateId_T]`.
  - Bold — emphasis only, never code.
- Extensive descriptions go in a separate markdown file, such as architecture notes.
- Architecture notes and findings live in `Docs/`: Conventions, Patterns, Decisions, Findings, Modules, References. Index: [Docs/README.md](Docs/README.md).

### Self-Improvement Loop
- After ANY correction from the user: update `.claude/tasks/lessons.md` with the pattern
- Write rules for yourself that prevent the same mistake
- Ruthlessly iterate on these lessons until mistake rate drops
- Review lessons at session start for relevant project
- Research patterns before starting to code.

## Task Management
1. **Plan First**: Write plan to `.claude/tasks/todo.md` with checkable items
2. **Verify Plan**: Check in before starting implementation
3. **Track Progress**: Mark items complete as you go
4. **Explain Changes**: High-level summary at each step
5. **Document Results**: Add review section to `.claude/tasks/todo.md`
6. **Capture Lessons**: Update `.claude/tasks/lessons.md` after corrections

## Build & Toolchain
- **Compiler:** ARM GCC (`arm-none-eabi-gcc` 15.2.1)
- **Target Architecture:** ARM Cortex-M4/M0+
- **Build System:** Makefile / CMake
- **C Standard:** C23
- **No dynamic allocation** — all memory is statically allocated
<!-- - **No standard library dependencies** beyond `<stdint.h>`, `<stdbool.h>`, `<stddef.h>`, `<string.h>` -->

## Key Conventions

### Architecture Patterns
- **HAL abstraction**: Hardware access through `HAL_*.h` headers with platform-specific implementations
- HAL functions must have focused signatures, few and focused parameters. Split logic into granular blocks.
- **Thread/ISR separation**: `_Thread.h` files define periodic processing functions; ISR-safe boundaries are explicit
- **State machines**: Hierarchical state machine framework in `Framework/StateMachine/` used extensively for motor control states
<!-- - **Fixed-point math**: 16-bit fractional (`fract16`) and Q16 fixed-point arithmetic — no floating point at runtime -->
- **Const struct descriptors pattern**:  "Static Polymorphism Pattern". Const struct handle, holds pointer to runtime state in RAM.
    This pattern is only for hardware descriptors and static polymorphism. Do not use it for what could be mutable only structs.
- **Stateless pure functions layer**: function parameter contain the entire state. Reusable pieces, or established, published concepts, physical or not.
- **Untypedef'd structs are the pure layer's multiple returns**, e.g. `struct foc_dq`, `struct foc_alphabeta`, `struct sta_axis`. A pure function returns one by value, and the caller unpacks it into its own fields at the call.
    The stateful layer never stores one as a field, and never takes or returns one in its functions, private helpers included.
- Utility functions pass context that is the closest layer the logic requires. No tramp parameters. Don't pass bool.
<!-- - **NvMemory pattern**: Configuration stored in Flash/EEPROM with structured read/write abstraction -->
- **Expression style**: Prefer concise, declarative expressions over procedural manipulation. Use library primitives (`math_clamp`, `fract16_div`, `fract16_mul`, `math_min/max`) composed into single expressions that mirror the domain formula. Avoid early-return ladders and temporaries that break a formula into steps. Function body should look like the equation in its docstring, not reconstruct it.
- For functions called with known constant values at compile time, input parameter correctness is a part of the API contract. favor `assert` instead of runtime error checking logic
- No token pasting technique for preprocessor macros. VA_ARGS for generic handling ergonomics is okay.

### Important Rules
- **No heap allocation** (`malloc`/`free` are never used)
- **No floating point** in runtime code paths — all math is integer/fixed-point
- **ISR-safe**: Shared data between ISR and thread contexts uses volatile and critical sections
- **Const-correctness**: Config/calibration data is `const` qualified and stored in non-volatile memory regions
- **Reentrant-safe**: Module functions operate on explicit instance pointers, no hidden global state

### Code Style
- **Short functions on one line**.
- **Function parameters order**: most "constant" first. The parameter least likely to change or is constant at call site goes first.
- **PascalCase** for types, structs, enums, and function names: `Motor_FOC_ProcAngle()`, `MotorController_Init()`
- **UPPER_SNAKE_CASE** for macros, enum values, and constants: `MOTOR_STATE_RUN`, `HAL_ADC_CHANNEL_COUNT`
- **camelCase** for local variables
- **snake_case** for pure functions on primitives
- Prefix functions with module name: `Motor_`, `Phase_`, `Encoder_`, `UART_`, etc.
- Files prefixed with `_` (e.g., `_Motor_Config.h`) are internal/private headers not intended for external inclusion
- Public API headers match the module directory name: `Motor.h`, `StateMachine.h`, `Protocol.h`

## API Design
- **Minimize forwarders.** When a wrapper function is a single-line pass-through to a peer module (`Outer_Foo(p) { return Inner_Foo(&p->inner); }`), it duplicates API surface without adding value. Prefer:
  - **Free-function bridges** — no wrapper type — when coordination surface is small (≤ ~5 functions) and peers are public vocabulary. Precedent: C stdlib (`fprintf`, `memcpy`).
  - **Mediator with peer getters** — wrapper keeps only coordination functions; exposes peers via getters — when coordination points are numerous or the wrapper holds its own state.
  - **Full-forwarder facade** — only when peer types must be hidden from callers (rare).
- Types earn themselves by holding state or enforcing invariants, not by re-exporting another type's API.
- For value-embedded members (`Outer_T { Inner_T Base; }`), forwarders are acceptable to keep inner layout private. Never return a mutable pointer to a value-embedded member.

## API Scoping
- **Parameters declare dependencies.** A function takes the narrowest struct that contains every field it reads or writes. `f(Input_T *)` states "reads only config."
- **Multiple access levels per module is expected. Consistency is within a scope, not across scopes.
- **Name by scope when disambiguation helps.** `Motor_Config_DegOfRpm` — the middle identifier says which struct-level. Reserve leading underscore for module-internal helpers, not narrower-scope public API.
- **Promote to a wider scope only on demand.** Add `Motor_DegOfRpm(const Motor_State_T *)` as a thin forwarder only when a real caller has the wider handle often enough that the extra typing matters.
- **Primary structs are exempt for clean ergonomics** e.g. `*_State_T`, `*_Config_T`

## Directory Map & Module Dependencies
See [ARCHITECTURE.md](ARCHITECTURE.md) (mermaid diagrams). Keep its diagrams updated when modules are added or moved.


