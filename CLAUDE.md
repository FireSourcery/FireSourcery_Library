# CLAUDE.md — FireSourcery_Library

## Project Overview
**FireSourcery_Library** is a modular, layered embedded systems library written in **C** for motor control applications (primarily targeting **ARM Cortex-M** microcontrollers NXP S32K / Kinetis KE0x families). It implements a full motor controller stack including FOC (Field-Oriented Control), sensor feedback, protocol communication, and peripheral abstraction.

## Core Principles
- **Expression through code**: Use code as the language to express what its doing. Favor code that expresses itself over extensive comments.
- **Declarative functions**. Favor less local variables, particularly in cases that do not impact optimization. Function body should look like the equation in its docstring, not reconstruct it.
- **Simplicity First**: Make every change as simple as possible. Impact minimal code. Favor regularity. Simplicity is measured on the resulting design (fewer concepts and adapters), not on the diff. Minimal Impact keeps unrelated code untouched; it does not veto the general form.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.

### Demand Elegance
- For non-trivial changes: pause and ask "is there a more elegant way?"
- **Find the general form.** Before adding a type, layer or adapter, ask which existing interface or published pattern it is an instance of: the framework's (`State_T`, `Packet_Codec_T`, a `HAL_*.h` contract) or another module's. If conforming lets existing consumers apply unchanged, propose that form beside the minimal one and name what it unlocks. Instance: the Mot protocol is a `Packet_Codec_T` (`MOT_PACKET_CODEC`), so the `Framework/Protocol` engine runs it unchanged. [Docs/Principles/GeneralForm.md](Docs/Principles/GeneralForm.md)
- The sign: an adapter from a framework type to a module's own, or a parallel struct beside one the framework already defines, means the two are one interface.
- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"
- Skip this for simple, obvious fixes – don't over-engineer
- Challenge your own work before presenting it

### Documentation
- Keep comments concise. Comments are reserved for points that the code cannot express, not what the code is doing.
- Rather than describing what the code is doing using comments, use visualizations i.e mermaid diagrams in a separate markdown file.
- When referring to a concept that is already modeled by a type or struct in code, refer to that type.
- Symbol notation for an entity that exists in code:
  - Markdown — backticks, so the preview highlights it: `TypeName_T`, `MOTOR_STATE_INIT`, even a bare member `INIT`.
  - C comments — brackets, `[TypeName_T]`, which the `.vscode/comment-reference` grammar highlights.
  - Bold — emphasis only, never code.
  - Older docs still use `[Name]` in markdown; convert them only when asked.
- **Rows and columns expose symmetry.** Where clauses share a form (the cases of one parameter, a value and its complement, one quantity per phase), set them as rows, one case per row, the shared parts aligned in columns. Apply it unsparingly: two parallel clauses are enough. C comments align with spaces; markdown uses a table. [Docs/Conventions/Comments.md](Docs/Conventions/Comments.md#layout)
- Extensive descriptions go in a separate markdown file, such as architecture notes.
- Architecture notes and findings live in `Docs/`: Principles, Conventions, Patterns, Decisions, Findings, References. Index: [Docs/README.md](Docs/README.md).
- A module's notes live beside its source, not in `Docs/`: `<Topic>.md` is the sheet, `<Topic>.exp.md` its detail and record. [Docs/Modules/README.md](Docs/Modules/README.md)
- A doc leads with a reference sheet of what is the case: facts, rules, options and their defaults, limits, and what is not measured. Derivations, each experiment, and verification logs go in files of their own.
- **Keep dated material apart from reference material.** History (logs, results, decisions, finished plans) is kept apart from documents that describe how things are now.
- Simulations go in `Motor/Docs/Simulation/<topic>/` with their plots; conventions in its [README.md](Motor/Docs/Simulation/README.md).
- Tracked files carry no absolute paths: relative to the repo or a sibling checkout, or a `<placeholder>` for a machine's tool.

### Self-Improvement Loop
- After ANY correction from the user: save it as a feedback memory, one fact with its why, written to prevent the same mistake
- Memory is current state: trim it, and expire what no longer holds
- A settled rule moves to its `Docs/` entry and leaves memory; one applied every session also gets a bullet here. Map: § Where knowledge goes in `README.md` of the Claude home, the folder that holds `.claude/memory/` (`autoMemoryDirectory`)
- Research patterns before starting to code.

## Task Management
1. **Plan First**: Write plan to `.claude/tasks/todo.md` with checkable items
2. **Verify Plan**: Check in before starting implementation
3. **Track Progress**: Mark items complete as you go
4. **Explain Changes**: High-level summary at each step
5. **Document Results**: Add review section to `.claude/tasks/todo.md`
6. **Capture Corrections**: a feedback memory after each correction

## Build & Toolchain
- **Compiler:** ARM GCC (`arm-none-eabi-gcc` 15.2.1)
- **Target Architecture:** ARM Cortex-M4/M0+
- **Build:** none in this repo, sources only (`Build/` is empty, `library.json` is parked). It compiles inside the board project that consumes it; verify a change in that build.
- **C Standard:** C23
- **No dynamic allocation** — all memory is statically allocated

## Key Conventions

### Architecture Patterns
- **HAL abstraction**: Hardware access through `HAL_*.h` headers with platform-specific implementations. A platform function the Peripheral layer does not call, used by the Board only, takes the platform prefix (`S32_`, `KE0x_`), and becomes `HAL_` when a Peripheral module needs it. [Peripheral/Notes.md](Peripheral/Notes.md)
- HAL functions must have focused signatures, few and focused parameters. Split logic into granular blocks.
- **Thread/ISR separation**: `_Thread.h` files define periodic processing functions; ISR-safe boundaries are explicit
- **State machines**: Hierarchical state machine framework in `Framework/StateMachine/` used extensively for motor control states.
- **Const struct descriptors pattern**:  "Static Polymorphism Pattern". Const struct handle, holds pointer to runtime state in RAM.
    This pattern is only for hardware descriptors and static polymorphism. Do not use it for what could be mutable only structs.
- **Stateless pure functions layer**: function parameter contain the entire state. Reusable pieces, or established, published concepts, physical or not.
- **Untypedef'd structs are the pure layer's multiple returns**, e.g. `struct foc_dq`, `struct foc_alphabeta`, `struct sta_axis`. A pure function returns one by value, and the caller unpacks it into its own fields at the call.
    The stateful layer never stores one as a field, and never takes or returns one in its functions, private helpers included.
- Utility functions pass context that is the closest layer the logic requires. No tramp parameters. Don't pass bool: [Docs/Principles/BooleanBlindness.md](Docs/Principles/BooleanBlindness.md)
- **Expression style**: Prefer concise, declarative expressions over procedural manipulation. Use library primitives (`math_clamp`, `fract16_div`, `fract16_mul`, `math_min/max`) composed into single expressions that mirror the domain formula. Avoid early-return ladders and temporaries that break a formula into steps. Function body should look like the equation in its docstring, not reconstruct it.
- For functions called with known constant values at compile time, input parameter correctness is a part of the API contract. favor `assert` instead of runtime error checking logic
- **Adding logic to a working function is a decision.** Removing logic needs no case. A repair goes in place; a constraint every caller meets is the `assert` above; any other check for correctness wraps the function in another one, so the original keeps its direct expression (`_Motor_Electrical_RateOfSi` wraps `angle16_of_arc`). A different behavior gets its own name. [Docs/Conventions/AddingLogic.md](Docs/Conventions/AddingLogic.md)
- No token pasting technique for preprocessor macros. VA_ARGS for generic handling ergonomics is okay.
- **Preprocessor paths sparingly**: they confuse the code. Variants are separate functions; an `#if` selects among them at one point only where it saves code, prevents a cross product, or is a configuration case (`FOC_ProcIFeedback`). A caller's or state's choice is a function call, a timing gap is tuning. An optional feature's definitions compile in every build; guard only the call or table row, and the linker drops them. [Docs/Conventions/ConditionalCompilation.md](Docs/Conventions/ConditionalCompilation.md), [Docs/Patterns/GuardedCall.md](Docs/Patterns/GuardedCall.md)

### Important Rules
- **No heap allocation** (`malloc`/`free` are never used)
- **No floating point** in runtime code paths — all math is integer/fixed-point
- **ISR-safe**: Shared data between ISR and thread contexts uses volatile and critical sections. A torn read or two-sided write found in review is not closed with a new lock; it is an open decision: [0010](Docs/Decisions/0010_SharedWordWrites.md), [0011](Docs/Decisions/0011_StateMachineLockNesting.md)
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
- A folder or file whose name starts with `.` is outside the library proper (a sketch, retired code, tool data), and so is a draft note `_*.md`. Skip it unless a task names it: no search, review, metrics, findings or graph nodes, and never cite it as the current design. [Docs/Conventions/LeadingDot.md](Docs/Conventions/LeadingDot.md)
- Public API headers match the module directory name: `Motor.h`, `StateMachine.h`, `Protocol.h`

## API Design
- **Minimize forwarders.** When a wrapper function is a single-line pass-through to a peer module (`Outer_Foo(p) { return Inner_Foo(&p->inner); }`), it duplicates API surface without adding value. Prefer:
  - **Free-function bridges** — no wrapper type — when coordination surface is small (≤ ~5 functions) and peers are public vocabulary. Precedent: C stdlib (`fprintf`, `memcpy`).
  - **Mediator with peer getters** — wrapper keeps only coordination functions; exposes peers via getters — when coordination points are numerous or the wrapper holds its own state.
  - **Full-forwarder facade** — only when peer types must be hidden from callers (rare).
- Types earn themselves by holding state or enforcing invariants, not by re-exporting another type's API.
- For value-embedded members (`Outer_T { Inner_T Base; }`), forwarders are acceptable to keep inner layout private. Never return a mutable pointer to a value-embedded member.
- **Interface tables point straight at regular-signature functions.** Give the function the table's shape (context first) and cast once at the row (`(OD_GetInfoFn_T)PDO_OdGetInfo`); no static adapter wrappers. A check that needs data the call chain doesn't own goes where that data is used, not threaded through every intermediate signature.
- Architecture notes behind these rules: `Docs/` Principles, Conventions, Patterns, Decisions.

## API Scoping
- **Parameters declare dependencies.** A function takes the narrowest struct that contains every field it reads or writes. `f(Input_T *)` states "reads only config."
- **Multiple access levels per module is expected. Consistency is within a scope, not across scopes.
- **Name by scope when disambiguation helps.** `Motor_Config_DegOfRpm` — the middle identifier says which struct-level. Reserve leading underscore for module-internal helpers, not narrower-scope public API.
- **Promote to a wider scope only on demand.** Add `Motor_DegOfRpm(const Motor_State_T *)` as a thin forwarder only when a real caller has the wider handle often enough that the extra typing matters.
- **Primary structs are exempt for clean ergonomics** e.g. `*_State_T`, `*_Config_T`

## Directory Map & Module Dependencies
See [ARCHITECTURE.md](ARCHITECTURE.md) (mermaid diagrams). Keep its diagrams updated when modules are added or moved.


