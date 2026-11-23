# Conventions

Rules followed every time: naming, notation, parameter order, API scoping, fixed point, units.
A convention keeps a change the size of its reason. Record one once the owner rules it, or once a
third instance follows it.

Each convention is backed by one of two things, and names which:

| Backed by | The rule's authority | A violation costs | Instance |
|---|---|---|---|
| Coordination | that there is one: an alternative followed everywhere would serve as well | a surprise to a reader who expected the shared way | [LeadingDot.md](LeadingDot.md) |
| A principle | the principle, stated in this repo's terms: its convention-side view | something in itself, even to a reader who expected it | [ApiScoping.md](ApiScoping.md), of [InterfaceSegregation.md](../Principles/InterfaceSegregation.md) |

A principle-backed convention links its principle's file. One whose principle has no file yet marks
a principle to write. The coordination sense is Lewis's (*Convention*, 1969): a regularity that
solves a coordination problem, where another would have served had everyone adopted it.

---

## Entry shape

One file per rule or per rule family, named for what it is (`AddingLogic.md`).

A single rule:

```markdown
# <Rule>

**Rule** — the rule, in a sentence; the index quotes it.
**Backed by** — the principle it applies, by link, or coordination.
**Instead of** — what it replaces.
**Instance** — a real instance in code, by path.
**From** — the ruling or the published source, dated.

Then the detail: cases, limits, the reasoning.
```

A rule family — several rules on one subject — is a sheet: one table, each row a rule with what it
replaces and what backs it, as [HardwareBinding.md](HardwareBinding.md) does. An entry written
before this shape converts when next touched.

---

## Written

| File | Rule in one line |
|---|---|
| [AddingLogic.md](AddingLogic.md) | Adding logic to a working function is a decision; what the logic is decides where it goes — in place, an `assert`, or a wrapping function |
| [ApiScoping.md](ApiScoping.md) | A parameter is the narrowest struct containing every field the function reads or writes; scope-named where scopes share a namespace |
| [BooleanState.md](BooleanState.md) | An on/off is the 0 of a value that already bounds the logic, else a value of a mode enum the poll `switch`es on; never a `bool` beside either |
| [Comments.md](Comments.md) | A comment holds only the why the code cannot state; one that restates a declaration is duplicated code |
| [ConditionalCompilation.md](ConditionalCompilation.md) | Variants are named functions compiled in every build; one `#if` selection point only dispatches |
| [ConfigStateRef.md](ConfigStateRef.md) | `Config_T` is declarative persisted data with one writable copy, in the form the device runs on; anything derived is written only by an Init/Reinit |
| [ConfigValidity.md](ConfigValidity.md) | Validity is a config invariant checked once at the config layer; conversions and derives carry no guards |
| [FixedPoint.md](FixedPoint.md) | A stored integer stands for q = n·u/2^k: the type names the representation, the suffix names the unit |
| [HardwareBinding.md](HardwareBinding.md) | The rules for binding a peripheral to the application — HAL, board setup, binding header — each naming what it replaces |
| [InitMacros.md](InitMacros.md) | An `*_INIT` macro earns its place only doing what a designated initializer cannot: cast, size, or derive |
| [LeadingDot.md](LeadingDot.md) | A leading `.` marks a folder or file outside the library proper; tools and reviews skip it |
| [Naming.md](Naming.md) | A name says what the thing is, and a qualifier only refines it. An underscore separates scope segments, case separates the words inside one: `Motor_FOC_ProcAngleV` reads Motor › FOC › ProcAngleV |
