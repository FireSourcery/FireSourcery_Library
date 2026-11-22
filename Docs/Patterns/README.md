# Patterns

How this codebase solves a recurring problem, and when to apply the solution. Each entry is marked
with a level: the three from *Pattern-Oriented Software Architecture* (Buschmann et al., 1996),
plus `Process`, this project's own addition:

| Level | Scope | Example |
|---|---|---|
| Architectural | The structure of the whole system | Pure math layer, const descriptors |
| Design | A component and how it works with others | Equation in a struct, Config / State / Ref |
| Idiom | A shape the C language itself forces | Snapshot before an edge poll, cast at the function pointer |
| Process | How the work itself is carried, not a shape in code | The planned-item brief |

Record a pattern once it is decided and has at least one instance in code.

---

## Entry shape

One file per pattern, named for what it is (`EquationStruct.md`).

```markdown
# <Pattern>

The force and the answer in one sentence; the index's Use when column quotes it.

**Level** — architectural, design, idiom or process.
**Force** — what varies, what must stay still.
**Shape** — the minimal C form.
**In code** — `Name` of each real instance, with its path.
**Rejected** — the alternatives and why.
**Source** — the published form, if any.
```

Names follow the `CLAUDE.md` notation: backticks for anything that exists in code. A mermaid
diagram where the structure is not obvious from the shape.

---

## Written

| File | Level | Use when |
|---|---|---|
| [Descriptor.md](Descriptor.md) | Architectural | What is fixed for the image is a `const` struct the compiler folds at each inlined call; its two forms follow |
| [DescriptorPattern.md](DescriptorPattern.md) | Architectural | A hardware-bound module: descriptor in FLASH, state in RAM, config in NVM, one handle |
| [DescriptorTemplate.md](DescriptorTemplate.md) | Idiom | A generic primitive over many shapes: the descriptor is the template argument, the state passed beside it |
| [EquationStruct.md](EquationStruct.md) | Design | A module computes a published equation every tick: where its coefficients, state, inputs and outputs go |
| [Facets.md](Facets.md) | Architectural | One object serves several kinds of client: one header per concern, each a complete view |
| [ProtocolStack.md](ProtocolStack.md) | Architectural | A wire protocol with a spec-fixed profile and a device integration: generic engine, opaque-context table |
| [WrapperSurface.md](WrapperSurface.md) | Design | An outer module exposes an inner one, or couples two peers: adapt, get or forward, and when a wrapper type earns itself |
| [EdgePollSnapshot.md](EdgePollSnapshot.md) | Idiom | An edge detector wraps a poll that writes what it compares: snapshot the prior value first |
| [EnumNamedValues.md](EnumNamedValues.md) | Idiom | An enum classifies a quantity: its enumerators are the quantity's values (range bounds, or a signed level), so the conversion is a mask, sign or `abs` |
| [GuardedCall.md](GuardedCall.md) | Idiom | An optional feature: compile its definitions in every build, guard the one reference, and the linker drops them |
| [HwSequencedScan.md](HwSequencedScan.md) | Design | A Hw trigger converts one of several channel sets and a transfer copies them: sets as records, the selection in the library, one completion ISR |
| [WriterOwns.md](WriterOwns.md) | Design | A value crossing threads is held by the module whose thread writes it, with the only mutable path; a module on another thread holds a `const` pointer, `volatile` where the write is asynchronous to its reads |
| [ParameterizedFunction.md](ParameterizedFunction.md) | Idiom | A parameter earns its place in two ways: on the API it trims the surface, one function over the values of one body; behind the API it lets internals reuse one implementation, its optional slots never reaching a caller. A parameter that picks which logic runs is a flag, and its variants stay separate functions |
| [PlannedItemBrief.md](PlannedItemBrief.md) | Process | A block is deferred: one self-contained brief per block — context, a drawn design, a code sketch, and the owner's boxes — that seeds the implementation |

---

## Candidates

Decided, not yet written up in this shape:
- Typed callback parameters, one cast at registration (idiom): notes in [FunctionPointerCasting.md](FunctionPointerCasting.md)
