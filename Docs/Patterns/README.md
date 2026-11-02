# Patterns

How this codebase solves a recurring problem, and when to apply the solution. Each entry is marked
with one of the three levels from *Pattern-Oriented Software Architecture* (Buschmann et al., 1996):

| Level | Scope | Example |
|---|---|---|
| Architectural | The structure of the whole system | Pure math layer, const descriptors |
| Design | A component and how it works with others | Equation in a struct, Config / State / Ref |
| Idiom | A shape the C language itself forces | Snapshot before an edge poll, cast at the function pointer |

Record a pattern once it is decided and has at least one instance in code.

---

## Entry shape

One file per pattern, named for what it is (`EquationStruct.md`).

```markdown
# <Pattern>

**Level** — architectural, design or idiom.
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
| [GuardedCall.md](GuardedCall.md) | Idiom | An optional feature: compile its definitions in every build, guard the one reference, and the linker drops them |

---

## Candidates

Decided, not yet written up in this shape:
- Typed callback parameters, one cast at registration (idiom): notes in [FunctionPointerCasting.md](FunctionPointerCasting.md)
