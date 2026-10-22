# Patterns

How this codebase solves a recurring problem, and when to apply the solution. Each entry is marked
with one of the three levels from *Pattern-Oriented Software Architecture* (Buschmann et al., 1996):

| Level | Scope | Example |
|---|---|---|
| Architectural | The structure of the whole system | Pure math layer, const descriptors |
| Design | A component and how it works with others | Equation in a struct, Config / State / Ref |
| Idiom | A shape the C language itself forces | Snapshot before an edge poll, cast at the function pointer |

Record a pattern once it is decided and has at least one instance in code. Before adopting a new
one, apply the adoption gate in `E:\Workspace\Project\md\references\README.md`.

---

## Entry shape

One file per pattern, named for what it is (`EquationStruct.md`).

```markdown
# <Pattern>

**Level** — architectural, design or idiom.
**Force** — what varies, what must stay still.
**Shape** — the minimal C form.
**In code** — `[Name]` of each real instance, with its path.
**Rejected** — the alternatives and why.
**Source** — the outside precedent: an extract in `md\`, or the original in `Ref\`.
```

Names follow the `CLAUDE.md` notation: `[Name]` for anything defined in the codebase,
`` `code` `` for literal code. A mermaid diagram where the structure is not obvious from the shape.

---

## Written

| File | Level | Use when |
|---|---|---|
| [EquationStruct.md](EquationStruct.md) | Design | A module computes a published equation every tick: where its coefficients, state, inputs and outputs go |
| [Facets.md](Facets.md) | Architectural | One object serves several kinds of client: one header per concern, each a complete view |

---

## Candidates

Decided, not yet written up in this shape:

- Const descriptors (architectural) — `../Conventions/DescriptorPattern.md`
- Config / State / Ref partitions (design) — `../Conventions/Config_State_Ref.md`
- Collaborator (design) — `../Conventions/Patterns.md`
- Generics (design) — `../Conventions/Generics.md`
- Cast at the table row (idiom) — `../Conventions/FunctionPointerCasting.md`
- Snapshot the prior state before an edge poll writes it (idiom)
- Typed callback parameters, one cast at registration (idiom)
- Init macros, kept only when they cast, allocate, derive, or build a constant expression (idiom)
