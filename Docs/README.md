# Docs

Architecture notes and findings for the library, in one place. [ARCHITECTURE.md](../ARCHITECTURE.md)
is the map of the code; this folder holds the reasoning behind it.

| Folder | Holds | Entry |
|---|---|---|
| [Principles/](Principles/README.md) | Why: the reasons the conventions, patterns and decisions derive from | `PrincipleName.md` |
| [Conventions/](Conventions/) | Rules followed every time: naming, notation, parameter order, API scoping, fixed point, units | free-form |
| [Patterns/](Patterns/README.md) | Solutions applied when their force is present: architectural, design, idiom | `PatternName.md` |
| [Decisions/](Decisions/README.md) | What we chose and why; binds later work | `NNNN_Title.md` |
| [Findings/](Findings/README.md) | What we measured or learned, dated; the evidence behind decisions | `YYYY-MM-DD_Title.md` |
| [Modules/](Modules/README.md) | The placement rule for module notes: they live beside the source they describe, not here | the rule only |

**Placement**

- Our own rules, patterns, decisions and findings live here, tracked with the code they describe.
- A module's notes live in the module's folder, beside the source they describe (`Motor/Motor/Motor_StateMachine.md` next to `Motor_StateMachine.c`); a stack-wide note at the stack's root (`Motor/StateMachineOverview.md`). A module's `README.md` stays there too, as its GitHub landing page. Adopted 2026-11-14; `Motor/Docs/` keeps simulations, references and the todo.
- Drafts keep a leading `_` or `.`, beside their topic.

---

Time as ticks of a named clock, the per-unit suffixes and SI units at the boundary are in
`Docs/Conventions/FixedPoint.md`, which moves to `Conventions/`.
