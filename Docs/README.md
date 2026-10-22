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
| [Modules/](Modules/README.md) | Design notes for one module, at the module's source path | the module's path |

**Placement**

- Our own rules, patterns, decisions and findings live here, tracked with the code they describe.
- A module's `README.md` stays in the module's folder, as its GitHub landing page.
- Drafts keep a leading `_` or `.`, beside their topic.

---

Conventions decided but not yet written up: enum mode over boolean on/off state; time as ticks of
a named clock.
