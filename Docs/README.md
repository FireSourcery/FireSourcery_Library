# Docs

Architecture notes and findings for the library, in one place. [ARCHITECTURE.md](../ARCHITECTURE.md)
is the map of the code; this folder holds the reasoning behind it.

| Folder | Holds | Entry |
|---|---|---|
| [Principles/](Principles/README.md) | Why: the reasons the conventions, patterns and decisions derive from | `PrincipleName.md` |
| [Conventions/](Conventions/README.md) | Rules followed every time: naming, notation, parameter order, API scoping, fixed point, units | `RuleName.md` |
| [Patterns/](Patterns/README.md) | Solutions applied when their force is present: architectural, design, idiom | `PatternName.md` |
| [Decisions/](Decisions/README.md) | What we chose and why; binds later work | `NNNN_Title.md` |
| [Findings/](Findings/README.md) | What we measured or learned, dated; the evidence behind decisions | `YYYY-MM-DD_Title.md` |
| [Modules/](Modules/README.md) | The placement rule for module notes: they live beside the source they describe, not here | the rule only |

**Entry contract** — every entry, in every category:

- The H1 is the name. The first paragraph under it is one sentence that stands alone, and the
  category index quotes it verbatim, so entry and index cannot drift. Where the shape's first
  field is itself the claim in a sentence — a principle's **Statement**, a convention's **Rule** —
  that field is the summary.
- Fields are bold labels, `**Label** — text`, set once in the category README and named by the
  category's published source. A new category derives its fields from the general form — the
  claim, its grounds, where it lands in code, its provenance — and names its source, or says none.
- Grounded: an instance in code by path, or the evidence a decision rests on.
- Provenance: the published source, or the owner's ruling with its date.

An entry written before this contract gains its summary line when next touched. Entries follow
this repo's code style, `PascalCase.md`. No YAML
frontmatter: the folder is the type, the filename the date or number, the first bold line the
status — frontmatter would restate all three. Revisit if these docs gain a machine consumer (an
Obsidian overlay, generated indexes); the summary line is the `description` property, positionally.

**Placement**

- Our own rules, patterns, decisions and findings live here, tracked with the code they describe.
- A module's notes live in the module's folder, beside the source they describe (`Motor/Motor/Motor_StateMachine.md` next to `Motor_StateMachine.c`); a stack-wide note at the stack's root (`Motor/StateMachineOverview.md`). A module's `README.md` stays there too, as its GitHub landing page. Adopted 2026-11-14; `Motor/Docs/` keeps simulations, references and the todo.
- A leading `.` on a folder or file marks it outside the library proper: a sketch, retired code, tool data. A draft note is `_*.md`. Both stay beside their topic, and tools and reviews skip them: [Conventions/LeadingDot.md](Conventions/LeadingDot.md).

---

Time as ticks of a named clock, the per-unit suffixes and SI units at the boundary are in
`Docs/Conventions/FixedPoint.md`, which moves to `Conventions/`.
