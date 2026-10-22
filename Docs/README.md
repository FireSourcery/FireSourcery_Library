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
- Sources stay outside the repo, in two tiers:
  - `E:\Workspace\Project\Ref\`: the originals, as published: vendor PDFs, books, standards, raw
    text dumps, reference projects.
  - `E:\Workspace\Project\md\`: our processed version of what we pull from `Ref\`: page-cited
    extracts, transcribed equations, condensed listings.
  Notes here cite them by path and page.
- Product material goes in `Kelly_KMC/Docs`: boards, customer documents, bench experiments.
- A module's `README.md` stays in the module's folder, as its GitHub landing page.
- Drafts keep a leading `_` or `.`, beside their topic.

---

## Not moved yet

| Now | Goes to |
|---|---|
| `Conventions/DescriptorPattern.md`, `Config_State_Ref.md`, `Patterns.md`, `Generics.md`, `FunctionPointerCasting.md` | `Patterns/` |
| `Motor/Docs/FixedPoint.md` | `Conventions/` |
| `Motor/Docs/*.md`, `Framework/StateMachine/Architecture.md`, `Peripheral/Notes.md`, `Peripheral/ADC/Notes.md`, `Transducer/Encoder/Encoder_Reference.md` | `Modules/`, at the same path |
| `Peripheral/ADC/ADC_Batch_JoinVariants.md` | `Decisions/`, proposed |
| PDFs in `Datasheets/`, `References/`, `Motor/Docs/Reference/` | `E:\Workspace\Project\Ref\`; then `References/` can go |

Conventions decided but not yet written up: enum mode over boolean on/off state; time as ticks of
a named clock.
