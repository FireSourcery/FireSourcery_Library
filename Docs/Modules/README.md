# Modules

Module notes live beside the source they describe, not in this folder: a note on `Peripheral/ADC` is
`Peripheral/ADC/<Topic>.md`, the motor state machine's is `Motor/Motor/Motor_StateMachine.md`, a note
covering a whole stack sits at the stack's root (`Motor/StateMachineOverview.md`, `Motor/ThreadOwnership.md`).
Adopted 2026-11-14 in place of the earlier plan to mirror source paths under `Docs/Modules/`; reversible.

Rules and patterns that apply across modules go in [../Conventions/](../Conventions/) and
[../Patterns/](../Patterns/README.md). A module's `README.md` stays in the module's folder, as its
GitHub landing page. `Motor/Docs/` keeps what is not a module note: `Simulation/`, `Reference/`,
`.Alternative/`, `todo.md`.

Finding a module's notes: `git ls-files '*.md' | grep -v Docs/` lists them; the sheet/detail pair is
`<Topic>.md` + `<Topic>.exp.md`.
