# Leading dot

A folder or file whose name starts with `.` is outside the library proper. Tools and reviews skip
it unless it is named.

| Name | Is | Example |
|---|---|---|
| `.Name/`, `.name`              | outside the library proper: a sketch of ideas, retired code, tool data | `Docs/.Var/`, `Motor/Motor/.bkSixStep/`, `.vscode/`, `.clang-format` |
| `_*.md`                        | a draft note, outside as well                                           | `Peripheral/ADC/_.md` |
| `_Name.h`, `_Name.c`, `_Name/` | private to its module, and live                                         | `_StateMachine.h` |
| any other name                 | the library                                                             | `Motor/Motor/Motor.h` |

## Skipped unless named

| | A dot path or a draft note |
|---|---|
| Graph views                    | left out |
| Search, review, audit, metrics | not read, not counted |
| Findings                       | none raised against it |
| Docs and comments              | not cited as the current design |
| Read                           | when a task names the path, or a live file links to it |

## Rules

1. **The name is the marker.** No list is kept. A sketch or retired code sits beside its topic:
   `Motor/Motor/.bkSixStep/` next to the code that replaced it.
2. **A dot folder covers everything under it.**
3. **Live code does not include a dot path.** A sketch may include live headers.
4. **A file in a live folder holds live code.** A file whose body is commented out, or that holds
   `#include` lines only without being an include or selector header, is retired: move it under a
   dot folder beside its topic, or delete it. Git keeps what is deleted.
5. **Each tool carries the same two lines**, in gitignore syntax, where a name without a slash
   matches at any depth.

   | Tool | File | Lines |
   |---|---|---|
   | Understand-Anything | `.ua/.understandignore`                           | `.*`, `_*.md` |
   | graphify            | `excludes` in `graphify-out/.graphify_build.json` | `.*`, `_*.md` |

   ripgrep skips hidden paths by default, so a plain search already leaves dot paths out.

## Not decided

- **Tests.** `.Test/` carries the dot, so it is outside for now. How tests become part of the
  library is open: [finding G1](../Findings/2026-11-17_KnowledgeGraphOpenFindings.md#5-the-graph-itself).

## Not taken

| Option | Why not |
|---|---|
| Two fixed folder names, `.sketch/` and `.bk/` | every existing folder renamed, for a distinction no tool needs |
| A list file at the repo root                  | has to be kept current, and nothing in a path's name says it is excluded |
| `_` as the marker                             | already means private and live on a source file |
