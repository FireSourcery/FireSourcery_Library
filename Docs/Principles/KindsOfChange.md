# Kinds of Change

**Statement** — A change is one kind at a time, refactor, repair, extension or optimization, and its
kind decides what it needs, where it goes and how it is verified. A refactor needs no case because
it leaves the code simpler; one that does not is a design change.

**Rationale** — Each kind changes one thing and is verified against what it keeps: a refactor keeps
behavior, a repair keeps the contract, an optimization keeps behavior and shows a measurement. Two
kinds in one change lose both checks: a behavior change inside a restructure passes as a refactor,
and the restructure buries the repair in its diff. A refactor is defined by its result, structure
made easier to understand. One that removes no concept, branch, local or adapter has paid its churn
for nothing, and one that adds a layer is a design change with a case to make. Simplicity is
measured on the resulting design, not on the diff.

The boundaries:

- **Changing the contract is not a repair.** A function that does what its docstring says, wrong
  for some caller, changes for every caller when changed in place: that is an extension, a new name
  or a decision.
- **A result moved by an LSB is a behavior change.** It is a repair where the old result was wrong;
  otherwise the change is not a refactor.
- **An optimization without a measurement** is complexity without a case.
- **A repair is minimal.** The cleanup it suggests is the next change, a refactor.
- **Refactor first**, so the repair or extension that follows is the small change.

**Implications**

| Kind | Changes | Needs | Goes | Verified by |
|---|---|---|---|---|
| Refactor | structure; behavior unchanged | no case, if the result is simpler | anywhere | identical machine code in a consuming board's build, or each difference explained |
| Repair | behavior on inputs the function already takes, back to its contract | the contract it restores | in place, minimal | the failing case passes |
| Extension | new behavior, or inputs no caller bounds | a decision | outside, a wrapper or a name of its own: [OpenClosed.md](OpenClosed.md) | its own case |
| Optimization | resource use; behavior unchanged | a measurement | in place | the measurement, and behavior unchanged |

- Where repaired and added logic go inside a function: [AddingLogic.md](../Conventions/AddingLogic.md).
- "Removing logic needs no case", `CLAUDE.md`, is the refactor row.

**Instances**

- A planned refactor: the var-id rename pass of
  [0012_VarEnumNaming.md](../Decisions/0012_VarEnumNaming.md) keeps values and order, so the image
  is byte-identical on both boards.
- An extension: `_Motor_Electrical_RateOfSi` wraps `angle16_of_arc` with the guard for wire input,
  `Motor/Motor/Motor_Electrical.h`.
- No repair or optimization is recorded yet.

**Source** — The four reasons to change software: adding a feature, fixing a bug, improving the
design, optimizing resource usage (Feathers, *Working Effectively with Legacy Code*, 2004, ch. 1).
Refactoring as a change to internal structure "to make it easier to understand and cheaper to
modify without changing its observable behavior" (Fowler, *Refactoring*). The two hats, refactoring
and adding function never at once (Fowler, after Kent Beck). "Make the change easy, then make the
easy change" (Beck, 2012).
