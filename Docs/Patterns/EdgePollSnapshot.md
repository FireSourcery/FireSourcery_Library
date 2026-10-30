# Edge Poll Snapshot

**Level** — idiom.

**Force** — An edge or change detector wraps a poll that writes the state it compares against. The
one-line form `return (p->X != Module_Poll(p));` reads `p->X` in one operand and writes it through
the call in the other. The operands of `!=` are unsequenced (C23 §6.5p2), so the order is undefined.
Parentheses add no sequence point. GCC on ARM loads before the call today; inlining, LTO or
`restrict` can flip it on any rebuild.

**Shape** — snapshot the prior value into a local first. `=` ends a full expression, so the read is
ordered before the mutation.

```c
static inline bool Module_PollEdge(Module_T * p_module)
{
    bool prev = p_module->Output;
    return (prev != Module_Poll(p_module));
}
```

The same rule covers any operator whose operands read a scalar on one side and write it through a
call on the other: `==`, `!=`, arithmetic, relational, bitwise, and function arguments. `&&`, `||`,
`?:` and `,` are sequence points and need no snapshot.

**In code** — `Hysteresis_PollEdge` in `Math/Hysteresis/Hysteresis.h`,
`Traction_Input_PollCmdEdge` in `Motor/MotorController/Traction/`, `PulseTimer_CaptureSampleTk` in
`Transducer/Pulse/PulseTimer.h`. All three were fixed in 2026-05.

**Rejected** — the one-line form. A declarative one-liner is preferred until it is undefined.

**Source** — ISO C23 §6.5p2, unsequenced side effects; Annex C, sequence points.
