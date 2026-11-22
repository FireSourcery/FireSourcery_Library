# Boolean Blindness

**Statement** — A `bool` says that something holds, not what: an on/off is carried by whatever
already has the meaning, never by a flag beside it. That is the value it gates, at its degenerate
0; a value of the mode or state enum it belongs to; or, for a parameter, the name of the function
it would select.

**Rationale** — A `bool` beside a value or a state is a second field for one concept. The pair has
states it can disagree in: enabled with a 0 limit, disabled with a nonzero one, `IsActive` set
while the mode says disabled. Each consumer branches on the bit, and the bit no longer says which
value or state it gates. As a parameter the same bit is opaque at the call site, `Foo_Set(p, true)`,
and puts two functions behind one name: the caller's choice is made twice, once as a literal and
once as a branch inside. Carried by the value, the mode or the name, the meaning stays where the
consumer reads it.

The limit: a `bool` that is itself the datum is a value, not a flag. A pin level, a register bit
or a sampled input selects no behavior; it is what is written or read (`HAL_Pin_WriteOutput`,
`Debounce_Poll`). A predicate returns a `bool` by nature: the principle governs what is stored and
passed, not what a question answers.

**Implications**

| The on/off | Carried by | Rule |
|---|---|---|
| A value already bounds the logic | the value's 0, its degenerate case | [ValueOverBool.md](../Conventions/ValueOverBool.md) |
| No value carries it | a value of an existing mode or state enum, else a new enum with `DISABLED = 0` | [ModeOverBool.md](../Conventions/ModeOverBool.md) |
| A parameter selects behavior | two functions, each named for its behavior | "Don't pass bool", `CLAUDE.md`; [AddingLogic.md](../Conventions/AddingLogic.md), a different behavior |
| A caller asks | a derived predicate, `*_IsEnabled`, never a stored field | [ValueOverBool.md](../Conventions/ValueOverBool.md) |

- The sign in a draft: a struct gaining an `Is*` field beside a value or a mode; a
  `*_Set*(…, bool isEnabled)`.
- Rejected instances: `RangeMonitor_T.IsActive`, `VBus_T.IsMonitorPaused`.

**Instances**

- `RangeMonitor_Mode_T {DISABLED, PAUSED, ACTIVE}` replaced `RangeMonitor_T.IsActive`:
  `Transducer/Monitor/Base/RangeMonitor.h`.
- `Monitor_T.Direction` carries `MONITOR_DISABLED` as one more direction, and `Monitor_IsEnabled`
  derives the question: `Transducer/Monitor/Base/Monitor.h`.
- `Motor_FieldWeakening_IsEnabled` is `IdLimit > 0`: `Motor/Motor/Motor_Electrical.h`.
  `FOC_DisableFieldWeakening` zeroes the state copy: `Motor/Motor/Math/FOC.h`.
- `TIMER_MODE_DISABLED` is a timer mode, and `_TimerT_Disable` and `_TimerT_Stop` are functions of
  their own, not a flag on one: `Framework/Timer/_TimerT.h`.

**Source** — "Boolean blindness" (Harper, *Existential Type*, 2011): a `bool` is a bit cut off from
the proposition it tested. "Make illegal states unrepresentable" (Minsky, *Effective ML*, Jane
Street) for the state half, the pair that can disagree. Flag Argument (Fowler, 2011) and Remove
Flag Argument (Fowler, *Refactoring*, 2nd ed., 2018) for the parameter half.
