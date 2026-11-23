# Boolean State

An on/off is the 0 of a value that already bounds the logic, else a value of a mode enum the poll
`switch`es on; never a `bool` beside either. The rules, in the order they are tried:

| # | Rule | Replaces | Backed by |
|---|---|---|---|
| 1 | A value that already bounds the logic carries the on/off as its 0, its degenerate case | an enable `bool` beside a limit, gain or rating | [BooleanBlindness.md](../Principles/BooleanBlindness.md) |
| 2 | Where no value carries it, or the tuning must survive the off, extend an existing mode, direction or state enum | a flag beside the state | [BooleanBlindness.md](../Principles/BooleanBlindness.md) |
| 3 | Otherwise add a mode enum with `DISABLED = 0`, so a zero-initialized struct is safe | a single-purpose `bool` | [BooleanBlindness.md](../Principles/BooleanBlindness.md), [EnumNamedValues.md](../Patterns/EnumNamedValues.md) |
| 4 | Transitions are explicit, and `DISABLED` is sticky | a mode any caller sets | |
| 5 | The question is a derived predicate, never a stored field | an `Is*` field | [BooleanBlindness.md](../Principles/BooleanBlindness.md) |
| 6 | A value that needs a branch is a sentinel, not a degenerate case; the branch stays in the one derive that reads it | a branch at each consumer | [SimplicityFavorsRegularity.md](../Principles/SimplicityFavorsRegularity.md) |
| 7 | A runtime hold writes the state copy, never Config | a Config write, persisted on the next NVM save | [ConfigStateRef.md](ConfigStateRef.md) |
| 8 | A rule from another Config value writes the Config value, where Config loads and on each write | a check of the other value in each consumer | [ConfigValidity.md](ConfigValidity.md) |

---

## Value: rule 1

A limit, gain or rating of 0 is the off where the logic clamps or scales by it: given 0, the
unchanged logic produces the off behavior.

- **Each tier bounds the next by `min`, so a 0 at any tier is the off.** `Motor_FieldWeakening_IdLimit`
  = min(`IfwLimit`, `Phase_IRatedFw_Pu()`): the board rating's 0 disables for the board, the
  user's 0 for the motor. The feature macro compiles the logic in or out; the values stay in Config.

`IfwLimit` = 0, read by unchanged logic:

| Reads it | Given 0 |
|---|---|
| `FOC_ProcIdFieldWeakening`, `FOC_MatchIdFieldWeakening` | `IdFw` clamps to [−0, 0]: no demagnetizing current |
| `Motor_FieldWeakening_IqLimit`, in `Motor_Config_ValidateFw` | √(Ipeak² − 0²) = Ipeak: the motoring limit keeps the full rated vector |
| `Motor_FieldWeakening_IsEnabled`, in `_Motor_SpeedLimitCeiling`, `Motor_IsConfigValid`, `Motor_ValidateConfig` | the speed ceiling falls to rated |

Cost: the one value holds either the tuning or the off. Disabling discards the tuned value, and
re-enabling means entering it again. Where the tuning must survive the off state, the on/off is a
mode: rule 2.

Published analogue: Null Object (Woolf, *Pattern Languages of Program Design 3*, 1998) and Special
Case (Fowler, *Patterns of Enterprise Application Architecture*, 2002), a case that runs through
the normal path in place of a check.

## Mode: rules 2–4

A gate, enable or pause on a state struct is a value of an enum mode that the poll `switch`es on.
One enum keeps the transitions in one switch and leaves room for the next mode.

- **An existing enum first.** `Monitor_T.Direction` carries `MONITOR_DISABLED` as one more
  direction. `_TimerT_Modal_Poll` has `TIMER_MODE_DISABLED` and `TIMER_MODE_STOPPED` as states.
- **Else a new one, `DISABLED = 0`.** `RangeMonitor_Mode_T {DISABLED, PAUSED, ACTIVE}` replaced
  `RangeMonitor_T.IsActive`. `UserDIn_Mode_T` is a prepared mode enum: two live values and
  commented future modes.
- **Explicit transitions.** `RangeMonitor_InitFrom` resolves `DISABLED` or `ACTIVE` from the Config
  and sets the mode last; Pause and Resume move only `ACTIVE` ↔ `PAUSED`, so `DISABLED` is sticky.
  The state diagram is in [ConfigStateRef.md](ConfigStateRef.md), Config interface.

## Questions and sentinels: rules 5–6

- A caller that asks reads a predicate over the value or the mode: `Motor_FieldWeakening_IsEnabled`
  is `IdLimit > 0`, `Monitor_IsEnabled` is `Direction != MONITOR_DISABLED`.
- `_Motor_VAlignRsLimit`: Rs 0 is unset and leaves VAlign unbounded. The branch is in that one
  derive, not at each reader of Rs.

## Holds and Config rules: rules 7–8

- **A runtime hold.** `FOC_DisableFieldWeakening` zeroes `FOC_T.FieldWeakening.IdLimit` until the
  next derive. `Modal_Disable` overrides `P_STATE->Mode` without persisting. `PAUSED` is
  `RangeMonitor_T`'s hold.
- **A rule from another Config value.** Sensorless selected ⇒ `IfwLimit` = 0,
  `Motor_Config_ValidateFwSensorMode`. It is a repair ([ConfigValidity.md](ConfigValidity.md)),
  applied where Config loads (`Motor_Reset`, before `Motor_FOC_InitParams` derives) and on both
  writes (`Motor_Config_SetSensorMode`, the `MOTOR_VAR_FW_ID_LIMIT` var). It persists on the next
  NVM save, and the app reads the 0 that runs.

Rejected instances: `RangeMonitor_T.IsActive`, `VBus_T.IsMonitorPaused`.
