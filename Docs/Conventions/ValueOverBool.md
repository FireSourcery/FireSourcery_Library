# Value over Boolean

An on/off that a value parameter already decides is that value, not a `bool` beside it. The off
value is the parameter's degenerate case: given it, the unchanged logic produces the off behavior.

**Backed by** — [BooleanBlindness.md](../Principles/BooleanBlindness.md).

- **A value first, then a mode.** A limit, gain or rating of 0 is the off where the logic clamps or
  scales by it. Where no value carries the on/off, it is a mode: [ModeOverBool.md](ModeOverBool.md).
- **The question is a derived function, never a stored field.** A caller that asks reads
  `Motor_FieldWeakening_IsEnabled`, `IdLimit > 0`.
- **Each tier bounds the next by `min`, so a 0 at any tier is the off.** `Motor_FieldWeakening_IdLimit`
  = min(`IfwLimit`, `Phase_IRatedFw_Pu()`): the board rating's 0 disables for the board, the
  user's 0 for the motor. The feature macro compiles the logic in or out; the values stay in Config.
- **A rule from another Config value writes the Config value.** Sensorless selected ⇒ `IfwLimit` = 0,
  `Motor_Config_ValidateFwSensorMode`. It is a repair ([ConfigValidity.md](ConfigValidity.md)),
  applied where Config loads (`Motor_Reset`, before `Motor_FOC_InitParams` derives) and on both
  writes (`Motor_Config_SetSensorMode`, the `MOTOR_VAR_FW_ID_LIMIT` var). It persists on the next
  NVM save, and the app reads the 0 that runs.
- **A runtime hold writes the state copy, never Config.** `FOC_DisableFieldWeakening` zeroes
  `FOC_T.FieldWeakening.IdLimit` until the next derive ([Config_State_Ref.md](Config_State_Ref.md)).
- **A value that needs a branch is a sentinel, not a degenerate case.** The branch stays in the one
  derive that reads it: `_Motor_VAlignRsLimit`, where Rs 0 is unset and leaves VAlign unbounded.

`IfwLimit` = 0, read by unchanged logic:

| Reads it | Given 0 |
|---|---|
| `FOC_ProcIdFieldWeakening`, `FOC_MatchIdFieldWeakening` | `IdFw` clamps to [−0, 0]: no demagnetizing current |
| `Motor_FieldWeakening_IqLimit`, in `Motor_Config_ValidateFw` | √(Ipeak² − 0²) = Ipeak: the motoring limit keeps the full rated vector |
| `Motor_FieldWeakening_IsEnabled`, in `_Motor_SpeedLimitCeiling`, `Motor_IsConfigValid`, `Motor_ValidateConfig` | the speed ceiling falls to rated |

Cost: the one value holds either the tuning or the off. Disabling discards the tuned value, and
re-enabling means entering it again. A mode fits where the tuning must survive the off state.

Why: a `bool` beside the value is a second field for one concept, a branch in each consumer, and two
states the pair can disagree in: enabled with a 0 limit, disabled with a nonzero one. Published
analogue: Null Object (Woolf, *Pattern Languages of Program Design 3*, 1998) and Special Case
(Fowler, *Patterns of Enterprise Application Architecture*, 2002), a case that runs through the
normal path in place of a check.
