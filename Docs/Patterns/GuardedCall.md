# Guarded Call

**Level** — idiom.

**Force** — An optional feature is in some builds and not others. The code that implements it should
compile, type-check and stay visible in every build, so a change elsewhere can't break it unseen and
the architecture reads with the feature in place. A build without the feature must not carry it.

**Shape** — compile the definitions in every build. Put the `#if` around the one reference that pulls
them in: the call, the table row, or the body of the entry point. With nothing referencing them,
`-ffunction-sections -fdata-sections` and `--gc-sections` drop the definitions at link, as both
boards build.

```c
/* Motor_FOC.c: in every build */
void Motor_FOC_ProcTorqueFeedback(Motor_T * p_motor) { /* ... */ }

/* The one reference */
static const State_Action_T RUN_ACTION_TABLE[MOTOR_STATE_ACTION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_ON_SPEED] = (State_Action_T)Run_OnSpeed,
#if defined(MOTOR_I_FEEDBACK_ON_PHASE)
    [MOTOR_STATE_INPUT_ON_PHASE] = (State_Action_T)Motor_FOC_ProcTorqueFeedback,
#endif
};
```

A `static inline` with no caller needs no guard at all: it emits nothing.

**In code**
- `Motor_FOC_ProcTorqueFeedback` and `Motor_FOC_IdTorqueReq`, `Motor/Motor/Motor_FOC.c`, unguarded
  since 2026-11-01. The ON_PHASE rows of `RUN_ACTION_TABLE` (`Motor_StateMachine.c`) and of the Torque
  Zero and Speed Zero tables (`StateMachine/Motor_Intervention.c`) are guarded by
  `MOTOR_I_FEEDBACK_ON_PHASE`.
- The electrical calibration chain, `Motor/Motor/StateMachine/Motor_CalibrationElectrical.c`, is
  compiled whole. Only the body of its entry, `Motor_Calibration_StartElectrical`, is guarded by
  `MOTOR_CALIBRATION_ELECTRICAL_ENABLE || !NDEBUG`.
- `Motor_FOC_CaptureI` and `Motor_FOC_TakeCapturedI`, `Motor/Motor/Motor_FOC.h`: `static inline`, no
  guard, no caller yet.
- Checked 2026-11-01 on the KE06 `Release.ld` link: `Motor_FOC_ProcTorqueFeedback`, `Electrical_Proc`,
  `Electrical_Entry` and `CALIBRATION_STATE_ELECTRICAL` are in the map's discarded sections, and
  `m_text` is 49,504 B with or without the unguarded definitions.

**Limit** — the linker drops the symbol, but the compiler has already seen it. A compiled definition
can change inlining in its own translation unit. With `FOC_DECOUPLE_ENABLE` on KE, compiling
`Motor_FOC_ProcTorqueFeedback` made GCC inline `Motor_FOC_AngleControl` into its seven callers:
`m_text` 50,144 → 52,704 B. Check the linked size, not only the discarded symbol. `noinline` on the
shared callee pins it, as `Motor_FOC_TakeI` is.

**Rejected** — guarding the definitions with the call. The code then compiles only in the builds
that enable it: a type or API change elsewhere breaks it unseen, the static analysis and the IDE skip
it, and the module reads as if the feature didn't exist. What the guarded call costs instead is
compile time, which isn't a concern here.

**Source** — none recorded. [ConditionalCompilation.md](../Conventions/ConditionalCompilation.md) has
the rule for when a preprocessor path is used at all.
