# 0006 Ramp limits act on the input

**Status** — accepted 2026-10-17.

**Context** — When a current limit drops, the torque ramp's output can sit outside the new bounds.
Clamping the ramp output in `_Motor_ApplyILimits` would land a drop within one tick, as ABB and
CiA 402 do by clamping the demand after the ramp.

**Decision** — `Ramp_SetLimits` sets bounds only. `Ramp_ProcNextOf` clamps the input each step, and
an output outside new bounds returns at the ramp slope (`Math/Ramp/Ramp.h`: "Output is not clamped,
an out of bounds output ramps back in"). `Ramp_SetOutputLimits` exists for an explicit output clamp.

**Consequences**

- A protective drop reaches the current loop at the ramp slope: a charge limit 100 → 10 A takes
  112 ms at 800 A/s, with the OV fault as the backstop.
- A source that needs a faster drop shapes it at the source, as a separate question.
- Clamping the output as well as the input does not change the stall limit cycle (simulated
  2026-10-17).
- Open inconsistency, not decided: `Accumulator_SetOutput`, used by `Ramp_SetOutputState` when
  entry matching, still clamps to the limits.
