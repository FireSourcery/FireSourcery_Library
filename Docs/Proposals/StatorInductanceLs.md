# StatorInductanceLs

Settle what `Motor_Ls_PuTurn` means: the αβ average (Ld + Lq) / 2 it returns today, or the per-phase winding self-inductance.

**Status**: deferred. This comes from the owner's Motor todo (2026-11-17) and was moved here 2026-11-22. Nothing
reads Ls today, so nothing waits on this. It unblocks when some consumer needs Ls: a parameter readback, an
identification routine, or a front end other than the EEMF observer. The outcome lands in
[Motor_Electrical.h](../../Motor/Motor/Motor_Electrical.h) and
[motor_electrical_math.md](../../Motor/Motor/Math/motor_electrical_math.md).

## Context

- `Motor_Ls_PuTurn` ([Motor_Electrical.h:99](../../Motor/Motor/Motor_Electrical.h#L99)) returns (Ld + Lq) / 2,
  computed from `Motor_Ld_PuTurn` and `Motor_Lq_PuTurn`. Line 100 records the alternative in a comment.
- The todo names `Motor_Electrical_LsTick`, read by `FOC_Sensorless_InitG`. Both names are gone. The observer now
  takes Lq on both axes, the active-flux EMF:
  - the call is `EmfObserver_InitWinding(..., Motor_Lq_PuTurn(...))`
    ([Sensorless_Sensor.c:118](../../Motor/Motor/Sensor/Sensorless/Sensorless_Sensor.c#L118))
  - the reasoning is "Lq on both axes" in [Sensorless.exp.md](../../Motor/Motor/Sensor/Sensorless/Sensorless.exp.md)
  - [Stator.h:63](../../Motor/Motor/Model/Stator.h#L63) notes that Ls = (Ld + Lq) / 2 leaves a 2θ error

  Nothing reads `Motor_Ls_PuTurn`.
- The config stores only `AngleFreqPerOhmD` = 1 / Ld and `AngleFreqPerOhmQ` = 1 / Lq
  ([Motor_Electrical.h:67](../../Motor/Motor/Motor_Electrical.h#L67)). No per-phase value is stored or measured.
- [motor_electrical_math.h:253](../../Motor/Motor/Math/motor_electrical_math.h#L253) says Ls = Ld = Lq for an SPM,
  and Ld ≠ Lq for an IPM.

## Design

The two definitions, in the reference-frame model. Here L_ls is the leakage inductance, L_A the mean magnetizing term
and L_B its 2θ term. Ld = L_ls + 3/2 · (L_A − L_B) and Lq = L_ls + 3/2 · (L_A + L_B).

| Definition | Expression | Measured as | Differs from the other by |
|---|---|---|---|
| αβ average (today) | (Ld + Lq) / 2 = L_ls + 3/2 · L_A | Half the line-to-line inductance (SPM) | |
| Phase self-inductance, abc | mean of L_aa = L_ls + L_A | One phase to neutral; needs access to the neutral | L_A / 2, the mutual term |

## Sketch

```c
/* today: the αβ average */
static inline uint32_t Motor_Ls_PuTurn(const Motor_Electrical_T * p_electrical) { return ((uint64_t)Motor_Ld_PuTurn(p_electrical) + Motor_Lq_PuTurn(p_electrical)) / 2U; }
```

The self-inductance definition cannot be derived from Ld and Lq alone. It would need its own stored rate, such as an
`AngleFreqPerOhm` for the phase.

## Done already

- The observer reads Lq, not Ls (see Sensorless.exp.md, "Lq on both axes").

## Open for you

**What `Ls` names**

- [ ] the αβ average (Ld + Lq) / 2, as today: the per-phase equivalent, half the line-to-line inductance
- [ ] the phase self-inductance in abc, with a stored value and a measurement through the neutral
- [ ] both, under two names

- Notes: `____`
