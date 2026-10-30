# 0004 PI anti-windup: band-clipped sliding clamp

**Status** — accepted 2026-10-23, after a hardware test.

**Context** — `CalcPI` in `Math/PID/PID.c` clamps the integral. Two forms were compared:

- **Anchored** (recommended 2026-10-16): bounds anchored at the accumulator,
  `math_min/max(IntegralAccum, ·)`, so the clamp never moves the integral against the error.
- **Sliding**: the band `[OutputMin − P, OutputMax − P]`, clipped to the output range.

On hardware, sliding was much more stable during field weakening. Why the anchored form fails there
(Base build, `FOC_DECOUPLE_DISABLE`):

- `PidIq`'s ceiling is √(vCircle² − Vd²). |Vd| ≈ ω·Lq·Iq grows with speed and FW depth, so the
  ceiling falls every tick. The anchor held I above the falling ceiling, a peak-hold on ceiling − P
  under ripple.
- Vq stayed pinned at the ceiling even at zero Iq error, so |V| = radius > Vfw (0.95 · radius).
  `FOC_ProcIdFieldWeakening` kept integrating IdFw negative until Iq overshot and unwound the hidden
  excess at Ki · e per tick: two integrators coupled through a dead zone, a limit cycle.
- The FW operating point has the Iq loop unsaturated with a 5 % margin. Only the band-consistent
  integral (P + I = u_sat) leaves the ceiling on the tick the error falls.
- The 2026-10-16 decoupling simulation missed this: it had no FW loop reading |V|, and mean iq
  cannot see a pinned Vq.

**Decision** — the sliding form:

```c
integralMin = math_clamp(OutputMin − P, OutputMin, OutputMax) << 15;
integralMax = math_clamp(OutputMax − P, OutputMin, OutputMax) << 15;
```

The anchored form stays commented out in `PID.c`. Do not reintroduce it.

**Consequences**

- Costs of instant back-calculation (Tt → 0): a |P| wider than the band drags I to the opposite
  edge, so the output recovers from the wrong edge; a transient band dip erases the integral, which
  recovers at the Ki rate.
- If either shows up, the option is finite-rate back-calculation, with Tt a power-of-two tick count
  (a shift). It follows persistent band shifts and barely moves on short dips. Not implemented.
- Kelly_KMC's untracked `Test/test_PID` still asserts the anchored property: NoPushAgainstError,
  HoldsIntegral, NoOverflow and LimitDip fail; CalcPI, the gain tests and Invariants pass. Build
  with host clang `-fgnuc-version=4.2.1`.
- Unchanged: `PID_SetOutputLimits` pulls the integral into range, `PID_CaptureOutputLimits` leaves
  it to the clamp. Gains are `fract16e_t Kp, KiPerTick`. `FOC_ProcIFeedback_BackLimit` has no
  callers.
