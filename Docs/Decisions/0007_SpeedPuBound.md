# 0007 Speed per-unit bounded at 1.0

**Status** — accepted 2026-10-12.

**Context** — The speed base derives from the controller V_MAX: ω_pu = 1.0 where the phase-peak back
EMF is V_MAX / k (k = 2 today). Field weakening starts below 1.0 whenever VBus < V_MAX: a 72 V bus on
a 94 V base starts it near 0.885.

**Decision** — ω_pu never exceeds 1.0. That is the bound of the speed PID input, so the field
weakening range runs inside the same speed PID. `Speed_Pu` saturation is the intended bound, not an
error.

**Consequences**

- No design argues from operation above ω_pu = 1.0.
- The bound also keeps the uncontrolled rectified line-line back EMF ≤ √3 / k · V_MAX.
- The unused top of the shared V_MAX voltage scale (phase ≤ VBus / √3, OmegaPsi ≤ 1 / k) is range,
  not resolution.
- Still worth flagging: inconsistent bounds between sensors. Hall and encoder reach 2.0, sensorless
  and SinCos stop at 1.0.
