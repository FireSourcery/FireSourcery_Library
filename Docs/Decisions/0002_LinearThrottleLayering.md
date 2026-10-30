# 0002 Linear throttle layering

**Status** — accepted 2026-09-26, not yet implemented.

**Context** — A single-axis lever commands forward, backward and braking. Mapping it in Traction
alone forces Run → Passive → Run to reverse, with a coast gap and an exact-zero speed guard. Mapping
it in Motor alone pulls vehicle policy into the motor.

**Decision**

- **Traction interprets the lever**: deadband, curves, and lever policy (brake to stop, or through
  zero). It emits a signed command in the calibrated frame (`Motor_SetTorqueCmd_Norm`,
  `Motor_SetSpeedCmd_Norm`) and never reads the speed sign.
- **Motor owns continuity through zero**: a Run variant where `Direction` tracks the speed sign
  (hysteresis, on `ON_SPEED`), with the anti-plugging VLimit following it. Commands in the motoring
  frame keep `Direction` latched; otherwise they limit-cycle at zero.
- **How much plugging is allowed** (a window around zero) is a Motor detail, added later behind the
  same contract.

**Consequences** — The Traction → Motor interface is a signed calibrated-frame command, so
motor-side anti-plugging changes do not ripple up. The direction policy itself is open in
[0009_DirectionPolicy.md](0009_DirectionPolicy.md).
