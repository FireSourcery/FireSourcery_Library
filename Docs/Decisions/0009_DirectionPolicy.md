# 0009 Direction and the VLimit policy

**Status** — proposed. Deferred 2026-10-17: keep `Direction` as a selectable policy now; finish the
end state later, probably a symmetric VLimit plus a window.

**Context** — Today `Direction` is a direction-lock policy, and `FOC_SetVLimits` writes the matching
anti-plugging half-plane. During rollback, forward torque needs v_q = R · i_q + E with E < 0. Once
|E| > R · i_q the PI pins v_q at 0, and the current is |E| over the winding impedance (90 mΩ: 18 V →
200 A at low speed), outside every I limit. A symmetric VLimit keeps the limits enforced; anti-plugging
then becomes command policy: Traction holds the gear, brakes against the motion and changes direction
at zero speed.

State on 2026-10-17:

- `Motor_Context_T.Direction` is policy, not derived state. `Motor_SetDirection` writes the field and
  the half-plane; every reader goes through `Motor_Direction()`, so the source can move in one place.
- Hall `Calibration_End` resets VLimit to NULL, because its entry aligns in the forward half-plane.

**Decision (proposed, 2026-10-18)** — no policy enum; the setter called is the policy.

- The loop composes `clamp_each(circle ∩ VLimit, e_q ± VWindow)` every cycle. `VWindow =
  FRACT16_MAX` is the identity, so nothing re-centers per tick.
- The band must clamp each bound into the circle (`interval_clamp_each`), not intersect: above base
  speed (|E| > V_bus / 2 + w) the window misses the circle, and an intersection gives low > high,
  which `PID_CaptureOutputLimits` rejects.
- The window replaces the half-plane; beyond |e| > w their intersection is empty.
- `Direction` is the half-plane's sign; a symmetric VLimit reads NULL, which is correct (no lock).
- Window sizing: w = 1.5 · R · I (27 V at 90 mΩ, 200 A) blocks reverse voltage only above |E| > w,
  about 37 % speed; below that, plug braking is allowed within the limits. |i_q| ≤ w / |Z| ≈ 300 A
  holds even with the PI saturated. w must cover the ψ error × top speed.
- Centering on OmegaPsi is free in the Base build (`FOC_DECOUPLE` off) but omits ω · L_d · i_d;
  `FOC_VqFeedforward` costs one multiply-add per cycle.

**Before a symmetric or window VLimit is usable**, the readers that are not the lock must change:

- Arming gates (Passive → Run, OpenLoop StartUp, Sensorless start) test `Motor_Direction() != NULL`;
  they must test that VLimit has width, or a run command. OpenLoop and Sensorless start keep
  requiring a direction.
- Mot/Gen key: `Direction` if locked, else the speed sign with a hysteresis deadband and symmetric
  motoring at standstill. The Gen default becomes a braking-torque limit; battery-charge protection
  belongs in `IChargeLimit` ([0005_ILimitFrames.md](0005_ILimitFrames.md)).
- TorqueZero's `_Motor_GeneratingOnly` needs a motion key: the half-plane fades braking at zero speed
  implicitly, and the new shapes drive through zero.
- I and V readbacks become signed i_q / v_q (`FOC_GetIPhase`, `FOC_GetVPhase` exist).
- `Motor_IsSpeedAligned` moves to Traction's gear. `Motor_UserMotoringOf` is valid only under a
  lock; Traction moves to the forward frame (CiA 402 shape).

**Consequences**

- Added 2026-10-18, unwired: `interval_clamp_each` (math_general.h), `hysteresis_sign`
  (hysteresis_math.h), `foc_vq_band` (foc_math.h), `FOC_VqWindow`, `FOC_VqBand`,
  `FOC_GetVLimitDirection`, `FOC_IsVLimitOpen` (FOC.h), `Motor_IsArmed`,
  `Motor_ILimitIntervalOfMotion`, `_Motor_BrakingOnly` and a commented VLimit-form `Motor_Direction`
  (Motor.h). Host probe: 366,626 band cases, 0 failures; the full-width window is the identity for
  |e_q| ≤ 16384.
- Wiring order: init `Foc.VWindow = FRACT16_MAX` first (zero-init pins v_q to e_q); `FOC_VqBand`
  replaces the vqBand lines in Base and Decouple; a motion state field and thresholds for
  `hysteresis_sign`; readbacks.
- Including `hysteresis_math.h` in Motor trips `-Werror`: the existing `abs(...) > uint32` at its
  line 170 is a sign-compare.
- The per-cycle dynamic sides (no-plugging, brake overlay, regen-only, plugging-only) are in
  `foc_math.h`, unwired; their behaviors are in `Motor/Docs/VLimitPolicy.md`. Steady
  i_q = (v_q − e_q) / |Z| maps a v_q band to a reachable i_q band: regen-only caps braking at
  |e_q| / |Z|, plugging-only forces braking ≥ |e_q| / |Z|, and motion-keyed sides cannot hold a hill
  at zero speed.
