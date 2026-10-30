# 0005 Current limit quadrants and keying frames

**Status** — accepted, 2026-10-13 to 2026-10-18.

**Context** — Current limits come from the bus (UV, OV, the DC supply window), heat, the user and
the protocol (CiA 402 0x60E0 / 0x60E1). Each is defined in a different frame, and a limit keyed by
the wrong frame either swaps sides in reverse or shuts the current loop at standstill. The physics
behind each bound is in [../Modules/Motor/LimitPhysics.md](../Modules/Motor/LimitPhysics.md).

**Decision**

- **Derates split per quadrant**: UV limits motoring (drawing) current, OV limits generating
  (returning) current, heat limits both as one absolute cap (R_on · I²). One shared derate would put
  OV chop on motoring current.
- **Each limit is keyed by the frame it is defined in**, and the resolved window is the
  intersection Mot/Gen ∩ Fwd/Rev ∩ supply:

  | Frame | Key | Holds | Zero policy |
  |---|---|---|---|
  | Supply | sign(v_q): power flow | DC charge / discharge window | v_q → 0 → open |
  | Mot/Gen | `Direction`: drive vs retard against the selected direction | user motoring / generating limits | `Direction` NULL → [0:0], the arming gate |
  | Fwd/Rev | `DirectionForward`: absolute torque direction | CiA 402 0x60E0 / 0x60E1 | never zero (config invariant) |

- **Never key supply logic by `Direction`.** Do not assume `FOC_SetVLimits`' anti-plugging
  half-plane is the only VLimit writer: `FOC_SetVSymmetric` may be used, so v_q can oppose
  `Direction` (plugging, rollback). Power flows by sign(v_q · i_q). Key by sign(v_q), then resolve
  onto `Direction` with `interval_aligned` / `interval_opposed` (`Motor_ILimitMotoringIn`). Comments
  such as FOC.h's "Inner layer assume plugging is clamped" are suspect.
- **Never key the AC pair by sign(v_q)**: v_q = 0 at standstill gives [0:0], and the loop can never
  start. The zero policy sits at the call site; it must never become `math_sign(vq)`.
- **Each key carries its own zero policy**, so a shared resolver must not impose one.
- **`Motor_QuadrantId`** (speed, i_q, v_q) is an operating-point classifier for status, not a limit
  key. Recommended form: the signed 7-state enum built from `sign_t` inputs, with the motion sign
  deadbanded by the caller. Plugging requires generating.

**Consequences**

- Mot/Gen keyed by `Direction` keeps rollback hill hold on the motoring limit. ABB (signed torque
  plus motoring / generating power) and SINAMICS (p1520 / p1521 plus p1530 / p1531) publish the same
  layering.
- A UV/OV ratio derate in the motoring frame approximates power flow; it is exact only under
  anti-plugging.
- Open, evaluated 2026-10-17: keying Mot/Gen by the mechanical quadrant (sign of τ · ω), as
  published, needs a zero-speed deadband with symmetric motoring inside it, and makes forward torque
  in rollback generating. See [0009_DirectionPolicy.md](0009_DirectionPolicy.md).
- The DC window belongs to the bus; with more than one motor on a bus it must be split.
- A limit-status word (CiA 402 bit 11, ABB 06.19 / 06.11 / 06.16) needs the raw request, because the
  `Ramp_SetTarget` store clamp hides it. Condition data objects for it (raw command with frame tag,
  effective window with a per-side source, operating point) were sketched 2026-10-17, not adopted.
