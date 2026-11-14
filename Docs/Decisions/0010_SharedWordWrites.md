# 0010 Multi-word values written by one thread and read by another

**Status** — proposed, open. Deferred 2026-11-14: the critical sections written for the review's M13 and
M28 were rolled back the same day; the code stores these values unguarded, as before, and each site
carries a `Decision 0010` pointer.

**Context** — Three values cross a thread boundary as more than one store on a core with no multi-word
atomic store (Cortex-M0+ and M4 alike), and one word is read-modify-written by both sides.

| Value | Writer | Reader | Torn read |
|---|---|---|---|
| `Motor_Context_T.ISupplyLimits`, `interval_t` {low, high} | main 1 ms (`MOTOR_I_SUPPLY_LIMITS_ON_MAIN`) or the 1 ms thread (`_ON_VBUS`), `Motor_FOC_ResolveISupplyLimits` | PWM thread speed tick | new `low` with old `high`: an inverted window for one tick |
| PID gains, `fract16e_t` {mantissa, shift} | protocol write on main, `_Motor_Tuning_Set*` → `PID_SetKp/Ki_Fixed32` | PWM thread current and speed loops | new mantissa with the old shift: a gain off by 2^Δshift for one tick, at the rail a current spike |
| `MotorController_Context_T.FaultFlags`, bit-field word | main `MotorController_PollFaultFlags` (init, fault clear) and `MotorController_PollRxLost` (1 ms), each bit a load-mask-store | 1 ms thread `TransitionFault` ORs its bits in, under the state-machine lock | the 1 ms OR lands between main's load and store: the fault set is lost until the next 16 ms re-send |

The review (Part1 M13, M28) rated the pair tears 0.5 and verified the lost OR by reading the code; none
has been observed on hardware. The writers are infrequent: a protocol write, a 1 ms resolve, an init or a
fault clear.

**Options**

1. Critical section at the writer, `_Critical_Enter/_Exit` (PRIMASK save and restore, nesting-safe). A
   few cycles per write, no new state. Rolled back: the owner wants the thread model decided, not patched
   site by site (Theme 6, thread and ISR data ownership).
2. Single writer per value. `ISupplyLimits` resolved on the reader's thread (the speed tick already
   resolves under `MOTOR_I_SUPPLY_LIMITS_ON_SPEED`); gains applied through the state machine's
   synchronous input path, which the PWM thread consumes; `FaultFlags` set only by the 1 ms thread,
   with main's clear sent as an input to it. Moves ownership, no locks.
3. Value-level atomicity. Pack `interval_t` and `fract16e_t` into one 32-bit word where the ranges
   allow (two `int16` limits in pu; mantissa and shift already fit), and derive the `FaultFlags`
   bits from the monitors at the reader instead of storing them. The fault word's monitor bits are a
   cache of `Monitor_GetStatus` today.
4. Sequence or double buffer. New state; refused by the no-new-state rule unless 1 to 3 fail.

**Decision** — none yet. Pick with Theme 6 of the review (Part3), which states the one thread model;
the lock fix already applied (M38: the state-machine lock carries PRIMASK) is independent of this and
stays.

**Consequences of waiting** — the three windows stay open at their current, low rates. If a
hardware run shows a one-tick torque step after a gain write or a fault that clears itself after a
1 ms set, this record names the mechanism.
