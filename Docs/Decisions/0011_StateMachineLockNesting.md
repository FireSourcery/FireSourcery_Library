# 0011 The state-machine lock and nested inputs

**Status** — proposed, open. Deferred 2026-11-14: the PRIMASK save-and-restore written for the review's M38 was
reverted the same day; the lock is the raw disable / enable pair, as before, and `_StateMachine_ReleaseAsyncInput`
carries a `Decision 0011` pointer. Decide with 0010 and Theme 6 of the review (thread and ISR data ownership).

**Context** — Under `STATE_MACHINE_ASYNC_CRITICAL` every asynchronous input and transition call is wrapped in
`_Critical_DisableIrq()` / `_Critical_EnableIrq()` (`_StateMachine_Lock.h`, the eight `StateMachine.h` call
sites). The enable is unconditional. A handler of one machine that sends an input to another machine nests two
such pairs, and the inner release re-enables interrupts for the rest of the outer handler and its leaf publish:

- `MotorController` `Fault_Entry` → `Motor_StateMachine_SetFault` and `Motor_ReleaseVZ` for each motor
  (`MotorController_StateMachine.c`).
- `Standby_Entry`, `Lock_Entry`, `Common_InputStandby` → `Motor_Table_ApplyControl` / `DisableAll` /
  `Calibration_Enter`, one input per motor.
- Any App handler that calls a `Motor_Apply*`.

The window is the same shape as M37's (the 1 ms thread's fault input against the unlocked main-thread
transition), so a fix for one should be the fix for both. No failure has been observed on hardware.

**Options**

1. PRIMASK save and restore through a local token at each call site (`critical_state_t lock;` plus two
   parameters). Nesting-safe from any context, no new state in the machine. Written and compile-checked,
   then reverted: the owner wants the lock model chosen, not patched.
2. The nesting counter already in `Critical.h` (`Critical_DisableIrq` / `Critical_EnableIrq` on
   `_Critical_InterruptDisableCount`). One global count for every machine; wrong if a release ever runs on
   a thread other than its acquire.
3. No nesting by construction: controller handlers return the motor inputs as work for a post-transition
   step, or the controller's `ENTRY` actions use the motors' synchronous (buffered) input path, which the
   PWM thread consumes. Moves the second machine's input off the first machine's lock.
4. Accept the window as M37's and close both with the 1 ms thread setting flags only (M37's option).

**Decision** — none yet.

**Consequences of waiting** — a controller transition that fans out to the motors runs its tail unlocked; the
1 ms thread's fault input can land inside it, as M37 already allows. The re-send at the next poll covers the
lost transition, so the exposure is one poll period.
