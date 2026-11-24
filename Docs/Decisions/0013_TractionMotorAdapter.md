# 0013 Traction motor adapter

Traction reaches each motor through a per-motor adapter on the Motor's own interface, and the controller's Traction leaves fan out over the motor table; Traction's input is not converted into a common `Motor_Input_T`.

**Status** — accepted and implemented 2026-11-23; amended 2026-11-24 (signatures, below).

**Context**

- Before, `Traction_*(const Traction_T *, Motor_Table_T *)` in `MotorController_Traction.c` walked the motor table
  itself (`Motor_Table_SetCmdWith` with a setter pointer, `Motor_Table_ApplyFeedbackMode`, `Motor_Table_ApplyControl`)
  and decided the brake hold over the table inside Traction (`Traction_ProcBrakeHold`).
- Two ways to take the table out of Traction:
  - **A, per-motor adapter.** Traction maps onto one motor through the Motor's command API. The controller's leaves
    run it over the table. The library already has this shape: `Cia402` / `Motor_Cia402` / `MotorController_Cia402`,
    and the `Motor_Sensor` modules.
  - **B, common input struct.** Traction converts its input and config into a `Motor_Input_T` (`Traction_ToMotorInput`,
    commented out in `Traction.c`), applied through one path (`Motor_ProcSyncInput`, `Motor_Table_ApplyInputs`, both
    commented out). The struct sketch in `Motor_User.h` carries `CmdValue`, `Direction`, `FeedbackMode`,
    `PhaseOutput` and optional limits.
- What B would have to carry, to cover what Traction does today:
  - **Edge and value.** A start edge sets the feedback mode and enables PWM once; a value update only writes the
    command. The sketch re-applies the output on each call, gated by a compare against the present phase state.
  - **Four command frames.** The throttle writes a motoring-frame speed (`Motor_SetSpeedMotoringCmd_Norm`) or Iq
    (`Motor_SetICmd_Norm`), the brake a negative Iq, the lever a signed speed or torque in the user Forward frame
    (`Motor_SetSpeedCmd_Norm`, `Motor_SetTorqueCmd_Norm`). One `CmdValue` read in the active feedback mode's frame
    (`Motor_SetActiveCmd_Norm`) cannot tell the motoring frame from the user Forward frame.
  - **Interventions.** The release in `IZERO` (`Motor_ApplyTorqueZero`) and the brake hold (`Motor_ApplySpeedZero`)
    are state-machine inputs, not a phase output or a feedback mode.
  - **Motor state.** The brake hold engages once no motor advances along its direction (`Motor_IsSpeedAligned` over
    every motor). That reads the motors; it cannot be a conversion of the input.
- B would grow into a copy of the Motor command API, with a dispatcher on the Motor side to undo it. A calls that API
  directly.
- Measured on the KE06 board, Release layout: A is 312 B smaller in flash than the table form it replaced, from dropping
  the setter-pointer loops.

**Decision**

- Three files, as in CiA 402:

  | File | Holds |
  |---|---|
  | `Traction.h` / `.c` | The domain: input, command edges, lever math, config, ConfigId. No Motor headers. |
  | `Motor_Traction.h` / `.c` | One motor: `Motor_Traction_*(Motor_T * p_motor, const Traction_T * p_vehicle, …)` through `Motor_User.h`, motor first as in `Motor_Cia402_*`. The header carries the mapping summary. Signatures amended 2026-11-24, below. |
  | `MotorController_Traction.c` | The leaves. They run `Motor_Traction` over the motor table (`ForEachMotor`), and keep every decision that reads all the motors. |

- The decisions that read every motor live in the leaves: the brake hold once no motor advances (`ApplyBrakeValue`,
  then `Motor_Traction_ApplyBrakeHold` on each motor), and a direction change at standstill
  (`Motor_Table_IsEverySpeedZero`). A new one goes there too, never into `Motor_Traction`.
- `Motor_Input_T` is not Traction's interface.

**Consequences**

- A change to the Motor command API touches `Motor_Traction` alone, as it touches `Motor_Cia402` alone for CiA 402.
- With several motors, each motor gets its feedback mode and then PWM in turn, instead of every mode and then every
  PWM. Each motor's own order is unchanged, and the hold still waits for every motor.
- Fits [0002_LinearThrottleLayering.md](0002_LinearThrottleLayering.md): Traction emits the signed calibrated-frame
  command, and `Motor_Traction_ApplyLeverValue` is that call.
- Not ruled on: the commented sync-input sketch in `Motor_User.h` (`Motor_Input_T`, `Motor_ProcSyncInput`), which was
  meant for buffered synchronous input. This record rejects it only as Traction's interface. The commented
  `Traction_ToMotorInput` in `Traction.c` is the rejected form.

**Amendment 2026-11-24 — signatures**

The form stands; the signatures narrow. Where the Decision above shows `Motor_Traction_*(Motor_T *, const Traction_T *, …)`
with a value parameter, it now reads as follows.

- **Two layers, as in `Motor_Cia402`:**
  - **Public:** every `Motor_Traction_*` takes `(Motor_T * p_motor, const Traction_T * p_traction)`, one signature.
    The value functions read their value from `Traction_Input_T` (`ThrottleValue`, `LeverValue`, `BrakeValue`).
  - **Protected:** `_Traction_ApplyThrottleValue`, `_Traction_ApplyLeverValue` and `_Traction_ApplyBrakeValue` take
    `(Motor_T *, mode, value)`, the one config field each reads and the value. They are declared in `Motor_Traction.h`
    under a Protected heading, with the leading underscore.
- **Why:**
  - One signature lets every leaf call go through `ForEachMotor`, which removes the separate loops for the throttle,
    lever and brake values.
  - The leaves read the same `Input` fields at the same moment as before, so behaviour is unchanged.
  - The protected form covers a value that is not the input's: the release in `ZERO` writes throttle 0 through
    `_Traction_ApplyThrottleValue`.
- **Rejected:** keeping the value as a third parameter (two signatures, two fan-out forms in the leaves).
- **Measured:** KE06 Release, a further 24 B smaller.
- **Naming:** the parameter is `p_traction`, renamed from `p_vehicle`.
