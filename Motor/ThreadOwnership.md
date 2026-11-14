# Thread Ownership

Every `Motor_Context_T`, `FOC_T` and `MotorController_Context_T` field that one thread writes and another reads: the writer's context, the reader, the field's width, and whether the store is one word. The multi-word crossings are the ones Decision [0010](../Docs/Decisions/0010_SharedWordWrites.md) records; this sheet locates them, it does not propose locks. The lock's own nesting: [0011](../Docs/Decisions/0011_StateMachineLockNesting.md). Derived from the call chains as of 2026-11-14.

## Contexts

| Context | Runs | Entered through |
|---|---|---|
| **PWM** | the control cycle, in the PWM ISR | `Motor_PWM_Thread`: `RotorSensor_CaptureAngle`, `_Motor_StateMachine_Thread` (the leaf's LOOP and NEXT, the ENTRY of a state NEXT selects), `Motor_PollSpeed` (ON_SPEED once per ms: the speed tick) |
| **ADC** | the conversion-complete ISR, above the PWM ISR's priority | the board's completion map: `Motor_ADC_CaptureIabc`, `Motor_ADC_CaptureVabc`, `VBus_ADC_Capture` |
| **1 ms** | the periodic timer ISR, at the PWM ISR's priority | `MotorController_Timer1Ms_Thread` → `_MotorController_VBus_Thread`: `VBus_PollMonitor`, `MotorController_SetFault` (under the lock), `Motor_FOC_ResolveISupplyLimits` under `MOTOR_I_SUPPLY_LIMITS_ON_VBUS` |
| **main L** | the main loop with interrupts off | `StateMachine_Tree_Input` / `StateMachine_Tree_InvokeTransition` under `STATE_MACHINE_ASYNC_CRITICAL`: the input handler, and the EXIT and ENTRY of the transition it returns. `Motor_User.c`, `Motor_StateMachine.c` handlers, `MotorController_StateMachine.c` handlers |
| **main** | the main loop, interrupts on | `MotorController_Main_Thread`: the 1 ms block (`_StateMachine_Branch_ProcSyncOutput` of the controller, `Socket_Proc` → `Motor_Var` / `MotorController_Var` setters, `Motor_FOC_ResolveISupplyLimits` under `_ON_MAIN`), the 16 ms block (DIN polls), the 1 s block (`MotorController_ProcILimits`, `_ProcSpeedLimits`, monitors, `Motor_Heat_Thread`) |

A state ENTRY runs in whichever context made the transition: PWM when NEXT selected it, main L when an input did. Rows below name both where it matters.

## Motor_Context_T

Width in bytes. "Single store": one aligned store of the whole value on the target.

| Field | Writer | Reader | Width | Single store | Note |
|---|---|---|---|---|---|
| `StateMachine.p_ActiveState` | PWM (NEXT, `_StateMachine_Branch_ProcSyncOutput`); main L (input handlers) | PWM; main (`Motor_GetStateId`, vars) | 4 | yes | The leaf is published after EXIT and ENTRY; the 1 ms ISR's fault input against an unlocked step: 0011 |
| `StateMachine.SyncInputMask`, `SyncInputs` | none (the motor uses the async input path only) | — | 4 + 4·N | — | Unused by the motor |
| `ControlTimerBase` | PWM (`Motor_PWM_Thread` increments; entries reset) | PWM (`Motor_IsISampleCycle`, SafeStop watchdog) | 4 | yes | PWM only |
| `FeedbackMode` | main L (`Motor_SetFeedbackMode` in the FEEDBACK_MODE handlers; `Intervention_InputFeedbackMode` stores directly) | PWM (`Run_Proc`, `Run_Entry`); main (vars) | 1 | yes | Locked |
| `UserCmd` | main, unlocked (`Motor_SetICmd`, `Motor_SetSpeedCmd`, … from `Motor_Var`, the app); PWM (`Run_Entry` match, `Motor_FOC_ClearFeedbackState`) | PWM (`Run_Proc`, `TorqueZero_Proc`) | 4 | yes | Two writers, last store wins; a main write between an entry's match and its LOOP is the newer command |
| `FaultFlags` | main L / 1 ms L (`TransitionFault`, `Fault_InputFault`); PWM (`SafeStop_Next`, `_Motor_TransitionFeedbackLost`, `Init_Next`) | PWM (`Fault_Entry`); main (vars) | 2 | no: bit-field read-modify-write | The locked RMW and the PWM RMW never interleave: the lock masks the PWM ISR, and the 1 ms ISR shares its priority |
| `Direction` | main L (`Motor_SetDirection` in the DIRECTION handlers); PWM or main L (`Deactivated_Entry`) | PWM every cycle (`Motor_Direction`, the limit resolves); main (vars) | 4 (enum) | yes | Locked. Written with `Foc.VLimit` |
| `p_ActiveSensor` | `Motor_Init` | PWM | 4 | yes | Init only |
| `SensorState.AngleSpeed` {Angle, Delta, limits} | PWM (`RotorSensor_CaptureAngle`, `_CaptureSpeed`) | PWM; main (vars) | 16 | per word | A main read of Angle and Delta together spans two cycles: display only |
| `SensorState.Speed_Pu` | PWM, speed tick | PWM; main (vars, `Motor_Table_IsEverySpeedZero`, the PHASE_BOUND supply law) | 4 | yes | |
| `SensorState.UnitRef` {PolePairs, AngleFreqBase, SpeedPuRef} | main (`Motor_InitUnits` ← `Motor_Reset` ← `Motor_Reinit`, in the controller's LOCK state) | PWM every cycle (angle integration, the speed base) | 12 | no | **Re-derived by `Motor_Reinit`** while the motor is DEACTIVATED or CALIBRATION; the PWM thread still captures the angle through it |
| `SpeedRamp` | PWM (speed tick `Ramp_ProcNextOf`, `Run_Entry` match); PWM or main L (`Motor_ResolveSpeedLimits` on entry) | PWM | 20 | — | PWM only; the limits pair is written on the reader's thread or under the lock |
| `PidSpeed.Kp`, `PidSpeed.KiPerTick` (`fract16e_t`) | main, unlocked (`_Motor_Tuning_SetSpeedKp/Ki` ← `_Motor_Var_PidTuning_Set`) | PWM, speed tick (`PID_ProcPI`) | 8 each (int32 mantissa, uint8 shift) | no | **0010 row 2**: new mantissa with the old shift for one tick |
| `PidSpeed.OutputMin/Max`, `IntegralAccum`, `Output`, `ErrorPrev` | PWM (`PID_ProcPI`, `Motor_ResolveILimits` on the speed tick); PWM or main L (entries) | PWM | 2, 2, 4, 2, 4 | per word | On the reader's thread or locked |
| `PidSpeed.Config`, `Foc.PidIq.Config`, `Foc.PidId.Config` | main (tuning setters, `_Motor_ResetTuning`) | main (vars, save); PWM never (read by `PID_InitFrom` at reset only) | 20 | no | Not a crossing |
| `PhaseInput.I` {A, B, C, Flags} | ADC (`Motor_ADC_CaptureIabc`: the whole `Phase_Data_T`); PWM (`Phase_Input_ClearI` in entries; `FOC_CaptureIData` clears Flags) | PWM, the LOOP's take (`Motor_FOC_CaptureI`: A, then B, then C, no flag check); PWM Passive (`FOC_CaptureIData`) | 8 | no: 3 halfwords and a flag byte | **The I set**: a completion landing between the take's first and last read mixes two sets. The ISR's store against Passive's read-then-clear of Flags marks a set consumed before it is read, once |
| `PhaseInput.V` {A, B, C, Flags} | ADC (`Motor_ADC_CaptureVabc`); PWM (`Phase_Input_ClearV`, `FOC_CaptureVBemfData` clears Flags) | PWM Passive and Deactivated (`FOC_CaptureVBemfData`) | 8 | no | Same shape as I; Passive only |
| `TorqueRamp` | PWM (`Ramp_ProcNextOf` every cycle, matches on entry); PWM or main L (`Motor_ResolveILimits` limits) | PWM | 20 | — | PWM only; limits on the reader's thread or locked |
| `ILimit.Motoring`, `ILimit.Generating` | main 1 s, unlocked (`MotorController_ProcILimits` → `Motor_SetILimitDerates` → `Motor_CaptureILimits`, two stores); `Motor_Init` | PWM speed tick and entries (`_Motor_ResolveILimits`); main (`Motor_User_ILimit*` vars) | 2 + 2 | no: 2 stores | A tick may pair the new Motoring with the old Generating; both carry the same derate ratio, so the pair differs only while the ratio moves |
| `ILimit.Forward`, `ILimit.Reverse` | `Motor_ResetILimit` (FRACT16_MAX) | none | 2 + 2 | — | Unused |
| `ISupplyLimits` (`interval_t`) | main 1 ms, unlocked (`_ON_MAIN`); or 1 ms ISR (`_ON_VBUS`); or PWM speed tick (`_ON_SPEED`); PWM or main L on entry (`Motor_FOC_ResolveISupplyLimitsOnEntry`); `Motor_Init` | PWM speed tick and entries (`_Motor_ResolveILimits` intersect); main (`Motor_User_ILimit*` vars) | 8 | no: 2 words | **0010 row 1**. Under `_ON_MAIN` the writer also reads `Foc.Vd`, `Foc.Id`, `Foc.Vq` and `VBus_Pu` from the other threads: a mixed operating point for one 1 ms sample |
| `SpeedLimit.Forward`, `SpeedLimit.Reverse` | main 1 s, unlocked (`MotorController_ProcSpeedLimits` → `Motor_SetSpeedLimitDerate`, two stores) | PWM speed tick and `Run_Entry` (`Motor_ResolveSpeedLimits`); main (vars) | 2 + 2 | no: 2 stores | As `ILimit` |
| `OpenLoopSpeedRamp`, `OpenLoopIRamp`, `OpenLoopAngle` | PWM | PWM | 20, 20, 16 | — | PWM only |
| `HeatMonitorState` | main 1 s (`Motor_Heat_Thread`) | main (vars) | — | — | main only |
| `Config` (whole struct) | main (`Motor_Init`, `Motor_Reinit`: struct copy from NVM, in init and the controller's LOCK state) | PWM and ADC, scattered fields below | sizeof | no | **Re-derived**: the copy runs field by field while the readers run |
| `Config.IabcZeroRef_Adcu` | main (`Motor_Config_SetIaZero_Adcu`, … vars); PWM (the ADC calibration state's end) | ADC every set (`Phase_Iabc_PuOfAdcu`) | 6 | no: 3 halfwords | A set converted across the three stores mixes zeros, once, during calibration |
| `Config.ILimitMotoring_Pu`, `ILimitGenerating_Pu` | main (vars) | PWM speed tick and entries (`Motor_ILimitMotoring`, `_Generating`); PWM open loop | 2 + 2 | no: 2 setters | As `ILimit` |
| `Config.SpeedLimitForward_Pu`, `SpeedLimitReverse_Pu`, `DirectionForward` | main (vars) | PWM speed tick (`Motor_ResolveSpeedLimits`) | 2, 2, 4 | per field | |
| `Config.IAlign_Pu`, `AlignTime_Cycles`, `OpenLoopRamp*` | main (vars) | PWM (open loop, align, calibration states) | 2 / 4 | per field | |
| `Config.Electrical`, `Config.FieldWeakening`, `Config.PidI`, `Config.PidSpeed` | main (vars, calibration commits) | PWM never directly: through the derived `Foc.Electrical`, `Foc.FieldWeakening`, the PID gains | — | — | See the `FOC_T` rows |

## FOC_T

| Field | Writer | Reader | Width | Single store | Note |
|---|---|---|---|---|---|
| `Sine`, `Cosine` | PWM (`FOC_SetTheta`) | PWM; main (none) | 2 each, 4-aligned | yes | |
| `Ialpha`, `Ibeta`, `Id`, `Iq` | PWM (`FOC_ProcClarkePark`), `FOC_ClearCaptureState` | PWM; main (`FOC_Var_Get`, `Motor_GetIPhase_Pu`, the supply resolve under `_ON_MAIN`, `Motor_FOC_IsMotoring`) | 2 each | yes | A main read of a pair spans cycles: display, and the `_ON_MAIN` operating point |
| `Vd`, `Vq` | PWM (`FOC_ProcIFeedback_Base`, `FOC_ProcVBemfClarkePark`, matches) | PWM; main (vars, the `_ON_MAIN` resolve) | 2 each | yes | As above |
| `Valpha`, `Vbeta`, `Va`, `Vb`, `Vc` | PWM (`FOC_ProcInvClarkePark`) | PWM (`Motor_FOC_WriteDuty`, the observer); main (vars) | 2 each | yes | |
| `PidIq.Kp`, `PidIq.KiPerTick`, `PidId.Kp`, `PidId.KiPerTick` | main, unlocked (`_Motor_Tuning_SetIKp/Ki`) | PWM every cycle (`PID_ProcPI`) | 8 each | no | **0010 row 2** |
| `PidIq.OutputMin/Max`, `PidId.OutputMin/Max` | PWM every cycle (`PID_CaptureOutputLimits`); main L (`FOC_SetVLimits` ← `Motor_SetDirection`) | PWM | 2 + 2 | per field | Locked or same thread |
| `PidIq`, `PidId` state (`IntegralAccum`, `Output`, `ErrorPrev`) | PWM | PWM; main (`FOC_Var_Get` integral) | 4, 2, 4 | per word | |
| `VLimit` (`interval_t`) | main L (`Motor_SetDirection` → `FOC_SetVLimits`); PWM or main L (`Deactivated_Entry`); `Motor_Reset` | PWM every cycle (`FOC_ProcIFeedback_Base`, `FOC_ProcVControl`) | 8 | no: 2 words | Locked: the PWM thread cannot interleave with the two stores |
| `VWindow` | `Motor_Reset` (0) | none wired | 2 | yes | [VLimitPolicy.md](Motor/Math/VLimitPolicy.md) |
| `Electrical` {Ld, Lq, Rs, Psi}, `LShift` | main, unlocked (`Motor_FOC_InitParams` ← `Motor_Var_ConfigElectrical_Set`, the ELECTRICAL_PU var type, `Motor_Reset`); PWM (the electrical calibration's COMMIT step) | PWM speed tick (`FOC_CaptureSpeed`: Ld, Lq, Psi); `LShift` in the decouple build only | 16 + 1 | no: 4 words and a byte | **Re-derived by `Motor_FOC_InitParams`**: a speed tick between the stores mixes old and new L, ψ for one tick |
| `ElectricalSpeed` {OmegaLd, OmegaLq, OmegaPsi} | PWM speed tick (`FOC_CaptureSpeed`) | PWM (`FOC_MatchIdFieldWeakening`, the decouple feedforward, `FOC_MatchIVSpeed`) | 12 | — | PWM only |
| `FieldWeakening` {IdLimit, IdGain} | main, unlocked (`Motor_FOC_InitParams`) | PWM every cycle (`FOC_ProcIdFieldWeakening`) | 2 + 2 | not guaranteed: a struct assignment the compiler may split | **Re-derived by `Motor_FOC_InitParams`** |
| `IdFw` | PWM (`FOC_ProcIdFieldWeakening`, `FOC_MatchIdFieldWeakening`, reset) | PWM | 2 | yes | No var reads it |

## MotorController_Context_T

The controller's state machine runs its LOOP on **main** (`_StateMachine_Branch_ProcSyncOutput`, the 1 ms block), not in an ISR. Its inputs come from main L (state commands, lock ops, the 1 s monitors' `MotorController_SetFault`) and from the 1 ms ISR under the lock (`_MotorController_VBus_Thread` → `MotorController_SetFault`).

| Field | Writer | Reader | Width | Single store | Note |
|---|---|---|---|---|---|
| `StateMachine.p_ActiveState` | main (LOOP's NEXT, unlocked); main L and 1 ms L (inputs) | main | 4 | yes | The 1 ms fault input can land inside an unlocked main-thread transition: M37, 0011 |
| `FaultFlags` | main, unlocked bit-field RMW (`MotorController_PollFaultFlags` in `Init_Next` and the fault clear; `MotorController_PollRxLost` in the 1 ms block; `Init_Next`, `Lock_InputLockOp_Blocking` single bits); 1 ms L and main L (`TransitionFault` OR, `Fault_InputFault`) | main (NEXT, `Standby`, vars) | 2 | no: bit-field RMW | **0010 row 3**: the 1 ms OR lands inside main's load-mask-store and is lost until the next poll |
| `StandbyExitMode`, `LockOpStatus`, `NvmStatus`, `Config`, `BootRef`, `OptDinState`, `StateCounter`, `AvgBuffer0/1` | main | main | — | — | main only |
| `ControlCounter` | PWM ISR (the application's increment) | the board's completion diagnostics, when built | 4 | yes | |
| `MicrosRef` | PWM ISR | the board's completion diagnostics | 4 | yes | |
| `ISampleMicrosRef` | ADC (the board's completion map) | PWM ISR (`ISequenceProfile` capture) | 4 | yes | |
| `ControlLoopProfile`, `ISequenceProfile` | PWM ISR | main (vars) | 4 each | yes | |

## Beside the three structs

| Field | Writer | Reader | Width | Single store | Note |
|---|---|---|---|---|---|
| `VBus_T.VBus_Pu`, `VBus_T.VBusInv_Pu` | ADC (`VBus_ADC_Capture`: Pu, then the divide into Inv) | PWM every cycle (`VBus_Pu` for the circle and the requests, `VBus_Inv_Pu` for the duty write); 1 ms ISR (`VBus_PollMonitor`); main (`_ON_MAIN` resolve, derates, vars) | 2 + 4 | no: 2 stores | A LOOP may normalize with an Inv one sample newer than the Pu its circle used; the two-sample filter keeps the step small |
| `ADC_State_T.ChannelMarkers`, `CompleteFlags`, `ActiveChannels` | PWM ISR, ADC ISR, main (marks) | the same | 4, 4, 1·N + 1 | RMW | The ADC module's own words: [Peripheral/ADC/VariantMap.md](../Peripheral/ADC/VariantMap.md), the board's set sheet |

## The multi-word crossings

| Value | Stores | Writer thread | Reader thread | Recorded |
|---|---|---|---|---|
| `ISupplyLimits` | 2 words | main 1 ms (`_ON_MAIN`) or 1 ms ISR (`_ON_VBUS`) | PWM speed tick | 0010 row 1 |
| PID gains, `Kp` and `KiPerTick` of `PidSpeed`, `Foc.PidIq`, `Foc.PidId` | mantissa word + shift byte, per gain | main, protocol write | PWM every cycle, speed tick | 0010 row 2 |
| `MotorController_Context_T.FaultFlags` | bit-field RMW | main; 1 ms L | main | 0010 row 3 |
| `PhaseInput.I` | 3 halfwords + flags | ADC ISR | PWM, the take, field by field | this sheet; the window is drawn in the board's control-cycle timeline |
| `Foc.Electrical`, `LShift`, `Foc.FieldWeakening` | 4 words + byte; 2 halfwords | main (`Motor_FOC_InitParams`) | PWM speed tick; every cycle | this sheet |
| `SensorState.UnitRef`, `Config` whole | 3 words; sizeof | main (`Motor_Reinit`, LOCK state) | PWM every cycle | this sheet |
| `ILimit`, `SpeedLimit`, `Config.ILimit*_Pu`, `Config.IabcZeroRef_Adcu` | 2 to 3 halfwords | main 1 s, vars | PWM speed tick; ADC ISR | this sheet, benign by construction |

Not multi-word, but two-sided: `UserCmd` (main store against the PWM thread's match), `ChannelMarkers` (three contexts' RMW).

## Sources

`Motor/Motor/Motor.h` (`Motor_Context_T`, the resolve inlines), `Motor_FOC.h` / `Motor_FOC.c` (`Motor_FOC_CaptureI`, `Motor_FOC_ResolveISupplyLimits`, `Motor_FOC_ClearFeedbackState`), `Motor_Thread.h` (`Motor_PWM_Thread`, `Motor_PollSpeed`, `Motor_Heat_Thread`), `Motor_StateMachine.c` (the handlers, `Run_Entry`, `Run_OnSpeed`, `Deactivated_Entry`), `StateMachine/Motor_Intervention.c`, `Motor_User.c` (`Motor_Set*Cmd`), `Motor.c` (`Motor_Init`, `Motor_Reset`, `Motor_InitUnits`, `Motor_FOC_InitParams`, `_Motor_Tuning_Set*`, `Motor_SetDirection`, `Motor_CaptureILimits`), `Motor_Var.c` (`_Motor_Var_PidTuning_Set`, `Motor_Var_ConfigElectrical_Set`), `ADC/Motor_ADC.h` (`Motor_ADC_CaptureIabc`), `Phase_Input/Phase_Input.h`, `Phase_ADC.h`, `Math/FOC.h` (`FOC_T`, `FOC_SetVLimits`, `FOC_InitElectrical`), `Math/FOC_Ext.h` (`FOC_CaptureIData`), `MotorController/MotorController.h` (`MotorController_Context_T`, `MotorController_PollRxLost`), `MotorController.c` (`MotorController_ProcILimits`), `MotorController_Thread.h` (`MotorController_Main_Thread`, `MotorController_Timer1Ms_Thread`), `MotorController_StateMachine.c` (`MotorController_PollFaultFlags`, `TransitionFault`), `MotorController_User.c` (`MotorController_SetFault`), `VBus/VBus.h`, `Math/Fixed/fract16e.h`, `Math/math_general.h` (`interval_t`), `Math/PID/PID.h` / `PID.c`, `Framework/StateMachine/_StateMachine_Lock.h`, `StateMachine.h` (`StateMachine_Tree_Input`), `Motor_StateMachine.h` (`_Motor_StateMachine`).

## Not shown / open

- The hardware registers the PWM thread writes (duty, gates) and the ADC ISR reads: the board's sheets.
- `CalibrationBuffer`: written by the calibration states (PWM) and read by their own steps; no cross-thread reader found.
- The sensor modules' own state (Hall, encoder, the observer) behind `p_ActiveSensor`: PWM only, except their Config, written by main.
- Whether GCC emits `Foc.FieldWeakening = (…){…}` as one word store or two halfword stores on each target: not checked in the disassembly.
- The order the leaf is published in against EXIT and ENTRY, and where a half-made transition is observable: item 4 of the visualization list, not this sheet.
- Which of 0010's options closes which row: the decision's.
