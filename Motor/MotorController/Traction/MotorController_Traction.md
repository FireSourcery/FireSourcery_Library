# MotorController with Traction

The controller tree with the Traction leaves in one picture, and the cross-layer sequences: what each Traction input sends to
the motor machines and the motor state after each step. The controller's root states, lock ids, fault flags and write
admission are in [MotorController_StateMachine.md](../MotorController_StateMachine.md); the motor side in
[Motor_StateTree.md](../../Motor/Motor_StateTree.md). Direction values are the user frame, `FORWARD` / `REVERSE` / `NULL`, mapped to
the motor's `CW` / `CCW` by `Config.DirectionForward` in `Motor_ApplyUserDirection`. "Standby ready" is
`Common_IsStandbyReady`: every motor `DEACTIVATED`, or every motor `PASSIVE` at speed 0.

Traction's own functions (`Traction.c`) act on one motor, through the motor's interface. The leaves run them over the motor
table and keep the decisions that read every motor: the brake hold once no motor advances (`ApplyBrakeValue`), a direction
change at standstill (`Motor_Table_IsEverySpeedZero`).

## Tree

`STATE_NEUTRAL` and `STATE_DRIVE` (`MotorController_Traction.c`) mount under `MC_STATE_MAIN` beside `MC_STATE_MAIN_MOTOR_CMD`;
`MC_STATE_LOCK_CALIBRATE_ADC` (`MotorController_Analog.c`) under `MC_STATE_LOCK`. Which leaf `START_MAIN` lands on is the
app's `ENTER_MAIN`: `MC_APP_TRACTION` answers `NEUTRAL` or `DRIVE`, the default app `MAIN_MOTOR_CMD`.

```mermaid
stateDiagram-v2
    direction TB
    [*] --> INIT
    state MAIN {
        direction LR
        state "bare MAIN" as MAIN_BARE
        state "MAIN_MOTOR_CMD" as MOTOR_CMD
        state "STATE_NEUTRAL" as NEUTRAL
        state "STATE_DRIVE" as DRIVE
        MAIN_BARE --> NEUTRAL: START_MAIN, ENTER_MAIN, a motor moving or direction NULL
        MAIN_BARE --> DRIVE: START_MAIN, ENTER_MAIN, direction FORWARD or REVERSE, every motor at speed 0
        NEUTRAL --> DRIVE: DIRECTION FORWARD or REVERSE, every motor already that way, or every motor at speed 0
        NEUTRAL --> NEUTRAL: DIRECTION NULL, Z to every motor again
        DRIVE --> NEUTRAL: DIRECTION NULL
    }
    state LOCK {
        direction LR
        state "bare LOCK" as LOCK_BARE
        state "LOCK_CALIBRATE_ADC" as CAL_ADC
        [*] --> LOCK_BARE
        LOCK_BARE --> CAL_ADC: CALIBRATE_ADC
        CAL_ADC --> LOCK_BARE: NEXT, 2000 ms
    }
    INIT --> STANDBY: NEXT, INIT_WAIT elapsed, no FaultFlags
    INIT --> FAULT: NEXT, INIT_WAIT elapsed, FaultFlags set
    STANDBY --> NEUTRAL: START_MAIN or AUTO exit, ENTER_MAIN, a motor moving or direction NULL
    STANDBY --> DRIVE: START_MAIN or AUTO exit, ENTER_MAIN, direction FORWARD or REVERSE, every motor at speed 0
    STANDBY --> MOTOR_CMD: START_MAIN, default app
    STANDBY --> STANDBY: STOP_MAIN or E_STOP, StandbyExitMode MANUAL
    STANDBY --> LOCK_BARE: LOCK_ENTER, standby ready
    MAIN --> STANDBY: STOP_MAIN or E_STOP from any leaf, standby ready
    LOCK_BARE --> STANDBY: LOCK_EXIT, every motor DEACTIVATED or CALIBRATION
    LOCK_BARE --> FAULT: LOCK_EXIT, a motor elsewhere, Motors flag
    LOCK_BARE --> MOTOR_CMD: MOTOR_CMD_MODE
    LOCK_BARE --> MOTOR_TUNING: MOTOR_TUNING_MODE
    MOTOR_TUNING --> STANDBY: STOP_MAIN or E_STOP, standby ready
    MOTOR_TUNING --> MOTOR_TUNING: MOTOR_TUNING_RESET
    MOTOR_TUNING --> LOCK_BARE: LOCK_ENTER, standby ready
    FAULT --> STANDBY: FAULT clear, no flag left
    note right of FAULT
        FAULT set enters from every root but INIT.
        The Traction leaves map only APP_USER, so FAULT and STATE_CMD walk up to MAIN.
    end note
```

| Leaf | ENTRY | Own input slots | Motor states while active |
|---|---|---|---|
| `STATE_NEUTRAL` | `PHASE_OUTPUT` `Z` to every motor | `APP_USER`: `DIRECTION`, `DRIVE_CMD` (release → `Z`, brake → `Traction_StartBrakeMode`, throttle ignored), `BRAKE_VALUE`; throttle and lever values ignored | `PASSIVE`; `RUN`, `INTERVENTION` while braking; direction retained |
| `STATE_DRIVE` | `Traction_Input_PollCmd` (syncs `DriveCmd`, no edge), `Motor_Table_ApplyUserDirection` with the buffered direction | `APP_USER`: `DIRECTION` (`NULL` → `NEUTRAL`; `FORWARD` / `REVERSE` applied at standstill only), `DRIVE_CMD`, `THROTTLE_VALUE`, `BRAKE_VALUE`, `LEVER_DIRECTION` (at standstill), `LEVER_VALUE` | `PASSIVE`, `RUN`, `INTERVENTION`, the start-up chain under `OPEN_LOOP` |
| `MAIN_MOTOR_CMD` | `PHASE_OUTPUT` `0` | `MOTOR_CMD`, empty handler | as `MAIN` |
| `LOCK_CALIBRATE_ADC` | averaging started | none | `CALIBRATION` leaf `CALIBRATION_STATE` (ADC zero) |

## Cross-layer table

Every motor-facing call the Traction leaves and the controller roots make, the motor input it becomes, and what each motor
state does with it. "ignored" is an unmapped slot or a handler that returns `NULL` without effect.

| Caller | Motor call | Motor input | `PASSIVE` | `RUN` | `TORQUE_ZERO`, `SPEED_ZERO` | `SAFE_STOP` | `OPEN_LOOP` branch | `DEACTIVATED`, `CALIBRATION`, `FAULT` |
|---|---|---|---|---|---|---|---|---|
| `Drive_Entry`; `Neutral_InputDirection` at standstill; `Drive_InputDirection`, `Drive_InputLeverDirection` at standstill | `Motor_Table_ApplyUserDirection` → `Motor_ApplyVirtualDirection`, on a change | `DIRECTION` | stored at speed 0, VLimit half-plane set | ignored | ignored | ignored | at speed 0: `NULL` → `PASSIVE`, else stored | `DEACTIVATED` stores at any speed; `CALIBRATION` at speed 0; `FAULT` ignores |
| `Traction_StartThrottleMode`, `Traction_StartBrakeMode`, each motor | `Motor_ApplyFeedbackMode`, on a change | `FEEDBACK_MODE` | stored | stored, `RUN` re-entered, loops re-matched | stored, applied by `Run_Entry` on resume | as its parent | stored | `DEACTIVATED` stores; `CALIBRATION`, `FAULT` ignore |
| `Traction_StartThrottleMode` (`SPEED`, `TORQUE`), `Traction_StartBrakeMode` (`TORQUE`), each motor | `Motor_ApplyControlState` `PWM` | `PHASE_OUTPUT` `PWM` | → `RUN` with direction and feedback; the start-up chain at speed 0 without feedback; else nothing | nothing | → `RUN` with feedback | refused | nothing | `CALIBRATION` V0; others ignore |
| `Neutral_Entry`, `Neutral_InputCmdStart` release, `Traction_StartDriveZero` `FLOAT` / `REGEN`, `Traction_Start*Mode` `VOLTAGE` / `PASSIVE`, `Common_InputStandby`, `Standby_Entry`, `Fault_Entry` (`Motor_ReleaseVZ`) | `Motor_Table_ApplyControl` `Z`; `Motor_ApplyControlState` `Z` from the `Traction_` functions, each motor | `PHASE_OUTPUT` `Z` | floats within the freewheel limit | → `PASSIVE` within the limit, else `SPEED_ZERO` | → `PASSIVE` within the limit, else `SPEED_ZERO` (re-entry) | → `PASSIVE` at speed 0, else refused | → `PASSIVE` | `CALIBRATION` floats → `PASSIVE`; `DEACTIVATED`, `FAULT` ignore |
| `Main_Entry`, `MotorCmd_Entry` | `Motor_Table_ApplyControl` `0` | `PHASE_OUTPUT` `0` | V0 | as `Z` | as `Z` | as `Z` | → `PASSIVE` | `CALIBRATION` V0; others ignore |
| `ApplyBrakeValue` once no motor `Motor_IsSpeedAligned`, then `Traction_ApplyBrakeHold` (`TORQUE`) each motor | `Motor_ApplySpeedZero` | `INTERVENTION` `SPEED_ZERO` | ignored | → `SPEED_ZERO` | → `SPEED_ZERO`; nothing when already active | refused | ignored | ignored |
| `Traction_StartDriveZero` `IZERO` | `Motor_ApplyTorqueZero` | `INTERVENTION` `TORQUE_ZERO` | ignored | → `TORQUE_ZERO` | → `TORQUE_ZERO`; nothing when already active | refused | ignored | ignored |
| `MotorController_Main_Thread`, serial `RxLost` | `Motor_ApplySafeStop` | `INTERVENTION` `SAFE_STOP` | ignored | → `SAFE_STOP` | → `SAFE_STOP` | refused | ignored | ignored |
| `Traction_ApplyThrottleValue`, `Traction_ApplyBrakeValue`, `Traction_ApplyLeverValue`, `Traction_StartDriveZero` `ZERO` | `Motor_SetSpeedMotoringCmd_Norm`, `Motor_SetICmd_Norm`, `Motor_SetSpeedCmd_Norm`, `Motor_SetTorqueCmd_Norm` | none: `UserCmd` written in any state | held; `Run_Entry` overwrites it from the matched ramp | followed by the loop | `TORQUE_ZERO` takes the generating half; `SPEED_ZERO` ignores | ignored | ignored | held |
| `Main_Entry` | `Motor_Table_EnableAll` (`Motor_Enable`) | command on `DEACTIVATED` | | | | | | `DEACTIVATED` → `PASSIVE` |
| `Standby_Entry`, `Lock_Entry` | `Motor_Table_DisableAll` (`Motor_Disable`) | command on `PASSIVE` | → `DEACTIVATED` at speed 0 | | | | | |
| `Lock_Entry` | `Motor_Calibration_Enter` | `CALIBRATION` root pointer | ignored (`NULL` slot) | ignored | ignored | ignored | ignored | `DEACTIVATED` at speed 0 → bare `CALIBRATION`; `CALIBRATION` re-enters the bare root; `FAULT` ignores |
| `LOCK_EXIT`, `MOTOR_CMD_MODE`, `MOTOR_TUNING_MODE` | `Motor_Calibration_Exit` | `CALIBRATION` `NULL` | ignored | ignored | ignored | ignored | ignored | `CALIBRATION` → `PASSIVE`; others ignore |
| `Fault_Entry` on overvoltage | `Motor_StateMachine_SetFault` `RequestV0` | `FAULT` set | → `FAULT`, V0 above the freewheel limit, else float | → `FAULT` | → `FAULT` | → `FAULT` | → `FAULT` | `DEACTIVATED`, `CALIBRATION` → `FAULT`; `FAULT` accumulates |
| `Fault_InputFault` clear | `Motor_StateMachine_TryClearFaultAll` | `FAULT` clear | ignored | ignored | ignored | ignored | ignored | `FAULT` → `DEACTIVATED` once no flag remains |

## Sequences

Participants: the user input (analog poll every 1 ms, or a protocol write), the controller machine with its Traction leaf, the
motor machine. A note under the motor is its state after the step.

### Throttle press

```mermaid
sequenceDiagram
    actor U as User input
    participant C as Controller, Traction leaf
    participant M as Motor machine
    Note over C,M: DRIVE, motors PASSIVE, direction applied by Drive_Entry
    U->>C: throttle above 0 (SetThrottle, SetThrottleBrake, or the analog poll)
    C->>C: Traction_Input_PollCmdEdge, RELEASE to THROTTLE
    C->>C: APP_USER DRIVE_CMD, Drive_InputCmdStart, Traction_StartThrottleMode
    alt ThrottleMode SPEED or TORQUE
        C->>M: FEEDBACK_MODE SPEED_CURRENT or CURRENT (Motor_ApplyFeedbackMode, on a change)
        Note over M: PASSIVE, mode stored
        C->>M: PHASE_OUTPUT PWM (Motor_ActivateControl)
        alt direction set and feedback available
            Note over M: RUN, loops matched from Z or V0, T0 from Z
        else no feedback, speed 0, sensorless active
            Note over M: PASSIVE SENSORLESS_COAST, or OPEN_LOOP SENSORLESS_ALIGN
        else no feedback, no start-up chain
            Note over M: PASSIVE, PWM ignored
        end
    else ThrottleMode VOLTAGE
        C->>M: PHASE_OUTPUT Z
        Note over M: PASSIVE, no drive
    end
    loop each value while the command stays THROTTLE
        U->>C: throttle value
        C->>C: APP_USER THROTTLE_VALUE, Traction_ApplyThrottleValue
        C->>M: UserCmd = value / 2 (Motor_SetSpeedMotoringCmd_Norm or Motor_SetICmd_Norm), no state input
        Note over M: RUN, the loop follows UserCmd
    end
    opt throttle already held when DRIVE was entered
        Note over C: Drive_Entry synced DriveCmd, no edge, no PWM sent
        Note over M: PASSIVE until the throttle is released and pressed again
    end
```

### Brake to hold

```mermaid
sequenceDiagram
    actor U as User input
    participant C as Controller, Traction leaf
    participant M as Motor machine
    Note over C,M: DRIVE or NEUTRAL, motors RUN along the direction
    U->>C: brake above 0
    C->>C: edge to BRAKE (brake wins over throttle), APP_USER DRIVE_CMD, Traction_StartBrakeMode
    alt BrakeMode TORQUE
        C->>M: FEEDBACK_MODE CURRENT
        Note over M: RUN re-entered (Run_InputFeedbackMode), loops re-matched
        C->>M: PHASE_OUTPUT PWM
        Note over M: RUN, PWM is nothing in RUN
    else BrakeMode PASSIVE or VOLTAGE
        C->>M: PHASE_OUTPUT Z
        Note over M: PASSIVE within the freewheel limit, else INTERVENTION SPEED_ZERO
    end
    loop each brake value, TORQUE mode
        U->>C: brake value
        C->>C: APP_USER BRAKE_VALUE, ApplyBrakeValue, Traction_ApplyBrakeValue each motor
        C->>M: UserCmd = minus value / 2 (Motor_SetICmd_Norm)
        Note over M: RUN, generating current, decelerating
        C->>C: ApplyBrakeValue, any motor still Motor_IsSpeedAligned
        opt no motor advancing along its direction
            C->>M: INTERVENTION SPEED_ZERO (Traction_ApplyBrakeHold, Motor_ApplySpeedZero), repeats are nothing
            Note over M: INTERVENTION SPEED_ZERO, speed loop toward 0, UserCmd not read
        end
    end
    M->>M: SpeedZero_Next, speed 0
    Note over M: PASSIVE, floating within the freewheel limit. Later ApplySpeedZero ignored, PASSIVE maps no INTERVENTION
    U->>C: brake released
    C->>C: edge to RELEASE, APP_USER DRIVE_CMD
    alt in DRIVE
        C->>C: Traction_StartDriveZero by ZeroMode
        Note over M: PASSIVE holds. FLOAT floats, IZERO is ignored, ZERO writes UserCmd 0
    else in NEUTRAL
        C->>M: PHASE_OUTPUT Z
        Note over M: PASSIVE
    end
```

### Direction change at speed

```mermaid
sequenceDiagram
    actor U as Selector or host
    participant C as Controller, Traction leaf
    participant M as Motor machine
    Note over C,M: DRIVE FORWARD, motors RUN at speed
    alt selector, analog input mode, every 1 ms
        U->>C: Shifter_ResolveDirection, MotorController_Traction_CaptureDirection, on a change only
    else host, serial or CAN
        U->>C: MotorController_Traction_ApplyDirectionCmd, on every write
    end
    C->>C: Input.Direction = value, APP_USER DIRECTION, Drive_InputDirection
    alt value NULL (a selector with a neutral position, or the host)
        C->>C: to NEUTRAL
        C->>M: Neutral_Entry, PHASE_OUTPUT Z
        Note over M: PASSIVE within the freewheel limit, else INTERVENTION SPEED_ZERO. Direction retained
    else value REVERSE (reverse-only selector, or the host)
        C->>C: not every motor at speed 0, nothing applied
        Note over C,M: the edge is consumed, Input.Direction holds REVERSE, not re-applied at standstill. Motors RUN FORWARD
    end
    Note over C,M: NEUTRAL, motors rolling forward
    U->>C: REVERSE
    C->>C: Neutral_InputDirection, InputDriveDirection
    alt every motor already user REVERSE
        C->>C: to DRIVE
    else every motor at speed 0
        C->>M: DIRECTION (Motor_Table_ApplyUserDirection)
        Note over M: PASSIVE, direction stored, VLimit half-plane set
        C->>C: to DRIVE, Drive_Entry applies the direction again
    else a motor still moving
        C->>C: stays NEUTRAL
        opt host path
            C->>U: buzzer blink, MotorController_Traction_GetDirection is still NULL
        end
        Note over C,M: the selector sends no further edge, NEUTRAL holds after the stop until the selector moves again
    end
```

### E_STOP under ZeroMode ZERO and IZERO

```mermaid
sequenceDiagram
    actor U as Host, standby input, or ForceDisableControl
    participant C as Controller, Traction leaf
    participant M as Motor machine
    Note over C,M: DRIVE after a release edge, Traction_StartDriveZero ran
    alt ZeroMode ZERO, ThrottleMode SPEED
        Note over M: RUN, speed loop holds 0, bridge energized
    else ZeroMode ZERO, ThrottleMode TORQUE
        Note over M: RUN, zero current, coasting, speed may be nonzero
    else ZeroMode IZERO
        Note over M: INTERVENTION TORQUE_ZERO, zero torque, coasting, never leaves on its own
    else ZeroMode FLOAT or REGEN
        Note over M: PASSIVE within the freewheel limit, else INTERVENTION SPEED_ZERO
    end
    U->>C: STATE_CMD E_STOP, the same handler as STOP_MAIN (MotorController_InputStateCommand, unlocked)
    C->>C: DRIVE maps no STATE_CMD, Main_InputStateCmd, Common_InputStandby
    alt every motor at speed 0, not standby ready (held by ZERO or IZERO)
        C->>M: PHASE_OUTPUT Z to every motor (Motor_Table_ApplyControl)
        Note over M: RUN at 0 to PASSIVE, TORQUE_ZERO at 0 to PASSIVE, floating
        C->>C: standby ready, Common_EnterStandby, StandbyExitMode MANUAL
        C->>M: Standby_Entry, PHASE_OUTPUT Z, then Motor_Disable
        Note over M: DEACTIVATED, direction NULL
        Note over C: STANDBY, Traction leaf gone
    else a motor still moving (RUN at speed, SPEED_ZERO, TORQUE_ZERO rolling)
        C->>C: refused, MotBuzzer_StandbyError, stays DRIVE
        Note over M: unchanged, under the throttle or coasting, E_STOP did not release it
    end
    opt MotorController_ForceDisableControl instead
        U->>M: Motor_ForceDisableControl each, passive output and feedback state cleared now, then PHASE_OUTPUT Z
        Note over M: PASSIVE within the freewheel limit, INTERVENTION SPEED_ZERO above it
        U->>C: STOP_MAIN, accepted once every motor is PASSIVE at speed 0
    end
```

### Fault

```mermaid
sequenceDiagram
    participant T as 1 ms ISR
    participant C as Controller, main thread
    participant M as Motor machine
    actor U as Host
    alt motor fault (Overheat from Motor_Heat_Thread, SAFE_STOP watchdog, PositionSensor)
        M->>M: FAULT input or NEXT
        Note over M: FAULT, V0 above the freewheel limit, else floating until the speed passes it
        C->>C: 1 s poll, Motor_Table_IsAnyState FAULT, MotorController_SetFault Motors (locked)
    else VBus over or under voltage
        T->>C: MotorController_SetFault VBusLimit from the 1 ms ISR (locked, Fault_Entry runs in the ISR)
    else board monitors (1 s), RxLost (1 ms, serial mode)
        C->>C: MotorController_SetFault from main
    end
    C->>C: TransitionFault, FaultFlags OR, Fault_Entry
    Note over C: FAULT. DRIVE or NEUTRAL left without an exit action
    alt VBus overvoltage
        C->>M: FAULT set RequestV0 to every motor (Motor_StateMachine_SetFault)
        Note over M: FAULT, V0 above the freewheel limit, else floating
    else otherwise
        C->>M: PHASE_OUTPUT Z to every motor (Motor_ReleaseVZ)
        Note over M: RUN to PASSIVE or SPEED_ZERO, PASSIVE floats within the limit, CALIBRATION to PASSIVE, FAULT unchanged
    end
    C->>C: MotBuzzer_Periodic
    U->>C: MOT_USER_SYSTEM_CLEAR_FAULT (MotorController_ClearFault, locked)
    C->>C: Fault_InputFault, clear the bits, board check, MotorController_PollFaultFlags
    C->>M: Motor_StateMachine_TryClearFaultAll each (FAULT clear)
    Note over M: FAULT to DEACTIVATED once no flag remains, other states ignore the clear
    alt no flag left
        C->>C: to STANDBY, Standby_Entry
        C->>M: PHASE_OUTPUT Z, then Motor_Disable
        Note over M: DEACTIVATED from PASSIVE at speed 0. A motor still in SPEED_ZERO keeps decelerating
        U->>C: START_MAIN
        C->>C: ENTER_MAIN, NEUTRAL or DRIVE by the buffered direction and the motors' speed
    else a flag remains (Motors, or a monitor still faulting)
        Note over C: FAULT holds, the buzzer stops once the word reads 0
    end
```

## Not shown / open

- The lever path (`MotorController_Traction_SetLever`, `LEVER_DIRECTION` on engage, `LEVER_VALUE` in `THROUGH_ZERO`) is
  in the cross-layer table only.
- `E_STOP` runs the same handler as `STOP_MAIN`: it does not release a moving motor. `MotorController_ForceDisableControl`
  is the path that does, outside the controller machine.
- A reversal requested while rolling is consumed (`Drive_InputDirection`, `InputDriveDirection`) and not re-applied at
  standstill; the selector path sends only on a change, so a reverse-only selector held in `REVERSE` does not reverse until
  it is moved again. The host path re-sends on every write.
- The brake hold ends in `PASSIVE`, floating within the freewheel limit, not held: the hold is the `SPEED_ZERO` phase.
- A throttle held through `Drive_Entry` gives no edge; the motor stays `PASSIVE` until the throttle is released.
- The fault clear lands in `STANDBY` on `FaultFlags` alone; a motor still in `SPEED_ZERO` at that moment keeps decelerating
  under `STANDBY`.
- The `AUTO` standby exit (analog, no standby input) and `MotorController_StandbyPin` edges are drawn as `START_MAIN` /
  `STOP_MAIN` only.
- Which thread `Fault_Entry` runs on, and the lock it holds while it sends the motor inputs, is in
  [InputVsLoop.md](../../../Framework/StateMachine/InputVsLoop.md).
