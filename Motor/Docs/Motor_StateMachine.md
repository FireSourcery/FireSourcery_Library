# Motor State Machine

One [MSM_MACHINE] per motor. States run on the PWM thread; inputs arrive from the main thread. This layer knows
nothing of STANDBY, NEUTRAL or DRIVE — [MotorController_StateMachine.md](MotorController_StateMachine.md) covers the
layers above.

## States

[Motor_StateId_T]

```mermaid
stateDiagram-v2
    [*] --> MOTOR_STATE_INIT
    MOTOR_STATE_INIT --> MOTOR_STATE_DEACTIVATED: INIT_WAIT, no FaultFlags
    MOTOR_STATE_INIT --> MOTOR_STATE_FAULT: INIT_WAIT, FaultFlags

    MOTOR_STATE_DEACTIVATED --> MOTOR_STATE_PASSIVE: Motor_Enable
    MOTOR_STATE_PASSIVE --> MOTOR_STATE_DEACTIVATED: Motor_Disable, zero speed
    MOTOR_STATE_DEACTIVATED --> MOTOR_STATE_CALIBRATION: CALIBRATION, zero speed
    MOTOR_STATE_CALIBRATION --> MOTOR_STATE_PASSIVE: CALIBRATION exit, PHASE_OUTPUT Z

    MOTOR_STATE_PASSIVE --> MOTOR_STATE_RUN: PHASE_OUTPUT PWM, direction set, feedback available
    MOTOR_STATE_RUN --> MOTOR_STATE_RUN: FEEDBACK_MODE
    MOTOR_STATE_RUN --> MOTOR_STATE_PASSIVE: PHASE_OUTPUT Z / 0, within freewheel limit
    MOTOR_STATE_RUN --> MOTOR_STATE_INTERVENTION: PHASE_OUTPUT Z / 0 above freewheel limit, intervention cmd
    MOTOR_STATE_INTERVENTION --> MOTOR_STATE_RUN: PHASE_OUTPUT PWM, feedback available
    MOTOR_STATE_INTERVENTION --> MOTOR_STATE_PASSIVE: PHASE_OUTPUT Z / 0 within freewheel limit, SAFE_STOP at zero speed
    MOTOR_STATE_INTERVENTION --> MOTOR_STATE_FAULT: SAFE_STOP watchdog

    MOTOR_STATE_PASSIVE --> MOTOR_STATE_OPEN_LOOP: OPEN_LOOP, zero speed
    MOTOR_STATE_OPEN_LOOP --> MOTOR_STATE_PASSIVE: PHASE_OUTPUT Z / 0, OPEN_LOOP exit, DIRECTION NULL

    MOTOR_STATE_FAULT --> MOTOR_STATE_DEACTIVATED: FaultFlags cleared
    MOTOR_STATE_FAULT --> MOTOR_STATE_CALIBRATION: CALIBRATION
    note right of MOTOR_STATE_FAULT
        FaultSet from every state
        but INIT and FAULT
    end note
```

| State | Phase output | Feedback | Description |
|---|---|---|---|
| INIT | unchanged | Off | Waits for speed and heat sensors, validates config and sensor calibration |
| DEACTIVATED | Float | Off, BEMF angle captured | Quiescent gateway. Direction NULL, config writable |
| PASSIVE | Float within the freewheel speed limit, else V0 | Off, BEMF angle captured for resume | Freewheel or hold |
| RUN | PWM | Current or voltage loop, optional speed loop, by [Motor_FeedbackMode_T] | Active control. Entry matches loop state to the output for a bumpless resume |
| INTERVENTION | PWM | Current loop regardless of FeedbackMode | Motor-level supervisor, below |
| OPEN_LOOP | V0, PWM in sub-states | Cleared | Align, start-up, open-loop run |
| CALIBRATION | V0, PWM in sub-states | Cleared | Sensor and parameter calibration. Config writable |
| FAULT | V0 on RequestV0 or above the freewheel limit, else Float | Off | Latched [Motor_FaultFlags_T]. Config writable |

The freewheel speed limit is where floating the phases is still safe. Above it, a release actively ramps down instead.

## Inputs

[Motor_StateInput_T]

| Input | Value | Effect |
|---|---|---|
| FAULT | [Motor_FaultCmd_T] set / clear deltas | Set enters FAULT from any state but INIT. Clear in FAULT returns to DEACTIVATED once every flag clears |
| PHASE_OUTPUT | [Phase_VOutMode_T] | PWM runs from PASSIVE or INTERVENTION. Z / 0 releases RUN, INTERVENTION, OPEN_LOOP; Z releases CALIBRATION |
| FEEDBACK_MODE | [Motor_FeedbackMode_T] | Stored in DEACTIVATED, PASSIVE, OPEN_LOOP, INTERVENTION. RUN re-enters to rematch its loops |
| DIRECTION | [Motor_Direction_T] | Applied in DEACTIVATED, and at zero speed in PASSIVE, OPEN_LOOP, CALIBRATION. Ignored in RUN, INTERVENTION, FAULT |
| OPEN_LOOP | sub-state pointer | Enters from PASSIVE at zero speed. NULL exits to PASSIVE |
| CALIBRATION | sub-state pointer | Enters from DEACTIVATED at zero speed, or from FAULT. NULL exits to PASSIVE |
| INTERVENTION | sub-state pointer | Enters from RUN or INTERVENTION. A target already on the active branch is a no-op. SAFE_STOP rejects. Sent by Motor_ApplyTorqueZero, Motor_ApplySpeedZero, Motor_ApplySafeStop |

Transition commands act only from a named start state.

| Function | From | To |
|---|---|---|
| Motor_Enable | DEACTIVATED | PASSIVE |
| Motor_Disable | PASSIVE, zero speed | DEACTIVATED |

[Motor_StateAction_T] are per-state hooks, not transitions: ON_SPEED runs the speed loop tick, ON_I_LIMITS re-resolves
current limits.

### Direction

Direction does not drive transitions. A direction is required to leave PASSIVE for RUN, but setting one does not
start the motor, so a layer above can store a direction while holding its own safe state. DEACTIVATED resets it to
NULL. Setting the direction also limits the FOC voltage output to that direction. User FORWARD / REVERSE map to
CW / CCW through `Config.DirectionForward` (`Motor_ApplyUserDirection`).

## Intervention

```mermaid
stateDiagram-v2
    state "MOTOR_STATE_RUN" as RUN
    state "MOTOR_STATE_PASSIVE" as PASSIVE
    state "MOTOR_STATE_FAULT" as FAULT

    state MOTOR_STATE_INTERVENTION {
        INTERVENTION_STATE_TORQUE_ZERO
        state INTERVENTION_STATE_SPEED_ZERO {
            INTERVENTION_STATE_SAFE_STOP
        }
        INTERVENTION_STATE_SPEED_ZERO --> INTERVENTION_STATE_TORQUE_ZERO: Motor_ApplyTorqueZero
        INTERVENTION_STATE_TORQUE_ZERO --> INTERVENTION_STATE_SPEED_ZERO: Motor_ApplySpeedZero, release above freewheel limit
    }

    RUN --> INTERVENTION_STATE_TORQUE_ZERO: Motor_ApplyTorqueZero
    RUN --> INTERVENTION_STATE_SPEED_ZERO: Motor_ApplySpeedZero, release above freewheel limit
    RUN --> INTERVENTION_STATE_SAFE_STOP: Motor_ApplySafeStop
    MOTOR_STATE_INTERVENTION --> INTERVENTION_STATE_SAFE_STOP: Motor_ApplySafeStop
    MOTOR_STATE_INTERVENTION --> RUN: PHASE_OUTPUT PWM, except SAFE_STOP
    MOTOR_STATE_INTERVENTION --> PASSIVE: release within freewheel limit
    INTERVENTION_STATE_SAFE_STOP --> PASSIVE: zero speed
    INTERVENTION_STATE_SAFE_STOP --> FAULT: watchdog
```

| Sub-state | Control | Resume |
|---|---|---|
| TORQUE_ZERO (ZTC) | Current loop, generating current only | PHASE_OUTPUT PWM |
| SPEED_ZERO | Speed loop to zero speed | PHASE_OUTPUT PWM |
| SAFE_STOP (SS1) | SPEED_ZERO's loop, watchdog to FAULT | rejected |

## Sub-States

| Sub-state | Parent | Entered by | Exits |
|---|---|---|---|
| OPEN_LOOP_STATE_ANGLE_ALIGN | OPEN_LOOP | Motor_OpenLoop_SetAngleAlign | OPEN_LOOP inputs |
| OPEN_LOOP_STATE_START_UP_ALIGN | OPEN_LOOP | Motor_OpenLoop_StartRunChain, direction set | OPEN_LOOP_STATE_RUN after `AlignTime_Cycles` |
| OPEN_LOOP_STATE_RUN | OPEN_LOOP | START_UP_ALIGN | OPEN_LOOP inputs |
| CALIBRATION_STATE_ELECTRICAL | CALIBRATION | Motor_Calibration_StartElectrical | CALIBRATION once Rs, Ld, Lq are measured |

ADC and sensor calibrations mount their own CALIBRATION sub-states in `Motor_Analog`, `Motor_Encoder`, `Motor_Hall`
and `Motor_SinCos`. Encoder homing is described in [Encoder_Reference.md](../../Transducer/Encoder/Encoder_Reference.md).
