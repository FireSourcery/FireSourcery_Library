# MotorController State Machine

Hierarchical state machines in three layers. The MotorController machine holds the system states and the app's
sub-states; each motor runs its own machine, detailed in [Motor_StateMachine.md](Motor_StateMachine.md).
[Conventional_StateMachine.md](Conventional_StateMachine.md) is the industry reference.

## Layers

```mermaid
flowchart TB
    subgraph MCSM["MCSM_MACHINE · main thread, 1 ms"]
        APP["App sub-states<br/>[MotorController_App_T] e.g. Traction"]
        ROOT["Root states<br/>[MotorController_StateId_T]"]
        APP -- "mounted under MC_STATE_MAIN" --- ROOT
    end
    MOT["MSM_MACHINE per motor · PWM thread<br/>[Motor_StateId_T]"]
    ROOT -- "Enable / Disable, CALIBRATION, FAULT" --> MOT
    APP -- "PHASE_OUTPUT, DIRECTION, FEEDBACK_MODE, cmd value" --> MOT
    MOT -. "state, speed, FAULT polled" .-> ROOT
```

| Layer | Owns | Does not know |
|---|---|---|
| Motor | commutation, feedback loops, speed estimation, motor faults, direction CW / CCW / NULL | STANDBY, NEUTRAL, DRIVE |
| MotorController | boot, multi-motor coordination, system faults, STANDBY, LOCK (calibration, NVM), motors DEACTIVATED ⇄ PASSIVE | app states and inputs |
| App | NEUTRAL / DRIVE, FORWARD / REVERSE, throttle / brake mapping, release behavior, direction-change checks, motors PASSIVE ⇄ RUN | |

The app is not a separate machine. Its states are sub-states of [MC_STATE_MAIN], run by [MCSM_MACHINE] on the same
thread, and its inputs arrive as [MC_STATE_INPUT_APP_USER].

Each [State_T] maps input ids to handlers in its transition table. An empty slot defers to the parent state.

## Design Rules

The root machine does not know app-specific states or inputs. [MC_STATE_STANDBY] belongs to the root;
an app mounts its sub-states under [MC_STATE_MAIN] through [MotorController_App_T] `ENTER_MAIN`.

- A new state is justified by a different safety envelope, not a different purpose.
- One-shot automated procedures get a state; interactive parameter adjustment does not.
- Orthogonal concerns get orthogonal mechanisms — e.g. what control structure is running vs. what input source is connected.
- Lock is for operations the operator must commit to and acknowledge as risky, outside normal operation.
- The operator use case does not conflate with the system's state.

## Root States

[MotorController_StateId_T], with the sub-states the root defines itself.

```mermaid
stateDiagram-v2
    [*] --> MC_STATE_INIT
    MC_STATE_INIT --> MC_STATE_STANDBY: INIT_WAIT, no FaultFlags
    MC_STATE_INIT --> MC_STATE_FAULT: INIT_WAIT, FaultFlags

    MC_STATE_STANDBY --> MC_STATE_MAIN: START_MAIN, or StandbyExitMode AUTO
    MC_STATE_MAIN --> MC_STATE_STANDBY: STOP_MAIN / E_STOP
    MC_STATE_STANDBY --> MC_STATE_LOCK: LOCK_ENTER

    state MC_STATE_MAIN {
        state "App sub-states" as APP
        [*] --> APP: App ENTER_MAIN
        [*] --> MC_STATE_MAIN_MOTOR_CMD: MC_APP_MOTOR_CMD ENTER_MAIN
    }

    state MC_STATE_LOCK {
        [*] --> MC_LOCK_SUB_ID_NONE
        MC_LOCK_SUB_ID_NONE --> MC_STATE_LOCK_CALIBRATE_ADC: CALIBRATE_ADC
        MC_STATE_LOCK_CALIBRATE_ADC --> MC_LOCK_SUB_ID_NONE: 2000 ms
    }
    MC_STATE_LOCK --> MC_STATE_STANDBY: LOCK_EXIT
    MC_STATE_LOCK --> MC_STATE_FAULT: LOCK_EXIT, motors active
    MC_STATE_LOCK --> MC_STATE_MAIN: MOTOR_CMD_MODE
    MC_STATE_LOCK --> MC_STATE_MOTOR_TUNING: MOTOR_TUNING_MODE

    MC_STATE_MOTOR_TUNING --> MC_STATE_STANDBY: STOP_MAIN / E_STOP
    MC_STATE_MOTOR_TUNING --> MC_STATE_MOTOR_TUNING: MOTOR_TUNING_RESET
    MC_STATE_MOTOR_TUNING --> MC_STATE_LOCK: LOCK_ENTER

    MC_STATE_FAULT --> MC_STATE_STANDBY: FaultFlags cleared
    note right of MC_STATE_FAULT
        FaultSet from every state
        but INIT and FAULT
    end note
```

- STOP_MAIN / E_STOP leaves MAIN and MOTOR_TUNING only once every motor is DEACTIVATED, or PASSIVE at zero speed. Otherwise the buzzer sounds and the state holds.
- LOCK commands are accepted at the bare LOCK only, not from a sub-state. MOTOR_CMD_MODE lands on [MC_STATE_MAIN_MOTOR_CMD].
- LOCK_EXIT and fault clear always land in STANDBY, never live MAIN.

| [MotorController_StateInput_T] | Handled by |
|---|---|
| FAULT | every state but INIT. [MotorController_FaultCmd_T] set / clear deltas |
| LOCK | STANDBY, MOTOR_TUNING, LOCK; FAULT accepts NVM save only. [MotorController_LockId_T] |
| STATE_CMD | STANDBY, MAIN, MOTOR_TUNING. [MotorController_StateCmd_T] START_MAIN, STOP_MAIN, E_STOP |
| MOTOR_CMD | [MC_STATE_MAIN_MOTOR_CMD] |
| APP_USER | app sub-states only |

Each state bounds the motor states it allows.

| State | Motor states | Use |
|---|---|---|
| INIT | INIT, DEACTIVATED | Boot. Waits for sensors, verifies board and VBus config; any motor not DEACTIVATED is a fault |
| STANDBY | DEACTIVATED | Parking brake, standby. Phases off; app inputs buffered, not applied |
| MAIN | PASSIVE, RUN, INTERVENTION, OPEN_LOOP | Normal operation. Entry enables the motors |
| MOTOR_TUNING | PASSIVE, RUN | Live PID tuning. Exit to STANDBY discards the live gains; LOCK keeps them for NVM save |
| LOCK | DEACTIVATED, CALIBRATION | Calibration, NVM save, reboot |
| FAULT | FAULT on overvoltage, otherwise released to Z | Error handling. Buzzer until cleared |

## App Sub-States: Traction

[Traction_StateId_T], mounted under [MC_STATE_MAIN]. Inputs are [Traction_StateInput_T], read from the buffered
[Traction_Input_T].

```mermaid
stateDiagram-v2
    state "MC_STATE_STANDBY" as STANDBY
    state "STATE_NEUTRAL" as NEUTRAL
    state "STATE_DRIVE" as DRIVE

    STANDBY: Motors DEACTIVATED, Output Float
    STANDBY: Parking brake, standby

    STANDBY --> MC_STATE_MAIN: START_MAIN
    MC_STATE_MAIN --> STANDBY: STOP_MAIN / E_STOP

    state MC_STATE_MAIN {
        [*] --> NEUTRAL: Direction NULL, or motors moving
        [*] --> DRIVE: Direction FORWARD / REVERSE
        DRIVE --> NEUTRAL: Direction NULL
        NEUTRAL --> DRIVE: Direction FORWARD / REVERSE, motors in that direction or stopped

        NEUTRAL: Motors PASSIVE, Direction retained
        NEUTRAL: Output Float, PWM on Brake
        NEUTRAL: Clutch disengaged, ready to engage

        DRIVE: Motors RUN, Direction FORWARD / REVERSE
        DRIVE: Output PWM on Throttle / Brake, ZeroMode on Release
    }
```

| State | Motor direction | Motor states | Accepted inputs |
|---|---|---|---|
| STANDBY | NULL | DEACTIVATED | none applied, all buffered |
| STATE_NEUTRAL | retained | PASSIVE, RUN while braking | Direction, Brake. Throttle has no effect |
| STATE_DRIVE | FORWARD / REVERSE | PASSIVE, RUN, INTERVENTION | Direction NULL, Throttle, Brake, Lever |

Direction is buffered in [Traction_Input_T] independent of state. FORWARD / REVERSE are the user frame, mapped to
CW / CCW by the motor's `DirectionForward` calibration.

### Direction Changes

| From | Input | Motors | Result |
|---|---|---|---|
| STANDBY | START_MAIN | moving | NEUTRAL |
| STANDBY | START_MAIN, Direction NULL | stopped | NEUTRAL |
| STANDBY | START_MAIN, Direction FORWARD / REVERSE | stopped | DRIVE, direction applied on entry |
| NEUTRAL | FORWARD / REVERSE | already in that direction | DRIVE |
| NEUTRAL | FORWARD / REVERSE | stopped | apply direction, DRIVE |
| NEUTRAL | FORWARD / REVERSE | moving the other way | stay NEUTRAL until stopped |
| DRIVE | NULL | any | NEUTRAL, direction retained |
| DRIVE | FORWARD / REVERSE | any | no change |
| DRIVE | Lever engaged from release | stopped | apply the lever's direction |

A direction command that does not take effect blinks the buzzer. The Motor checks zero speed again before applying
a direction.

### Throttle, Brake, Release

Edges of [Traction_Cmd_T] start a mode; values in between set the command.

| Cmd | Motor |
|---|---|
| THROTTLE | Feedback mode by [Traction_ThrottleMode_T], PHASE_OUTPUT PWM: PASSIVE → RUN |
| BRAKE | By [Traction_BrakeMode_T]: current feedback, PWM, opposing current cmd. INTERVENTION SPEED_ZERO holds zero speed once no motor advances |
| RELEASE | By [Traction_ZeroMode_T], below |

| [Traction_ZeroMode_T] | Motor |
|---|---|
| FLOAT | PHASE_OUTPUT Z: RUN → PASSIVE within the freewheel speed limit, else INTERVENTION SPEED_ZERO |
| IZERO | RUN → INTERVENTION TORQUE_ZERO |
| REGEN | not implemented |
| ZERO | setpoint zero, no state change |

## Sequences

### Startup

```mermaid
sequenceDiagram
    actor U as User
    participant MC as MotorController
    participant T as Traction
    participant M as Motor
    Note over MC,M: MC_STATE_INIT, MOTOR_STATE_INIT
    M->>M: MOTOR_STATE_MACHINE_INIT_WAIT, verify config and sensor
    M->>M: DEACTIVATED, or FAULT
    MC->>MC: MC_STATE_MACHINE_INIT_WAIT, verify board, VBus, motors DEACTIVATED
    MC->>MC: STANDBY, or FAULT
    U->>MC: START_MAIN, or StandbyExitMode AUTO
    MC->>M: Main_Entry enables motors, DEACTIVATED to PASSIVE
    MC->>T: ENTER_MAIN
    T->>T: NEUTRAL, or DRIVE by buffered Direction
```

### Drive

```mermaid
sequenceDiagram
    actor U as User
    participant T as Traction
    participant M as Motor
    Note over T,M: STATE_NEUTRAL, MOTOR_STATE_PASSIVE
    U->>T: Direction FORWARD
    alt motors stopped, or already FORWARD
        T->>M: user direction FORWARD
        T->>T: DRIVE
    else motors moving REVERSE
        T->>T: stay NEUTRAL, buzzer
    end
    U->>T: Throttle edge
    T->>M: FEEDBACK_MODE, PHASE_OUTPUT PWM
    M->>M: PASSIVE to RUN
    U->>T: Throttle value
    T->>M: speed or current cmd
    U->>T: Release edge
    alt ZeroMode FLOAT
        T->>M: PHASE_OUTPUT Z
        M->>M: PASSIVE, or INTERVENTION SPEED_ZERO above freewheel limit
    else ZeroMode IZERO
        T->>M: Motor_ApplyTorqueZero
        M->>M: INTERVENTION TORQUE_ZERO
    end
```

### Fault

```mermaid
sequenceDiagram
    actor U as User
    participant MC as MotorController
    participant M as Motor
    M->>M: fault detected, MOTOR_STATE_FAULT
    MC->>M: poll, any motor FAULT
    MC->>MC: FaultFlags.Motors, MC_STATE_FAULT
    alt overvoltage
        MC->>M: RequestV0
    else
        MC->>M: release to Z, exit calibration
    end
    U->>MC: clear faults
    MC->>MC: re-poll fault conditions
    MC->>M: clear motor faults, FAULT to DEACTIVATED
    MC->>MC: all clear, STANDBY
```
