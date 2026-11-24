# RotorSensor interface

The rotor angle sensor interface `RotorSensor_T`: its vtable, state and registry, the four sensors that implement it (Hall, Encoder, SinCos, Sensorless), what each does in each operation, and the order the motor calls them. Board-agnostic; which sensors a board compiles and wires is the board's sheet. The sensorless entry in depth: [Sensorless.md](Sensorless/Sensorless.md).

## Interface, state, registry, implementations

```mermaid
classDiagram
    direction LR

    class RotorSensor_VTable_T {
        <<interface>>
        RotorSensor_Proc_T CAPTURE_ANGLE
        RotorSensor_Proc_T CAPTURE_STATOR
        RotorSensor_Proc_T CAPTURE_SPEED
        RotorSensor_Test_T IS_FEEDBACK_AVAILABLE
        RotorSensor_Proc_T ZERO_INITIAL
        RotorSensor_Proc_T INIT
        RotorSensor_Proc_T INIT_UNITS
        RotorSensor_Test_T VERIFY_CALIBRATION
    }

    class RotorSensor_T {
        <<table entry, const>>
        RotorSensor_VTable_T P_VTABLE
        RotorSensor_State_T P_STATE
    }

    class RotorSensor_State_T {
        <<Motor_Context_T.SensorState, one per motor>>
        Angle_T AngleSpeed
        UnitRef PolePairs AngleFreqBase SpeedPuRef
        accum32_t Speed_Pu
        angle16_t MechanicalAngle
    }

    class RotorSensor_UnitRef_T {
        <<argument of RotorSensor_InitUnitsFrom>>
        uint32_t PollingFreq
        uint8_t PolePairs
        angle_freq_t AngleFreqBase
    }

    class RotorSensor_Table_T {
        <<Motor_T.SENSOR_TABLE, const>>
        RotorSensor_T EMPTY
        Hall_RotorSensor_T HALL
        Encoder_RotorSensor_T ENCODER
        SinCos_RotorSensor_T SIN_COS
        Sensorless_Sensor_T SENSORLESS
        RotorSensor_Of(id) RotorSensor_T
        RotorSensor_Validate(p_sensor, id) bool
    }

    class RotorSensor_Id_T {
        <<enumeration>>
        SENSORLESS 0
        HALL 1
        ENCODER 2
        SIN_COS 3
    }

    class Hall_RotorSensor_T {
        RotorSensor_T BASE
        Hall_T HALL
        PulseTimer_T TIMER
        AngleCounter_T P_COUNTER
        uint32_t POLLING_FREQ
    }

    class Encoder_RotorSensor_T {
        RotorSensor_T BASE
        Encoder_T ENCODER
    }

    class SinCos_RotorSensor_T {
        RotorSensor_T BASE
        SinCos_T SIN_COS
        SinCos_Analog_T ANALOG
        uint32_t POLLING_FREQ
    }

    class Sensorless_Sensor_T {
        RotorSensor_T BASE
        EmfObserver_T P_OBSERVER
        FOC_T P_FOC
        Motor_Electrical_T P_ELECTRICAL
        EmfObserver_Config_T P_NVM_CONFIG
    }

    class MOTOR_SENSOR_VTABLE_EMPTY {
        <<RotorSensor.c>>
        every Proc a no-op
        every Test false
    }

    RotorSensor_T --> RotorSensor_VTable_T : P_VTABLE
    RotorSensor_T --> RotorSensor_State_T : P_STATE, shared by every entry of one motor
    RotorSensor_State_T ..> RotorSensor_UnitRef_T : UnitRef derived by RotorSensor_InitUnitsFrom

    RotorSensor_Table_T *-- RotorSensor_T : EMPTY
    RotorSensor_Table_T *-- Hall_RotorSensor_T : HALL
    RotorSensor_Table_T *-- Encoder_RotorSensor_T : ENCODER, MOTOR_SENSOR_ENCODER_ENABLE
    RotorSensor_Table_T *-- SinCos_RotorSensor_T : SIN_COS, MOTOR_SENSOR_SIN_COS_ENABLE
    RotorSensor_Table_T *-- Sensorless_Sensor_T : SENSORLESS, MOTOR_SENSOR_SENSORLESS_ENABLE
    RotorSensor_Table_T ..> RotorSensor_Id_T : Config.SensorMode selects, on reboot

    Hall_RotorSensor_T *-- RotorSensor_T : BASE
    Encoder_RotorSensor_T *-- RotorSensor_T : BASE
    SinCos_RotorSensor_T *-- RotorSensor_T : BASE
    Sensorless_Sensor_T *-- RotorSensor_T : BASE

    RotorSensor_VTable_T <|.. Hall_RotorSensor_T : HALL_VTABLE, static in Hall_Sensor.h
    RotorSensor_VTable_T <|.. Encoder_RotorSensor_T : ENCODER_VTABLE
    RotorSensor_VTable_T <|.. SinCos_RotorSensor_T : SIN_COS_VTABLE
    RotorSensor_VTable_T <|.. Sensorless_Sensor_T : SENSORLESS_SENSOR_VTABLE
    RotorSensor_VTable_T <|.. MOTOR_SENSOR_VTABLE_EMPTY : EMPTY and every unknown id
```

- `RotorSensor_T` is the base: first member of every sensor, no back pointer. `P_STATE` is the motor's `SensorState`, so every entry of one motor publishes into the same `AngleSpeed` and `Speed_Pu`; only the active sensor is ever called.
- No NULL checks: every slot of a vtable is filled, a sensor that reads the rotor directly sets `CAPTURE_STATOR` to `RotorSensor_CaptureStatorNone`.
- `RotorSensor_Of` returns `EMPTY` for an id whose entry is not compiled: feedback never available, no start-up chain, the motor stays in PASSIVE on a PWM request.

## What the motor calls, in order

```mermaid
flowchart LR
    subgraph PWM["Motor_PWM_Thread, every control cycle, Motor_Thread.h"]
        direction LR
        CA["Motor_CaptureSensor<br/>RotorSensor_CaptureAngle"] --> SM["_Motor_StateMachine_Thread<br/>the state's LOOP and NEXT, the FOC's take and write"]
        SM --> CS["Motor_CaptureStator<br/>RotorSensor_CaptureStator<br/>compiled under MOTOR_SENSOR_SENSORLESS_ENABLE"]
        CS --> PS["Motor_PollSpeed, SPEED_TIMER at 1 ms<br/>RotorSensor_CaptureSpeed, then the ON_SPEED action"]
        PS --> TB["ControlTimerBase++"]
    end
```

| Caller | Operation | Where |
|---|---|---|
| `Motor_Init` | `RotorSensor_Of(Config.SensorMode)` → `p_ActiveSensor`, then `RotorSensor_Init` → `INIT`, `_RotorSensor_Reset` | Motor.c |
| `Motor_Reset` → `Motor_InitUnits` | `RotorSensor_InitUnitsFrom` with PollingFreq = `MOTOR_CONTROL_FREQ`, PolePairs, AngleFreqBase → derives `UnitRef.SpeedPuRef`, then `INIT_UNITS` | Motor.c |
| `Motor_SetDirection` | `ZERO_INITIAL` | Motor.c |
| `Init_Next` | `VERIFY_CALIBRATION` → `FaultFlags.PositionSensor`; false sends INIT to FAULT | Motor_StateMachine.c |
| `Passive_InputControl`, PWM request | `IS_FEEDBACK_AVAILABLE` → RUN; else at speed 0 `Motor_Sensor_StartUpState` (the sensor's chain, NULL where none) | Motor_StateMachine.c, Motor_Sensor.c |
| RUN and INTERVENTION `NEXT` | `_Motor_TransitionFeedbackLost`: `IS_FEEDBACK_AVAILABLE`; lost → `Motor_Sensor_FallbackState` under `MOTOR_OPEN_LOOP_FALLBACK_ENABLE`, else PASSIVE within the freewheel limit, FAULT above it | Motor_StateMachine.c |
| `Intervention_InputControl`, PWM request | `IS_FEEDBACK_AVAILABLE` → RUN | Motor_StateMachine.c |
| Sensor chains | `ZERO_INITIAL` at `Align_Entry` (sensorless); `IS_FEEDBACK_AVAILABLE` in `Coast_Next`, `StartUp_Next`; `VERIFY_CALIBRATION` at `Calibration_End` (SinCos) | Motor_Sensorless.c, Motor_SinCos.c |
| Protocol | `_Motor_Var_Rotor_Get`: electrical angle, delta, speed, mechanical angle, direction from `P_STATE` | RotorSensor.h |

## Operation by sensor

| Operation | Hall (`Hall_Sensor.h`) | Encoder (`Encoder_Sensor.c`) | SinCos (`SinCos_Sensor.c`) | Sensorless (`Sensorless_Sensor.c`) | EMPTY |
|---|---|---|---|---|---|
| `INIT` | `Hall_Init`, `PulseTimer_Init`, extended watch-stop 500 ms | `Encoder_MT_Init_InterruptQuadrature`, `Encoder_EnableQuadratureMode` | `SinCos_Init` | `EmfObserver_Init` from `P_NVM_CONFIG`; per-tick state cleared | no-op |
| `INIT_UNITS` | `AngleCounter_InitFrom`: 6 counts per electrical revolution at `POLLING_FREQ`, electrical base (`_ElSpeed`; a `_MechSpeed` variant exists, not in the vtable) | encoder `Config.AngleFreqBase` = base / PolePairs (mechanical revolution), `Encoder_MT_InitUnits` | `SinCos_Config_SetAngleRatio(PolePairs)` | `EmfObserver_InitWinding(Rs, Lq)` while the model is valid; invalid keeps the prior winding | no-op |
| `CAPTURE_ANGLE` (control tick) | on a Hall edge: `PulseTimer_CaptureCount`, snap the counter to the sector angle, sector limits; else `AngleCounter_Interpolate`; publish `Angle` | `MechanicalAngle` = count angle × direction ref; `Angle` = × PolePairs; no interpolation | ADC sin, cos → `SinCos_CaptureAngle` → electrical; `Delta` = wrapped difference; `MechanicalAngle` | `AnglePll_Predict` (θ̂ += ω̂); publish `Angle`, `Delta` through the 2-sample filter | no-op |
| `CAPTURE_STATOR` (after the state machine) | none | none | none | `EmfObserver_Eemf_ProcHeld(Id, Iq)` with the held v(k−1), then `EmfObserver_HoldV(Vα, Vβ)` | no-op |
| `CAPTURE_SPEED` (1 ms) | `PulseTimer_CaptureFreq` (decays without pulses), `AngleCounter_ResolveAngleDelta` → `Delta`, `Speed_Pu`, both 2-sample filtered | `Encoder_MT_CaptureFreqD`, `ResolveInterpolation`, `Speed_Pu`, `Angle_CaptureSpeed_Pu`, `Encoder_PollIndexCapture` | `Angle_ResolveSpeed_Pu` from `Delta`, 2-sample filtered | `Angle_ResolveSpeed_Pu`, 0 while `EmfObserver_IsIndeterminate` (|ê| at the floor); 2-sample filtered | no-op |
| `IS_FEEDBACK_AVAILABLE` | true | `Encoder_IsAligned` (ALIGNED or better) | true | `EmfObserver_IsLocked` | false |
| `ZERO_INITIAL` | `Hall_ZeroInitial`, zero the counter, seed it with the sector centre and its limits, `PulseTimer_SetInitial` | `Encoder_MT_SetInitial`: pulse and speed state only, the position reference survives | one capture, `Angle` set, `Angle_StopDelta`, `Speed_Pu` 0 | `EmfObserver_Reset`: per-sample state; config and winding kept | no-op |
| `VERIFY_CALIBRATION` | `Hall_IsCalibrationTableValid` | true, stub | `Max_Adcu > Min_Adcu` and `ElectricalRotationsRatio > 0` | `K_smo != 0`, `LpfCoef != 0`, `Winding.G != 0` | false |
| Start-up chain (`Motor_Sensor_StartUpState`) | NULL | NULL from the table; `Motor_Encoder_GetStartUpState` returns START_UP but is reached only by the `Motor_Encoder_StartUpChain` command | NULL | `Motor_Sensorless_StartUpState`: SENSORLESS_COAST under PASSIVE until the coast time, then SENSORLESS_ALIGN → SENSORLESS_START_UP → RUN | NULL |
| Fallback (`Motor_Sensor_FallbackState`) | NULL | NULL | NULL | SENSORLESS_FALLBACK while `EmfObserver_IsWeak` | NULL |
| Calibration sub-state (under CALIBRATION) | CALIBRATION_STATE_HALL: six current-aligned vectors A, −C, B, −A, C, −B, `Hall_CalibrateState` at each `AlignTime`; an invalid table → FAULT | STATE_ENCODER_HOMING_ALIGN → STATE_ENCODER_HOMING: align, then index search; timeout → FAULT | CALIBRATION_STATE_SIN_COS: V-align A → `AngleOffset`, −C → `CcwPositive`; `VERIFY_CALIBRATION` at the end | none, tuning comes from NVM | none |
| Open-loop sub-states (under OPEN_LOOP) | none | ALIGN → VALIDATE_ALIGN → VALIDATE_CLOSED_LOOP (commands); START_UP → START_UP_ALIGN → START_UP_VALIDATE_ALIGN → START_UP_VALIDATE_CLOSED_LOOP → RUN | none | SENSORLESS_ALIGN, SENSORLESS_START_UP, SENSORLESS_FALLBACK | none |
| Command entry | `Motor_Hall_Calibrate`, `Motor_Hall_Cmd` | `Motor_Encoder_StartHoming`, `_StartAlignZero`, `_StartValidateAlign`, `_StartValidateClosedLoop`, `_StartUpChain` | `Motor_SinCos_Calibrate`, `_Cmd` | `Motor_Sensorless_StartRunChain` (ALIGN at once, no coast) | |
| Peripheral collaborators | `Hall_T` pins, `PulseTimer_T` on a board timer, `AngleCounter_T` | `Encoder_T`: counter or emulated pins A, B, Z | `SinCos_Analog_T`: 2 ADC conversions, marked by `SinCos_RotorSensor_MarkAnalog` | none: `FOC_T` and `Motor_Electrical_T` of the same motor | |

## Sensor-owned sub-states and their roots

```mermaid
stateDiagram-v2
    direction LR
    state PASSIVE {
        SENSORLESS_COAST
    }
    state OPEN_LOOP {
        SENSORLESS_ALIGN --> SENSORLESS_START_UP : AlignTime
        SENSORLESS_FALLBACK
        state "START_UP (Motor_Encoder.c)" as ENC_START_UP
        state "START_UP_ALIGN" as ENC_SU_ALIGN
        state "START_UP_VALIDATE_ALIGN" as ENC_SU_VALIDATE
        state "START_UP_VALIDATE_CLOSED_LOOP" as ENC_SU_CLOSED
        ENC_START_UP --> ENC_SU_ALIGN
        ENC_SU_ALIGN --> ENC_SU_VALIDATE : align settled
        ENC_SU_VALIDATE --> ENC_SU_CLOSED : encoder tracks
    }
    state CALIBRATION {
        CALIBRATION_STATE_HALL
        CALIBRATION_STATE_SIN_COS
        STATE_ENCODER_HOMING_ALIGN --> STATE_ENCODER_HOMING : align settled
    }
    PASSIVE --> SENSORLESS_COAST : PWM request, speed 0, coast time not elapsed
    PASSIVE --> SENSORLESS_ALIGN : PWM request, speed 0, coast time elapsed
    PASSIVE --> ENC_START_UP : Motor_Encoder_StartUpChain, speed 0
    SENSORLESS_COAST --> SENSORLESS_ALIGN : coast time elapsed
    SENSORLESS_COAST --> RUN : lock
    SENSORLESS_COAST --> PASSIVE : direction cleared, or Z or 0 requested
    SENSORLESS_START_UP --> RUN : lock, FOC re-Parked at the observer angle
    RUN --> SENSORLESS_FALLBACK : lock lost, observer still weak-tracking
    SENSORLESS_FALLBACK --> RUN : lock
    ENC_SU_CLOSED --> RUN : timer, speed and Vq signs agree
    ENC_SU_VALIDATE --> FAULT : encoder static under open loop
    ENC_SU_CLOSED --> FAULT : speed against Vq
    CALIBRATION_STATE_HALL --> CALIBRATION : six steps, table valid
    CALIBRATION_STATE_HALL --> FAULT : table invalid
    CALIBRATION_STATE_SIN_COS --> CALIBRATION : three steps, PositionSensor flag from VERIFY_CALIBRATION
    STATE_ENCODER_HOMING --> CALIBRATION : index found
    STATE_ENCODER_HOMING --> FAULT : travel budget spent
```

The command-only encoder states ALIGN, VALIDATE_ALIGN and VALIDATE_CLOSED_LOOP mirror the START_UP_* chain and return to OPEN_LOOP instead of RUN. Every sub-state reports its root's id (`PATH_ID.Depth0`); sub-state ids are not allocated.

## Not shown / open

- `RotorSensor_Table.h` turns `MOTOR_SENSOR_HALL_ENABLE` on when neither it nor `MOTOR_SENSOR_ENCODER_DISABLE` is defined; the second test reads as if `MOTOR_SENSOR_HALL_DISABLE` were meant. `RotorSensor_Of`'s HALL case and `ROTOR_SENSOR_ENABLED.HALL` are unconditional: the table has no build without Hall.
- `Motor_Sensor.h` defines table init macros for EMPTY, HALL, ENCODER and SENSORLESS only; an application's `MOTOR_SENSOR_TABLE_INIT_SIN_COS` has no definition, and `Motor_Sensor.h` does not include `Motor_SinCos.h`. `MOTOR_SENSOR_SIN_COS_ENABLE` is not buildable through the table as written.
- The encoder start-up chain is not in `Motor_Sensor_StartUpState` (its declaration is commented in `Motor_Encoder.h`); the encoder's direction calibration (`StartDirection`, `ProcDirection`) has no state wrapping it; its `VERIFY_CALIBRATION` is a `true` stub.
- `HALL_VTABLE` is `static` in a header, one copy per translation unit that includes it; the other three vtables are `extern const`.
- `Motor_CaptureStator` is compiled only under `MOTOR_SENSOR_SENSORLESS_ENABLE` (`Motor_Thread.h`), although every vtable fills the slot.
- Hall ISR mode (`MOTOR_HALL_MODE_ISR`, `Hall_CaptureAngle_ISR` through `Motor_HallEncoder.h`) and the mechanical-speed Hall units (`Hall_RotorSensor_InitUnits_MechSpeed`) exist and are not selected by any vtable or board.
- Sequence B of the sensorless correction (between the take and the loop) and SENSORLESS_RUN are commented out in `Motor_Sensorless.c`; form A, `CAPTURE_STATOR` after the state machine, is what is built.
- The `TIMER` member of `RotorSensor_T` and the `DirectionErrorCount` reset are commented out.
