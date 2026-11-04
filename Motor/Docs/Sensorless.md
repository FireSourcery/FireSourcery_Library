Vgeometry # Sensorless

The rotor angle from the stator's EMF: the extended-EMF sliding-mode observer (`EmfObserver_T`, `Model/EmfObserver.h`) behind the `RotorSensor_T` interface (`Sensorless_Sensor_T`, `Sensor/Sensorless/Sensorless_Sensor.h`), and its start-up chain (`Sensor/Sensorless/Motor_Sensorless.c`).

| | |
|---|---|
| Selected by | `Config.SensorMode` = `ROTOR_SENSOR_ID_SENSORLESS`, in a build with `MOTOR_SENSOR_SENSORLESS_ENABLE`. Takes effect on reboot: `Motor_Init` sets `p_ActiveSensor` |
| Predict | Every control cycle, at `CAPTURE_ANGLE`: θ̂ += ω̂ (`AnglePll_Predict`), then published to `RotorSensor_State_T.AngleSpeed` |
| Correct | Every control cycle, after the write: `Motor_Sensorless_Thread`, from the application's `BOARD_PWM_ISR`. Corrects ω̂ only (`AnglePll_Correct`); the next predict carries it into θ̂. About 40 µs on KE06, estimated: [Timeline](#timeline) |
| Front end | EEMF SMO in the frame θ̂ (`EmfObserver_Eemf_Proc`): i Parked at θ̂, the held command at θ̂ − Δθ̂/2, as if applied across the whole period the sample closes. It is applied from its load: [Timeline](#timeline) |
| Inputs | The set the LOOP took: I while driven, V while floating (`Motor_ADC_MarkPhases`). The command held after the last write (`EmfObserver_HoldV`) |
| Plant | `Stator_Winding_T` from `Config.Electrical`: Rs, Lq on both axes, at `INIT_UNITS` on `Motor_Reset`. An electrical var reaches the FOC at once, the winding on `Motor_Reinit` |
| Tuning | `EmfObserver_Config_T` from NVM (`P_NVM_CONFIG`). Default `EMF_OBSERVER_CONFIG_DEFAULT(VBus, v_max, I, i_max, Freq)`, Freq the control frequency. Vars are read only |
| Feedback | `IS_FEEDBACK_AVAILABLE` is the lock: ‖ê‖ above `MagFloor` and the error within `LockErrTol` for `LockHoldCount` corrections, held through `LossHoldCount` |
| Calibrated | `K_smo`, `LpfCoef`, `Winding.G` nonzero, else `INIT` → `FAULT` with `PositionSensor` |
| Rates | Predict, correct, lock counts and the PLL PI at the control frequency (`Config.Pll.SampleFreq`); the model's G per control cycle (the turn base); `CAPTURE_SPEED` at `MOTOR_SPEED_LOOP_FREQ` |
| Not wired | `Motor_Sensorless_StartRunChain` (a command entry), the αβ SMO (`EmfObserver_AlphaBeta_Proc`, `_ProcHeld`) and the voltage model (`EmfObserver_VModel_Proc`) |
| Not measured | No hardware run, no plant simulation of this wiring. `K_smo`, `SmoSat` and the PLL gains are not tuned per motor. The correction's duration is an instruction-count estimate |

## Structure

```mermaid
classDiagram
    direction LR
    class Motor_T {
        RotorSensor_Table_T SENSOR_TABLE
        P_MOTOR
    }
    class Motor_Context_T {
        p_ActiveSensor
        RotorSensor_State_T SensorState
        Motor_Config_T Config
        FOC_T Foc
        Phase_Input_T PhaseInput
    }
    class Sensorless_Sensor_T {
        RotorSensor_T BASE
        P_OBSERVER
        P_ELECTRICAL
        P_NVM_CONFIG
    }
    class RotorSensor_T {
        P_VTABLE
        P_STATE
    }
    class SENSORLESS_SENSOR_VTABLE {
        INIT
        INIT_UNITS
        CAPTURE_ANGLE
        CAPTURE_SPEED
        IS_FEEDBACK_AVAILABLE
        ZERO_INITIAL
        VERIFY_CALIBRATION
    }
    class RotorSensor_State_T {
        Angle_T AngleSpeed
        UnitRef
        Speed_Pu
    }
    class EmfObserver_T {
        Stator_Winding_T Winding
        EmfObserver_Axis_T Alpha, Beta
        EmfObserver_Axis_T D, Q
        AnglePll_T Pll
        fract16_t VOutAlpha, VOutBeta
        EmfObserver_Config_T Config
    }
    class EmfObserver_Axis_T {
        Stator_Axis_T Stator
        fract16_t Z
        fract16_t Emf
    }
    class Stator_Axis_T {
        fract16_t I
        fract16_t IPrev
        fract16_t V
    }
    class Stator_Winding_T {
        accum32_t R
        accum32_t L
        accum32_t G
    }
    class AnglePll_T {
        Angle_T Base
        PID_T LoopFilter
        fract16_t PhaseError
        AnglePll_FilterTuning_T FilterTuning
        ufract16_t Magnitude
        uint16_t LockCount
        uint16_t LossCount
    }
    class EmfObserver_Config_T {
        fract16_t K_smo
        fract16_t SmoSat
        fract16_t LpfCoef
        AnglePll_FilterTuning_T PllFilterTuning
        PID_Config_T Pll
    }
    class Motor_Electrical_T {
        Rs
        AngleFreqPerOhmQ
    }

    Motor_T --> Motor_Context_T : P_MOTOR
    Motor_T *-- Sensorless_Sensor_T : SENSOR_TABLE.SENSORLESS
    Sensorless_Sensor_T *-- RotorSensor_T : BASE
    RotorSensor_T --> SENSORLESS_SENSOR_VTABLE : P_VTABLE
    RotorSensor_T --> RotorSensor_State_T : P_STATE
    Motor_Context_T *-- RotorSensor_State_T : SensorState
    Motor_Context_T --> RotorSensor_T : p_ActiveSensor
    Motor_Context_T *-- Motor_Electrical_T : Config.Electrical
    Sensorless_Sensor_T --> EmfObserver_T : P_OBSERVER, static
    Sensorless_Sensor_T --> Motor_Electrical_T : P_ELECTRICAL
    Sensorless_Sensor_T --> EmfObserver_Config_T : P_NVM_CONFIG, in NVM
    EmfObserver_T *-- Stator_Winding_T : Winding
    EmfObserver_T *-- EmfObserver_Axis_T : D, Q wired
    EmfObserver_T *-- AnglePll_T : Pll
    EmfObserver_T *-- EmfObserver_Config_T : Config, the RAM copy
    EmfObserver_Axis_T *-- Stator_Axis_T : Stator, the model
```

The observer is allocated by the table entry (`SENSORLESS_SENSOR_INIT`), so `Motor_Context_T` carries none of it. `RotorSensor_State_T` is shared by every sensor of the motor.

```mermaid
flowchart TB
    subgraph App["Application"]
        PWM["BOARD_PWM_ISR"]
        ADC["ADC completion ISR"]
    end
    subgraph MotorLayer["Motor"]
        Thread["Motor_Thread.h<br/>Motor_PWM_Thread"]
        SM["Motor_StateMachine.c<br/>PASSIVE entry, feedback lost"]
        Chain["Motor_Sensorless.c<br/>start-up chain, Motor_Sensorless_Thread"]
        FOCm["Motor_FOC.c<br/>Motor_FOC_AngleControl"]
        Analog["Analog/Motor_Analog.h<br/>PhaseInput"]
    end
    subgraph SensorLayer["Sensor"]
        RS["RotorSensor.h<br/>the vtable, RotorSensor_State_T"]
        Adapter["Sensorless_Sensor.h, .c<br/>vtable methods, ProcEemf"]
    end
    subgraph ModelLayer["Model"]
        Obs["EmfObserver.h"]
        Stator["Stator.h"]
    end
    subgraph MathLayer["Math"]
        Pll["Math/Angle/AnglePll.h"]
        SMath["foc_sensorless_math.h<br/>stator_math.h, foc_math.h"]
        Base["Angle.h, PID.h"]
    end

    PWM --> Thread
    PWM --> Chain
    ADC --> Analog
    Thread --> RS
    Thread --> SM
    SM --> Chain
    SM --> FOCm
    RS -. P_VTABLE .-> Adapter
    Chain --> Adapter
    Chain --> Analog
    Adapter --> Obs
    Obs --> Stator
    Obs --> Pll
    Obs --> SMath
    Pll --> Base
```

## One control cycle

```mermaid
sequenceDiagram
    autonumber
    participant ADC as ADC ISR
    participant ISR as BOARD_PWM_ISR
    participant Thr as Motor_PWM_Thread
    participant Sen as Sensorless_Sensor
    participant St as Active state
    participant Obs as Motor_Sensorless_Thread

    Note over ADC: the set the LOOP takes, placed by the board
    ADC->>ADC: Motor_Analog_CaptureIabc → PhaseInput.I, or Vabc → PhaseInput.V while floating
    ISR->>Thr: Motor_PWM_Thread, each motor
    Thr->>Sen: RotorSensor_CaptureAngle → CAPTURE_ANGLE
    Sen->>Sen: AnglePll_Predict, θ̂ += ω̂
    Sen-->>Thr: SensorState.AngleSpeed ← θ̂, ω̂
    Thr->>St: LOOP
    St->>St: in RUN, Motor_FOC_AngleControl at θ̂ — SetTheta, take I, PI, InvPark
    St->>St: Motor_FOC_WriteDuty
    Thr->>St: NEXT, _Motor_TransitionFeedbackLost in RUN and INTERVENTION
    St->>Sen: IS_FEEDBACK_AVAILABLE → EmfObserver_IsLocked
    opt the speed timer, 1 ms
        Thr->>Sen: CAPTURE_SPEED → Speed_Pu from ω̂
    end
    Note over ISR: the board starts or arms the next set, board specific
    ISR->>Obs: Motor_Sensorless_Thread, each motor, last
    Obs->>Obs: CaptureStator → Sensorless_Sensor_ProcEemf → AnglePll_Correct, ω̂
    Obs->>Obs: EmfObserver_HoldV, this cycle's command
```

The correction is off the write's deadline: the FOC already Parked the sample at the θ̂ this cycle published, the frame the correction Parks it in.

## The correction

```mermaid
sequenceDiagram
    participant M as Motor_Sensorless_Thread
    participant A as Sensorless_Sensor_ProcEemf
    participant O as EmfObserver_Eemf_Proc
    participant P as AnglePll_T

    M->>M: return unless the adapter is the active sensor
    M->>M: CaptureStator, Phase_ReadVOut
    alt PWM or V0
        M->>M: i = Clarke of PhaseInput.I, v = VOutAlpha, VOutBeta, 0 under V0
    else Z, floating
        M->>M: i = 0, v = Clarke of PhaseInput.V
    end
    M->>A: i_αβ, v_αβ
    A->>A: Park i at θ̂, v at θ̂ − Δθ̂/2
    A->>O: i_d, i_q, v_d, v_q
    O->>O: X = ω̂·L, stator_reactance
    loop the D and Q axes
        O->>O: Stator_Axis_Integrate — î ← î + G·(v − X·J·i − R·î − z), z of the last sample
        O->>O: z = smo_z(K_smo, SmoSat, î − i)
        O->>O: ê = lpf_step(LpfCoef, ê, z)
    end
    O->>P: foc_rotor_of_emf(sign of ω̂, ê) → AnglePll_ProcRelative
    P->>P: error = −y / ‖ê‖, 0 at or below MagFloor
    P->>P: _AnglePll_ProcLock — LockCount, LossCount
    P->>P: AnglePll_Correct — ω̂ = PI(error)
    M->>M: EmfObserver_HoldV — Foc.Valpha, Foc.Vbeta under PWM, else 0
```

| `Phase_ReadVOut` | The analog thread marks | i | v |
|---|---|---|---|
| `PHASE_VOUT_PWM` | I | Clarke of `PhaseInput.I` | the held command, `VOutAlpha`, `VOutBeta` |
| `PHASE_VOUT_0` | I | Clarke of `PhaseInput.I` | 0: the EMF drives i through the shorted winding |
| `PHASE_VOUT_Z` | V | 0 | Clarke of `PhaseInput.V`: the terminals read the EMF |

The mode is the one the state left for the next period. On the cycle it changes, the correction misreads one sample.

## Signal flow

```mermaid
flowchart LR
    Th["θ̂, ω̂<br/>this cycle's prediction"]
    subgraph Inputs["Sensorless_Sensor_ProcEemf, the frame θ̂"]
        I["PhaseInput.I"] --> CI["Clarke"] --> ParkI["Park at θ̂"]
        VH["VOutAlpha, VOutBeta<br/>the held command"] --> ParkV["Park at θ̂ − Δθ̂/2"]
    end
    Th --> ParkI
    Th --> ParkV
    subgraph Smo["EmfObserver_Eemf_Proc, the D and Q axes"]
        Vax["axis voltage<br/>v − X·J·i, X = ω̂·L"] --> Model["model<br/>î ← î + G·(v − R·î − z)"]
        Model --> Z["z = K_smo·sat(î − i, SmoSat)"]
        Z -->|z, the next sample| Model
        Z --> Lpf["LPF → ê"]
    end
    subgraph Tracker["AnglePll_T"]
        Rot["rotor vector of ê<br/>sign of ω̂"] --> Det["detector −y / ‖ê‖<br/>0 at or below MagFloor"]
        Det --> Lock["lock<br/>IS_FEEDBACK_AVAILABLE"]
        Det --> PI["AnglePll_Correct<br/>PI → ω̂"]
        PI --> Pred["AnglePll_Predict<br/>θ̂ += ω̂, every control cycle"]
    end
    ParkI -->|i_d, i_q| Vax
    ParkI -->|i_d, i_q| Z
    ParkV -->|v_d, v_q| Vax
    Lpf --> Rot
    Th -->|ω̂| Vax
    Pred -->|the next cycle's θ̂, ω̂| Out["SensorState.AngleSpeed<br/>the FOC's angle"]
```

The prediction feeds both the FOC and the next correction's Parks: the loop closes through `AnglePll_T.Base`.

## Indices across cycles

```mermaid
flowchart LR
    subgraph Prev["cycle k−1"]
        C1["correct → ω̂(k−1)"]
        H1["hold v(k−1)"]
        W1["write v(k−1)"]
        W1 --> C1 --> H1
    end
    subgraph Now["cycle k"]
        P["predict θ̂(k) = θ̂(k−1) + ω̂(k−1)"]
        S["the set i(k), sampled across the period v(k−1) held"]
        F["LOOP: Park i(k) at θ̂(k), PI, write v(k)"]
        C["correct: i(k) at θ̂(k), v(k−1) at θ̂(k) − Δθ̂/2 → ω̂(k)"]
        H["hold v(k)"]
        P --> F --> C --> H
        S --> F
        S --> C
    end
    subgraph Next["cycle k+1"]
        P2["predict θ̂(k+1) = θ̂(k) + ω̂(k)"]
        C2["correct with v(k)"]
    end
    C1 --> P
    H1 --> C
    C --> P2
    H --> C2
```

## Timeline

What each step of control cycle k reads and writes, on the built current-loop sequence: KE06, the 20 kHz PWM, a 100 µs control cycle, the set on the trigger 16.5 µs before the top. The lanes are the values as they stand: a value lasts from its write to the next. Open circles read, filled ones write. The bottom panel puts the angles in the rotor's frame at 300 Hz electrical, locked: θ̂(j) stands for the rotor at sample j.

![The observer's values on the current-loop timeline](Simulation/Sensorless/SensorlessSequence.png)

[Simulation/Sensorless/sensorless_sequence.py](Simulation/Sensorless/sensorless_sequence.py), on the ISample scheduler and its built sequence, with the prediction in place of the Hall capture and the correction added after the write. Durations are estimates at k = 1, the model's 32 ns per instruction; the board measured its code at about 1.2 times its estimates.

| Step | Reads | Writes | At, µs from the top |
|---|---|---|---|
| The ADC ISR captures | the set, sampled at −12 | `PhaseInput.I` ← i(k) | −1 |
| Predict, `CAPTURE_ANGLE` | θ̂(k−1), ω̂(k−1) | θ̂(k) | 1.2 |
| The LOOP: Park, PI | i(k), θ̂(k) | | 12 |
| Write | | v(k): Vdq(k) turned by θ̂(k) | 28.2 |
| Load, at the next top | v(k) | the duty, for 100 µs | 50 |
| Correct | i(k), θ̂(k), ω̂(k−1), the held v(k−1) | î, z, ê(k), then ω̂(k) | 28.2 to 66.7 |
| Hold | v(k) | `VOutAlpha`, `VOutBeta` | 68.0 |
| Predict, cycle k+1 | θ̂(k), ω̂(k) | θ̂(k+1) | 101.2 |

- **The correction runs past the next top.** About 1,240 instructions, 39.6 µs: the driven path's ~470, six 64-bit multiplies, three divides, two sine and cosine lookups, a square root and the PI. The PWM ISR between control cycles waits behind it, 19.5 µs, then arms the next set 10.4 µs ahead of its trigger, 29.9 without the observer. From k 1.14 the arm misses, and the set waits a control cycle. On the 1 ms speed tick, about 17 µs more ahead of it, it misses at k 1.
- **The held command is not the voltage across the period the sample closes.** The write loads at the next top, 62 µs after the sample, so that period holds v(k−2) for 62 µs and v(k−1) for 38. The correction pairs i(k) with v(k−1) across all of it. Its modeled voltage leads the applied one by the rotation in sample → load, ω·62 µs: 6.7° at 300 Hz. The hold angle θ̂ − Δθ̂/2 is right only for a command applied from the sample on.
- **The FOC's own command** lags the rotor by sample → load plus half the hold, 112 µs, 12.1° at 300 Hz, absorbed by the PI's integrators. The observer models half the hold, 50 µs.
- **Not drawn**: the sense chain's age, by which the current read lags the current, on top of the same offset.

### With the fixes, not built

The same timeline with both fixes, to compare:
1. **The correction after the arm.** At the 20 kHz PWM, the PWM ISR between control cycles runs it, after its marks and its arm. At the 10 kHz PWM, with one PWM ISR per control cycle, the arm already comes before it.
2. **The hold angle by the load.** v(k−1) is Parked at θ̂(k) + (L/P − ½)·Δθ̂, in place of θ̂(k) − Δθ̂/2. L is the time from sample to load, 62 µs; P is the control period, 100 µs.

![The timeline with the fixes](Simulation/Sensorless/SensorlessSequenceFixed.png)

[Simulation/Sensorless/sensorless_sequence_fixed.py](Simulation/Sensorless/sensorless_sequence_fixed.py), on the same model.

| | Built | With the fixes |
|---|---|---|
| The correction | 28.2 to 68.0 µs, in the control cycle's PWM ISR | 53.6 to 100.9 µs, in the between ISR, through the next set's ADC ISR |
| The between ISR | starts 19.5 µs late | on time |
| Its arm, ahead of the trigger, at k 1 / 1.2 | 10.4 µs / misses from k 1.14 | 29.9 / 29.1 µs |
| The same, on the 1 ms speed tick | misses / misses | 29.9 / 10.1 µs |
| The next control cycle starts late, at k 1 / 1.2 | 0 / 4.9 µs | 2.3 / 12.8 µs |
| The same, on the speed tick | 0 / 25.3 µs: loads at the bottom, 25 µs later | 2.3 / 31.9 µs: loads at the bottom, 25 µs later |
| Its write passes its top | at k 1.2 on the speed tick | from k 1.23; at k 1.2 on the speed tick |
| The modeled voltage against the applied one, 300 Hz | leads by 6.7° | 0.0° |

- **A missed arm costs a control cycle; a late write, half a PWM period.** The first fix trades one for the other, and moves the slack from the arm to the next cycle's write. Either way the correction's 40 µs is the cost; reusing the FOC's Park of i(k), which the LOOP already made at θ̂(k), would take off its Clarke, one Park and one sine and cosine.
- **The hold angle is exact for a Vdq steady across v(k−2) and v(k−1).** A Vdq step between them is modeled in v(k−1)'s for the L it isn't applied. Holding both commands and their L-weighted mean is exact for any Vdq, at 2 more fields. L/P belongs to the board's sequence.

## Init and selection

```mermaid
flowchart TB
    Init["Motor_Init"] -->|1| Of["RotorSensor_Of(Config.SensorMode)<br/>→ p_ActiveSensor"]
    Of --> RInit["RotorSensor_Init → INIT<br/>Sensorless_Sensor_Init"]
    RInit --> OInit["EmfObserver_Init(P_NVM_CONFIG)<br/>the Config copy, AnglePll_InitPid, InitFilterTuning, Reset"]
    Init -->|2| Reset["Motor_Reset"]
    Init -->|3, StateMachine_Init, after INIT_WAIT| InitState
    Reinit["Motor_Reinit"] --> Reset
    Reset --> Units["Motor_InitUnits → RotorSensor_InitUnitsFrom<br/>SpeedPuRef, then INIT_UNITS"]
    Units --> Wind["Sensorless_Sensor_InitUnits<br/>Motor_Electrical_IsValid(P_ELECTRICAL)"]
    Wind -->|valid| WInit["EmfObserver_InitWinding(Rs, Motor_Lq_PuTurn)<br/>Stator_Winding_Init → R, L, G"]
    Wind -->|invalid| Keep["the prior winding"]
    Reset --> FOCp["Motor_FOC_InitParams<br/>the FOC's view of the same model"]
    Var["an electrical var"] --> FOCp
    Var -. applies on .-> Reinit
    InitState["INIT, Init_Next"] --> Ver["VERIFY_CALIBRATION<br/>K_smo, LpfCoef, Winding.G nonzero"]
    Ver -->|fails| Fault["FAULT, PositionSensor"]
    Ver -->|passes| Deact["DEACTIVATED"]
```

`RotorSensor_Init` runs before `Motor_Reset`, so the winding is resolved after the config copy and before `INIT` verifies it. `ZERO_INITIAL` (`EmfObserver_Reset`) clears the per-sample state and keeps the config and the winding.

## States

```mermaid
stateDiagram-v2
    PASSIVE --> SENSORLESS_ALIGN: PWM, direction set, no lock, zero speed
    PASSIVE --> RUN: PWM, direction set, locked
    state OPEN_LOOP {
        SENSORLESS_ALIGN --> SENSORLESS_START_UP: AlignTime_Cycles
        note right of SENSORLESS_ALIGN
            ZERO_INITIAL, EmfObserver_Reset
            V0, then the align current at angle 0
        end note
        note right of SENSORLESS_START_UP
            EmfObserver_SeedAngle at the ramp's angle and speed
            the open-loop ramp
        end note
    }
    SENSORLESS_START_UP --> RUN: lock, FOC_MatchTheta
    OPEN_LOOP --> PASSIVE: Z or 0, DIRECTION NULL
    RUN --> PASSIVE: lock lost, within the freewheel limit
    RUN --> FAULT: lock lost above it, PositionSensor
    INTERVENTION --> PASSIVE: lock lost, within the freewheel limit
    INTERVENTION --> FAULT: lock lost above it
    note left of PASSIVE
        The observer runs in every state while selected
        floating, it tracks the terminal EMF
    end note
```

- `Motor_Sensor_StartUpState` returns `SENSORLESS_ALIGN` only while the adapter is the active sensor; `Config.SensorMode` can name it before the reboot that makes it so.
- The hand-off re-Parks the FOC's sample and command at θ̂ (`FOC_MatchTheta`), and `Run_Entry` seeds the loop and the torque ramp from them.
- Full state machine: [Motor_StateMachine.md](Motor_StateMachine.md).
- The math, drawn as geometry: [SensorlessMath.md](SensorlessMath.md).
