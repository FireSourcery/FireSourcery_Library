Vgeometry # Sensorless

The rotor angle from the stator's EMF: the extended-EMF sliding-mode observer (`EmfObserver_T`, `Model/EmfObserver.h`) behind the `RotorSensor_T` interface (`Sensorless_Sensor_T`, `Sensor/Sensorless/Sensorless_Sensor.h`), and its start-up chain (`Sensor/Sensorless/Motor_Sensorless.c`).

| | |
|---|---|
| Selected by | `Config.SensorMode` = `ROTOR_SENSOR_ID_SENSORLESS`, in a build with `MOTOR_SENSOR_SENSORLESS_ENABLE`. Takes effect on reboot: `Motor_Init` sets `p_ActiveSensor` |
| Predict | Every control cycle, at `CAPTURE_ANGLE`: θ̂ += ω̂ (`AnglePll_Predict`), then published to `RotorSensor_State_T.AngleSpeed` |
| Correct | Every control cycle a state wrote the FOC's command: the sensor's `CAPTURE_STATOR` (`Sensorless_Sensor_CaptureStator`), after the state machine (`Motor_CaptureStator` in `Motor_PWM_Thread`), every cycle; valid on a cycle the FOC wrote, the caller's constraint. i is the FOC's take, its Id, Iq: in the frame θ̂ where the FOC Parks at θ̂ (RUN, INTERVENTION); in the ramp's in the start-up chain. Corrects ω̂ only (`AnglePll_Correct`); the next predict carries it into θ̂. About 33 µs on KE06, estimated: [Timeline](#timeline) |
| States | `SENSORLESS_ALIGN`, `SENSORLESS_START_UP` under `OPEN_LOOP`, then `RUN`. RUN, INTERVENTION and the chain correct on their write; the other states on what `Foc` holds, outside the caller's constraint. [States](#states) |
| Front end | EEMF SMO in the frame θ̂ (`EmfObserver_Eemf_Proc`): i Parked at θ̂, the held command at θ̂ − Δθ̂/2, as if applied across the whole period the sample closes. It is applied from its load: [Timeline](#timeline) |
| Inputs | The FOC's take of the set the LOOP took, I while driven (`Motor_ADC_MarkPhases`). The command held after the last write (`EmfObserver_HoldV`) |
| Plant | `Stator_Winding_T` from `Config.Electrical`: Rs, Lq on both axes, at `INIT_UNITS` on `Motor_Reset`. An electrical var reaches the FOC at once, the winding on `Motor_Reinit` |
| Tuning | `EmfObserver_Config_T` from NVM (`P_NVM_CONFIG`). Default `EMF_OBSERVER_CONFIG_DEFAULT(VBus, v_max, I, i_max, Freq)`, Freq the control frequency. Vars are read only |
| Feedback | `IS_FEEDBACK_AVAILABLE` is the lock: ‖ê‖ above `MagFloor` and the error within `LockErrTol` for `LockHoldCount` corrections, held through `LossHoldCount` |
| Calibrated | `K_smo`, `LpfCoef`, `Winding.G` nonzero, else `INIT` → `FAULT` with `PositionSensor` |
| Rates | Predict, correct, lock counts and the PLL PI at the control frequency (`Config.Pll.SampleFreq`); the model's G per control cycle (the turn base); `CAPTURE_SPEED` at `MOTOR_SPEED_LOOP_FREQ` |
| Not wired | `Motor_Sensorless_StartRunChain` (a command entry), the correction between the take and the PI (`Motor_Sensorless_ProcTorqueReq`, form B), the αβ SMO (`EmfObserver_AlphaBeta_Proc`, `_ProcHeld`) and the voltage model (`EmfObserver_VModel_Proc`) |
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
        P_FOC
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
        CAPTURE_STATOR
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
        Chain["Motor_Sensorless.c<br/>start-up chain"]
        FOCm["Motor_FOC.c<br/>Motor_FOC_AngleControl, Motor_FOC_WriteDuty marks the tick"]
        Analog["Analog/Motor_Analog.h<br/>PhaseInput"]
    end
    subgraph SensorLayer["Sensor"]
        RS["RotorSensor.h<br/>the vtable, RotorSensor_State_T"]
        Adapter["Sensorless_Sensor.h, .c<br/>vtable methods, CAPTURE_STATOR"]
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
    ADC --> Analog
    Thread --> RS
    Thread --> SM
    SM --> Chain
    SM --> FOCm
    RS -. P_VTABLE .-> Adapter
    Chain --> FOCm
    FOCm --> Analog
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
    participant St as RUN, the active state

    Note over ADC: the set the LOOP takes, placed by the board
    ADC->>ADC: Motor_Analog_CaptureIabc → PhaseInput.I
    ISR->>Thr: Motor_PWM_Thread, each motor
    Thr->>Sen: RotorSensor_CaptureAngle → CAPTURE_ANGLE
    Sen->>Sen: AnglePll_Predict, θ̂ += ω̂
    Sen-->>Thr: SensorState.AngleSpeed ← θ̂, ω̂
    Thr->>St: LOOP
    St->>St: Motor_FOC_AngleControl at θ̂ — SetTheta, take I, PI, InvPark
    St->>St: Motor_FOC_WriteDuty, marks the tick written
    Thr->>St: NEXT, feedback lost
    St->>Sen: IS_FEEDBACK_AVAILABLE → EmfObserver_IsLocked, the last cycle's
    opt Motor_CaptureStator, every cycle
        Thr->>Sen: CAPTURE_STATOR — the take, P_FOC's Id, Iq → AnglePll_Correct, ω̂
        Sen->>Sen: EmfObserver_HoldV, this cycle's command
    end
    opt the speed timer, 1 ms
        Thr->>Sen: CAPTURE_SPEED → Speed_Pu from ω̂
        Thr->>St: ON_SPEED, RUN's action table
    end
    Note over ISR: the board starts or arms the next set, board specific
```

The correction is off the write's deadline: the FOC already Parked the sample at the θ̂ this cycle published, the frame the correction Parks it in. The start-up chain and INTERVENTION run the same order. Hall, Encoder and SinCos define `CAPTURE_STATOR` empty (`RotorSensor_CaptureStatorNone`).

## The correction

```mermaid
sequenceDiagram
    participant S as Motor_CaptureStator
    participant M as Sensorless_Sensor_CaptureStator
    participant A as EmfObserver_Eemf_ProcHeld
    participant O as EmfObserver_Eemf_Proc
    participant P as AnglePll_T

    S->>M: CAPTURE_STATOR, P_FOC the motor's Foc
    M->>A: Foc.Id, Foc.Iq, the take; the held VOutAlpha, VOutBeta
    A->>A: Park v at θ̂ − Δθ̂/2
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
    M->>M: EmfObserver_HoldV — Foc.Valpha, Foc.Vbeta
```

| State | The FOC's angle | i | v |
|---|---|---|---|
| RUN, INTERVENTION | θ̂ | the take, `Foc.Id`, `Iq`: the FOC's Clarke, sine and cosine, and Park | the held command, `VOutAlpha`, `VOutBeta` |
| `SENSORLESS_ALIGN`, `SENSORLESS_START_UP` | the ramp's | the take, in the ramp's frame: θ̂'s while re-seeded, off θ̂ by the load angle once \|ê\| holds above the floor | the held command |
| Any other state that writes through `Motor_FOC_WriteDuty`: open loop, calibration | its own | the take, in its frame | the held command |
| PASSIVE, floating | θ̂, the V capture's | 0: the V capture clears I, none flows | the measured terminal voltage, `FOC_CaptureVBemfData` writes `Valpha`, `Vbeta`: the EMF |
| PASSIVE, shorted (V0) | θ̂, the I capture's | the current the EMF drives, `FOC_CaptureIData` | 0: the I capture clears V, the terminals at 0 |
| DEACTIVATED | none | 0, the feedback clear | 0, the feedback clear |
| FAULT | none | the last, not cleared | the last, not cleared |

Id, Iq are θ̂'s only where the FOC Parked at θ̂, and only without `FOC_I_SAMPLE_DELAY` (with it, at θ̂ − ω·delay). Elsewhere the observer reads them in the FOC's frame. Parking Iα, Iβ at θ̂ (`EmfObserver_ParkI`) is correct in every state, at a sine and cosine and a Park more; it waits for the sample delay's work. A cycle that writes drives PWM, so the set is I and v is the command.

## Signal flow

```mermaid
flowchart LR
    Th["θ̂, ω̂<br/>this cycle's prediction"]
    subgraph Inputs["the FOC's take, and the hold, in the frame θ̂"]
        I["PhaseInput.I"] --> CI["Clarke, the FOC's"] --> ParkI["Park at θ̂<br/>the FOC's in RUN, INTERVENTION"]
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

What each step of control cycle k reads and writes in RUN, on the built current-loop sequence: KE06, the 20 kHz PWM, a 100 µs control cycle, the set on the trigger 16.5 µs before the top. The lanes are the values as they stand: a value lasts from its write to the next. Open circles read, filled ones write. The bottom panel puts the angles in the rotor's frame at 300 Hz electrical, locked: θ̂(j) stands for the rotor at sample j.

![The observer's values on the current-loop timeline](Simulation/Sensorless/SensorlessSequenceA.png)

[Simulation/Sensorless/sensorless_sequence.py](Simulation/Sensorless/sensorless_sequence.py), on the ISample scheduler and its built sequence, with the prediction in place of the Hall capture and the correction after the write. Durations are estimates at k = 1, the model's 32 ns per instruction; the board measured its code at about 1.2 times its estimates.

| Step | Reads | Writes | At, µs from the top |
|---|---|---|---|
| The ADC ISR captures | the set, sampled at −12 | `PhaseInput.I` ← i(k) | −1 |
| Predict, `CAPTURE_ANGLE` | θ̂(k−1), ω̂(k−1) | θ̂(k) | 1.2 |
| The take: Clarke, Park | i(k), θ̂(k) | `Foc.Id`, `Iq` ← i(k) at θ̂(k) | 12 |
| PI, write | the take | v(k): Vdq(k) turned by θ̂(k) | 28.2 |
| Correct | the take, θ̂(k), ω̂(k−1), the held v(k−1) | î, z, ê(k), then ω̂(k) | 28.2 to 61 |
| Load, at the next top | v(k) | the duty, for 100 µs | 50 |
| Hold | v(k) | `VOutAlpha`, `VOutBeta` | 61.2 |
| Predict, cycle k+1 | θ̂(k), ω̂(k) | θ̂(k+1) | 101.2 |

- **The correction runs past the next top.** 32.9 µs: the SMO, the detector and the PLL as counted on the removed thread's 1,239 instructions, and the hold angle's sine and cosine and v's Park, 3.1 µs estimated. The FOC's take supplies the rest: its Clarke, θ̂'s sine and cosine, and i's Park. The PWM ISR between control cycles waits behind it, 12.7 µs, then arms the next set 17.2 µs ahead of its trigger, 29.9 without the observer, 10.4 with the thread. From k 1.24 the arm misses, and the set waits a control cycle. On the 1 ms speed tick, about 17 µs more, it is 0.2 µs ahead at k 1 and misses at k 1.2.
- **The held command is not the voltage across the period the sample closes.** The write loads at the next top, 62 µs after the sample, so that period holds v(k−2) for 62 µs and v(k−1) for 38. The correction pairs i(k) with v(k−1) across all of it. Its modeled voltage leads the applied one by the rotation in sample → load, ω·62 µs: 6.7° at 300 Hz. The hold angle θ̂ − Δθ̂/2 is right only for a command applied from the sample on.
- **The FOC's own command** lags the rotor by sample → load plus half the hold, 112 µs, 12.1° at 300 Hz, absorbed by the PI's integrators. The observer models half the hold, 50 µs.
- **Not drawn**: the sense chain's age, by which the current read lags the current, on top of the same offset.

### Fixes, not built

Two fixes were drawn on the removed thread ([sensorless_sequence_fixed.py](Simulation/Sensorless/sensorless_sequence_fixed.py)):
1. **The correction after the arm.** At the 20 kHz PWM, the PWM ISR between control cycles runs it, after its marks and its arm: on the thread, the arm 29.9 µs ahead at k 1 and 1.2, and the next control cycle's write past its top from k 1.23. In the LOOP, the correction precedes the arm wherever the arm follows the PWM thread; moving it after the arm takes it out of the state again.
2. **The hold angle by the load.** v(k−1) is Parked at θ̂(k) + (L/P − ½)·Δθ̂, in place of θ̂(k) − Δθ̂/2. L is the time from sample to load, 62 µs; P is the control period, 100 µs. It applies to the built form as is, and closes the 6.7°.

- **A missed arm costs a control cycle; a late write, half a PWM period.** The first fix trades one for the other, and moves the slack from the arm to the next cycle's write.
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
    state OPEN_LOOP {
        SENSORLESS_ALIGN --> SENSORLESS_START_UP: AlignTime_Cycles
        note right of SENSORLESS_ALIGN
            ZERO_INITIAL, EmfObserver_Reset
            V0, then the align current at angle 0
            corrected on the FOC's Clarke
        end note
        note right of SENSORLESS_START_UP
            EmfObserver_SeedAngle at the ramp's angle and speed,
            at entry and while |ê| is at or below MagFloor
            the open-loop ramp, corrected on the FOC's Clarke
        end note
    }
    SENSORLESS_START_UP --> RUN: lock, FOC_MatchTheta
    PASSIVE --> RUN: PWM, direction set, locked
    OPEN_LOOP --> PASSIVE: Z or 0, DIRECTION NULL
    RUN --> PASSIVE: lock lost, within the freewheel limit
    RUN --> FAULT: lock lost above it, PositionSensor
    INTERVENTION --> PASSIVE: lock lost, within the freewheel limit
    INTERVENTION --> FAULT: lock lost above it
    note right of RUN
        RUN and INTERVENTION corrected on the FOC's take
    end note
    note left of PASSIVE
        No write: corrected on the last command
        Foc holds, outside the caller's constraint
    end note
```
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
- The hand-off re-Parks the FOC's sample and command at θ̂ (`FOC_MatchTheta`), and `Run_Entry` seeds the loop and the torque ramp from them. The lock it reads is the last cycle's: the correction runs after the state machine.
- **No reliable catch-spin.** In PASSIVE the observer corrects on i = 0 and the last RUN command held in `Foc.Valpha`, `Vbeta`: ê reads that command, the lock drops after `LossHoldCount`, and ω̂, the speed feedback, drifts on it. A PWM request then waits on a speed that is not the rotor's. The acquisition procedure's DETECT phase replaces this.
- `SENSORLESS_RUN`, the alternate RUN for form B, is kept commented out in `Motor_Sensorless.c`.
- Full state machine: [Motor_StateMachine.md](Motor_StateMachine.md).
- The math, drawn as geometry: [SensorlessMath.md](SensorlessMath.md).
- Why the observer has its shape, the float-plant findings, the options not built, the Open list: [Sensorless.exp.md](Sensorless.exp.md).
