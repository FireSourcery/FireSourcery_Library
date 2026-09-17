# Encoder Reference Acquisition

Flow reference for `Encoder.c` / `Encoder_ISR.h` and the state machine in
`Motor/Motor/Sensor/Encoder/Motor_Encoder.c`.

An incremental encoder measures change, never position. Every absolute value in the
drive comes from an event that *asserts* a known angle into `AngleCounter.Base.Angle`.
There are two such events, and one acquisition procedure built from them.

---

## 1. Two datums, one dependency

| | Align | Index (Z) |
|---|---|---|
| Asserts | `theta := 0` | `theta := Config.IndexAngleOffset` |
| Mechanism | force `Id` at `theta_e = 0`, rotor d-axis settles | once-per-mechanical-rev marker on `PIN_Z` |
| Valid | mod one **electrical** revolution | mod one **mechanical** revolution |
| Accuracy | approximate — settle position carries cogging, friction and load error | exact — a calibrated constant |
| Unlocks | commutation only | commutation **and** absolute position |
| Costs | moves the rotor, needs current, unusable under load or free-spin | needs motion, which needs commutation you do not yet have |

```mermaid
flowchart LR
    AB["A/B edges"] --> ACC["Angle accumulator<br/>relative, exact in delta"]
    ACC --> SPD["Speed / FreqD<br/>valid with no datum"]

    AL["Align event<br/>Id at theta_e = 0"] -- "theta := 0" --> ACC
    ZI["Index event<br/>Z rising edge"] -- "theta := IndexAngleOffset" --> ACC

    AL --> EF["Electrical frame<br/>mod 360 deg elec"]
    ZI --> MF["Mechanical frame<br/>mod 360 deg mech"]
    EF --> COM["Commutation"]
    MF --> COM
    MF --> POS["Absolute position"]

    EF -. "enables the motion<br/>needed to reach Z" .-> ZI
```

The dotted edge is why align always precedes the sweep: **index is strictly stronger,
but align is the only datum obtainable from standstill.**

Align is also only *approximate*. That is the second reason to keep sweeping to Z rather
than settling for the align frame — the snap replaces a settle-error-bearing angle with a
calibrated one, and commutation accuracy improves for the rest of the power cycle.

---

## 2. Three reference frames

```
d-axis  : zero at the rotor pole.  Set by Align.                        Read by commutation, 20 kHz.
Index   : zero at the Z marker.    Config.IndexAngleOffset from d-axis. The bridge.
User    : zero at virtual home.    Config.VirtualHomeOffset from Z.     Read by position, per packet.
```

`Base.Angle` stays in the **d-axis frame**. The application zero is a derived constant
applied on read, so the hot path pays nothing and `VirtualHomeOffset` can change at
runtime with no side effects:

```c
UserZeroAngle = Config.IndexAngleOffset + Config.VirtualHomeOffset;   /* on either offset write */

static inline uint16_t Encoder_GetAngle_User(const Encoder_State_T * p_encoder)
{
    return (p_encoder->AngleCounter.Base.Angle - p_encoder->UserZeroAngle) >> ENCODER_ANGLE_SHIFT;
}
```

Two constants, and neither can do the other's job:

- `IndexAngleOffset` — **measured, never chosen.** Fixed by how the encoder was bolted to
  the motor. Conventionally the *commutation offset*.
- `VirtualHomeOffset` — **chosen, never measured.** Arbitrary, user-settable over comms.
  CiA 402 *home offset* (0x607C).

---

## 3. Reference state lattice

Monotonic. Promotion is unconditional; only fault demotes. Written by the polling thread
only — never from an ISR, so a fault demote cannot be lost to an index edge in flight.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> NONE

    NONE: REF_STATE_NONE
    NONE: speed valid, angle arbitrary
    ALIGNED: REF_STATE_ALIGNED
    ALIGNED: theta_e valid, approximate
    HOMED: REF_STATE_HOMED
    HOMED: theta_e and theta_m valid, exact

    NONE --> ALIGNED: align settle, theta := 0
    NONE --> HOMED: index edge, theta := Z
    ALIGNED --> HOMED: index edge, theta := Z
    HOMED --> HOMED: index edge, re-snap, err = drift
    ALIGNED --> ALIGNED: re-align, theta := 0
    HOMED --> NONE: fault / sensor loss
    ALIGNED --> NONE: fault / sensor loss
```

`HOMED -> ALIGNED` does not exist. `Encoder_CaptureAlignZero` refuses to run once homed.

---

## 4. One acquisition procedure, three terminal actions

All three use cases run the **same** acquisition: align, then open-loop sweep until Z.
They differ only in what happens after the index edge.

```mermaid
flowchart TB
    G{"RefState on entry"}
    G -- "NONE" --> A["ALIGN<br/>energize d-axis, wait settle<br/>theta := 0, ALIGNED"]
    G -- "ALIGNED" --> S
    G -- "HOMED" --> T{"terminal action"}
    A --> S["OPEN LOOP SWEEP<br/>exit on index edge<br/>travel budget 1.5 mech rev"]
    S -- "budget spent" --> F["FAULT PositionSensor"]
    S -- "index edge<br/>theta := IndexAngleOffset, HOMED" --> T

    T -- "calibrate" --> TC["Commit IndexAngleOffset<br/>phases off, stop"]
    T -- "run" --> TR["Validate, closed loop<br/>continue commanded motion"]
    T -- "home" --> TH["Closed loop<br/>position move to UserZeroAngle"]
```

> **Calibration is the exception to the entry gate.** It always runs a fresh align and a
> fresh sweep even when already `HOMED`, because it is *measuring* the offset rather than
> consuming it. Skipping the align would leave the captured angle in no frame at all.

| Terminal action | Align | Sweep exit | Validate | Ends | RefState after |
|---|---|---|---|---|---|
| **Calibrate** | always, never skipped | index edge | none today | commit offset, phases off | `HOMED` |
| **Run** (first time, position unknown) | skip if `ALIGNED` or `HOMED` | index edge | tracking, then polarity | closed loop, `RUN` | `HOMED` |
| **Home** (user command) | skip if `ALIGNED` or `HOMED` | index edge, skipped entirely if `HOMED` | inherits the sweep's | closed loop, then position move | `HOMED` |

---

## 5. Case A — Index calibration

Commissioning. Measures `IndexAngleOffset` and stops; the motor does not continue into a run.

```mermaid
stateDiagram-v2
    direction LR
    state "STATE_ENCODER_HOMING_ALIGN" as A
    state "STATE_ENCODER_HOMING" as S
    state "MOTOR_STATE_CALIBRATION" as C
    state "MOTOR_STATE_FAULT" as F

    [*] --> A
    A: energize d-axis, wait settle
    A: CaptureAlignZero, theta := 0, ALIGNED
    S: sweep, exit on index edge only
    S: ProcHoming spends travel budget
    C: CalibrateIndexAngleOffset
    C: IndexAngleOffset := Capture
    C: Base.Angle += error, IsIndexCalibrated := true

    A --> S: settled
    S --> C: index edge, HOMED
    S --> F: budget spent
    C --> [*]: phases off, stop
```

The commit is an assignment, not an accumulation — the measurement *is* the offset, and the
prior value does not participate. A factory zero needs no special case.

```c
int32_t error = Encoder_GetIndexAngleError(p_encoder);   /* Capture - Offset, bind before the commit */

p_encoder->Config.IndexAngleOffset = (uint32_t)p_encoder->IndexAngleCapture;
p_encoder->AngleCounter.Base.Angle += error;
```

`error` exists only for the second line. The ISR snapped `Base.Angle` to the *old* offset at
the index edge, so the live angle trails the committed frame until the next Z; carrying the
error over fixes that. Bind it before the commit — the commit is what makes it zero.
`Capture` itself needs no adjustment: the error re-reads as 0 automatically.

---

## 6. Case B — Normal run, first time, position unknown

The sweep **is** the start-up ramp, not a blocking search: the machine is moving and
producing torque throughout. Staying open loop until Z means closed loop begins with an
exact commutation angle instead of the align settle estimate.

```mermaid
stateDiagram-v2
    direction LR
    state "ALIGN" as A
    state "OPEN LOOP SWEEP" as S
    state "VALIDATE TRACKING" as V1
    state "VALIDATE POLARITY" as V2
    state "MOTOR_STATE_RUN" as R
    state "MOTOR_STATE_FAULT" as F

    [*] --> A: skip if already ALIGNED or HOMED
    S: exit on index edge
    S: travel budget 1.5 mech rev
    V1: angle must have advanced
    V2: closed loop handoff, speed vs Vq sign

    A --> S: settled
    S --> V1: index edge, HOMED
    S --> F: budget spent
    V1 --> V2: angle advanced
    V1 --> F: angle static
    V2 --> R: polarity agrees
    V2 --> F: polarity opposed
    R --> R: later index edges re-snap, err = drift
```

Once `HOMED`, subsequent starts in the same power cycle skip the whole chain — align and
sweep both. This runs **once per power-on**, not on every start.

### If the encoder has no Z channel

Then `ALIGNED` is the ceiling and the sweep must exit on a **timer** instead, accepting
the align settle error as the permanent commutation reference. This is the one
configuration where the sweep does not end at an index edge.

---

## 7. Case C — User command: go to virtual home

Same acquisition as Case B; different terminal action. No offset commit — it consumes
`IndexAngleOffset`, it does not measure it.

```mermaid
stateDiagram-v2
    direction LR
    state "RefState?" as G
    state "ALIGN" as A
    state "OPEN LOOP SWEEP" as S
    state "CLOSED LOOP HANDOFF" as H
    state "POSITION MOVE" as M
    state "MOTOR_STATE_FAULT" as F

    [*] --> G
    G --> A: NONE
    G --> S: ALIGNED
    G --> M: HOMED, nothing to acquire
    A --> S: settled
    S: exit on index edge
    S --> H: index edge, HOMED
    S --> F: budget spent
    H --> M
    M: target UserZeroAngle
    M: needs FeedbackMode.Position
    M --> [*]: in position
```

The position move waits on closed loop position control. `FeedbackMode.Position` exists
as a bit, but `PidPosition` is commented out and there is no
`MOTOR_FEEDBACK_MODE_POSITION` preset.

---

## 8. Index ISR and the polling thread

The ISR does two stores, no branch, no mode dispatch. It captures raw and interprets
nothing; every interpretation happens in the polling thread.

```c
/* polling thread, Encoder_StartHoming - arms the capture */
p_encoder->IndexAngleCapture = ENCODER_INDEX_NOT_CAPTURED;

/* ISR - two stores */
static inline void Encoder_CaptureIndex(Encoder_State_T * p_encoder)
{
    p_encoder->IndexAngleCapture       = p_encoder->AngleCounter.Base.Angle;
    p_encoder->AngleCounter.Base.Angle = p_encoder->Config.IndexAngleOffset;
}
```

**Why there is no third store.** The ISR writes two fields from one input; together they
reconstruct exactly one fact and nothing more. A latch is a *third* bit of information, so
the choice is only ever to spend a third store or to reserve one value out of the two
already written. The angle domain has no spare encoding space to reserve accidentally —
`angle32_per_count(cpr) = UINT32_MAX / cpr + 1` rounds up, so low bits are not
structurally zero, and every `int32_t` pattern is a legal angle. Hence the armed sentinel.

```mermaid
sequenceDiagram
    autonumber
    participant ISR as Index ISR
    participant SM as Motor state machine
    participant ST as Encoder state
    participant NV as Config (NvM)

    Note over ISR,NV: calibration pass
    SM->>ST: align settle, theta := 0, ALIGNED
    SM->>ST: StartHoming - arm Capture := NOT_CAPTURED
    SM->>SM: open loop sweep toward Z
    ISR->>ST: Capture := theta
    ISR->>ST: theta := IndexAngleOffset_old
    SM->>ST: PollIndexCapture sees armed value, RefState := HOMED
    SM->>NV: IndexAngleOffset += (Capture - IndexAngleOffset)
    Note over NV: offset now equals theta_measured



    Note over ISR,ST: every run thereafter
    SM->>ST: align settle, theta := 0, ALIGNED
    ISR->>ST: Capture := theta
    ISR->>ST: theta := IndexAngleOffset
    SM->>ST: RefState := HOMED, drift = Capture - offset (expect 0)
```

`Encoder_PollIndexCapture` runs from `Encoder_RotorSensor_CaptureSpeed` on **every** path,
not only while searching — so an index edge reached outside a deliberate sweep promotes
the same way. Only the travel budget belongs to the search.


