# Writer Owns

A value crossing threads is held by the module whose thread writes it, with the only mutable path; a module on another thread holds a `const` pointer, `volatile` where the write is asynchronous to its reads.

**Level** — design.

**Force** — A value is written in one context (a transfer, an ISR, a periodic thread) and read in
others. Which side holds it by value and which by pointer follows from that, so the type states the
direction of the crossing and a write from the reading side does not compile. What stays still is
one writer per value.

**Shape**

| Side | Holds | Qualifier | Writes |
|---|---|---|---|
| Owner | the storage, by value or through its descriptor's mutable pointer | `volatile` when a transfer or an ISR writes it | yes, on its thread; the same thread announces the write (the completion, the tick) |
| Reader, by reference | a pointer, in its descriptor or as a parameter | `const volatile`; `const` when it reads only between writes | never |
| Reader, by capture | a copy in its own struct, written by its capture function on the owner's thread | — | its copy only: it owns the copy |

```c
/* Owner: the ADC. The transfer or the ADC ISR fills the buffer; its completion announces it */
typedef const struct ADC_Module { ...; volatile adc_result_t * P_CHANNEL_RESULTS; } ADC_T;

/* Reader by reference */
typedef const struct ADC_Conversion { const volatile adc_result_t * P_RESULT; ADC_T * P_ADC; adc_channel_t CHANNEL; } ADC_Conversion_T;

/* Reader by capture: the motor's handler, on the ADC's thread, converts into storage the motor owns */
static inline void Motor_ADC_CaptureIabc(Motor_Context_T * p_motor, adc_result_t a, adc_result_t b, adc_result_t c)
    { p_motor->PhaseInput.I = Phase_Iabc_PuOfAdcu(&p_motor->Config.IabcZeroRef_Adcu, a, b, c); }
```

- The qualifiers carry the contract: `const` says this side does not write, `volatile` says another
  context does.
- Capture, not reference, when the reader needs the value in its own terms (adcu to pu) or held across
  its own processing. The capture function is the copy's one writer
  ([Naming.md](../Conventions/Naming.md), Capture).
- One writer per word, not per struct. A struct two contexts write splits by write authority:
  `ADC_State_T` keeps `ChannelMarkers` and `CompleteFlags` apart because the clear authority differs
  (`Peripheral/ADC/ADC.h`).
- The pattern places the storage once a writer is chosen. Choosing one writer where two exist today
  is [0010](../Decisions/0010_SharedWordWrites.md) option 2; the crossings, field by field, are in
  [Motor/ThreadOwnership.md](../../Motor/ThreadOwnership.md).

**In code**

- `ADC_T.P_CHANNEL_RESULTS` in `Peripheral/ADC/ADC.h`, written by the transfer or `_ADC_CaptureTo`
  (`_ADC.h`). `ADC_Conversion_T.P_RESULT` in `Peripheral/ADC/ADC_Conversion.h` is the reference:
  "Feature module holds the pointer. ADC owns the state."
- `VBus_T`: `MotorController_T.P_VBUS` is `VBus_T *` (`Motor/MotorController/MotorController.h`; init,
  the monitor poll), `Motor_T.P_VBUS` is `const VBus_T *` (`Motor/Motor/Motor.h`, "Read-only"). The
  sample lands through `VBus_ADC_Capture` (`Motor/Motor/VBus/VBus.h`) on the ADC's thread.
- `SysTime_Millis` (`System/SysTime/SysTime.h`), `volatile`, written by `SysTime_CaptureMillis_ISR`.
  Its readers hold `const volatile uint32_t *`: `Timer_Base_T.P_TIME` (`Framework/Timer/Timer.h`),
  `Protocol_T.P_TIMER`, `CAN_T.P_TIMER`, `UserDIn_T.P_TIMER`.
- Capture: `Motor_ADC_CaptureIabc` (`Motor/Motor/ADC/Motor_ADC.h`), above.

**Exceptions in code**

- `_TimerT_ZeroBase` (`Framework/Timer/_TimerT.h`) casts the `const` off `P_TIME` to zero the base.
  Its wrapper `Timer_ZeroBase` has no caller.
- Two writers: `Motor_Context_T.UserCmd`, both `FaultFlags` words, the `PhaseInput` flags. Located in
  ThreadOwnership.md; the multi-word ones are 0010's rows.

**Rejected**

| Alternative | Why not |
|---|---|
| The reader holds the storage, the writer a pointer into it | The writer's thread writes into another module's struct, and the reader's type no longer says who writes it |
| Mutable pointers on both sides | Any write from either side compiles; the direction lives in comments only |
| A copy per reader by default | A copy is a capture with a writer of its own; it pays only where the reader needs its own terms or a held value |
| A lock around shared storage | The thread model decides locks once: [0010](../Decisions/0010_SharedWordWrites.md), [0011](../Decisions/0011_StateMachineLockNesting.md) |

**Source** — The single-writer principle (Martin Thompson, "Single Writer Principle", *Mechanical
Sympathy*, 2011): one writer per datum, any number of readers. Shared references are read-only, one
mutable reference excludes all others (*The Rust Programming Language*, ch. 4.2, "References and
Borrowing"). The temporal firewall, a unidirectional data-sharing interface that the sender writes and
the receivers read (Kopetz and Nossal, FTDCS 1997).
