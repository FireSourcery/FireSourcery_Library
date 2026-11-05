# Hw Sequenced Scan

**Level** — design.

**Force** — A hardware trigger starts a channel set, and a transfer copies its results without the
CPU. Which set converts may change at run time, and the change must land between one sequence's end
and the next trigger. Only the board knows which slot feeds which consumer. The selection protocol is
the same on every board.

**Shape**

| Part | Holds | Written |
|---|---|---|
| Slot table | every set, each a contiguous slot range | once, at init |
| Transfer | every slot into one buffer, on the last slot's request | once, at init |
| `ADC_Sequence_T` | one per set: its ADC, its channel range, its map, a context | `const` |
| `ADC_SequenceTrigger_T` | the selection: `p_ActiveSequence`, `p_NextSequence` | `ADC_SequenceTrigger_SetNext` from any thread, applied by the completion |
| Completion ISR | clear the transfer's flag, then `ADC_SequenceTrigger_OnCompleteDma_ISR` | every completion |

- A slot that didn't convert keeps its last result, so copying every slot is valid whichever set ran.
  One register selects the set: on S32K1, the PDB channel's pre-trigger enable mask.
- The map reads the buffer by channel name and hands each consumer its whole set in one call.
- A scan with nothing to select is one more record, over every slot. The ISR is the same.
- A scan at its own rate, with no consumer waiting on its completion, gets its own trigger and no
  ISR. Its readers pull from the buffer.
- The records are descriptors [Descriptor.md](Descriptor.md). The completion's work is a generic
  protocol, answered once in the library [InformationExpert.md](../Principles/InformationExpert.md).

```c
/* The board's binding header, included into the one translation unit that holds the instances */
static volatile adc_result_t Results[CHANNEL_COUNT];    /* the transfer's destination */

static void OnSetI(void * p_context) { CaptureIabc(&Motor, Results[CHANNEL_IA], Results[CHANNEL_IB], Results[CHANNEL_IC]); }
static void OnSetV(void * p_context) { CaptureVabc(&Motor, Results[CHANNEL_VA], Results[CHANNEL_VB], Results[CHANNEL_VC]); }

static const ADC_Sequence_T SET_I = ADC_SEQUENCE(&Adcs[1U], ADC_MASK_RANGE(CHANNEL_IA, 4U), OnSetI, NULL);
static const ADC_Sequence_T SET_V = ADC_SEQUENCE(&Adcs[1U], ADC_MASK_RANGE(CHANNEL_VA, 4U), OnSetV, NULL);
static ADC_SequenceTrigger_T Trigger;

void Scan_Init(void)         { InitTransfer(Results); ADC_SequenceTrigger_ActivateDma(&Trigger, &SET_I); } /* before the trigger runs */
void Scan_Select(void)       { ADC_SequenceTrigger_SetNext(&Trigger, IsFloating() ? &SET_V : &SET_I); }   /* any thread */
void SCAN_COMPLETE_ISR(void) { ClearTransferFlag(); ADC_SequenceTrigger_OnCompleteDma_ISR(&Trigger); }
```

**In code**
- `ADC_Sequence_T`, `ADC_SequenceTrigger_T`, `ADC_SequenceTrigger_ActivateDma`,
  `ADC_SequenceTrigger_SetNext` and `ADC_SequenceTrigger_OnCompleteDma_ISR`,
  `Peripheral/ADC/ADC_Sequence.h`. The hardware write is `_ADC_ActivateDma`, `Peripheral/ADC/_ADC.h`.
- `ADC_SequenceTrigger_OnComplete` is the software started form. No instance yet.
- One board instance, in the application repo: a two-ADC S32K1 (PDB, eDMA). The PWM triggered ADC
  alternates a current set and a phase voltage set of 4 slots each, or converts all 8 on every
  trigger. The other ADC free runs on its own PDB.

**Limit**
- A set is a contiguous range: the hardware converts slot ranges, and `ADC_Sequence_ActivateDma`
  asserts it. Order the slot table so each set is one range.
- A request applies at the next completion, and converts on the trigger after it.
- The map runs through a function pointer from the ISR.

**Rejected**

| Alternative | Why not |
|---|---|
| A trigger, a transfer and a buffer per set | The switch is several writes, and each reader must know which buffer holds its channel |
| One completion ISR per mode, under `#if` | The ISRs differ only in which set and which map: that is data |
| Reading the trigger's register back to find the set that completed | The software chose the set, and `p_ActiveSequence` records it |
| The board keeps the next set: an id, a snapshot, a compare, an activate per set | The protocol is the same for every trigger, so the library answers it once |
| One capture call per channel | The consumer needs the set, and the set lands at once |

**Source** — table-driven methods (McConnell, *Code Complete*, 2nd ed., ch. 18) for the set records;
none recorded for the whole. The rules it follows: [HardwareBinding.md](../Conventions/HardwareBinding.md).
How it was reached: [2026-11-04_HwScanBindingSimplification.md](../Findings/2026-11-04_HwScanBindingSimplification.md).
