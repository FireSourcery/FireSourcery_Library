# ADC Variant Map

The activation styles the ADC module holds, which preset builds each, and the shared state each one touches. The module is header-only (`Peripheral/ADC/*.h`): a style is built when a board's binding header includes and calls it, so "built" is read from the two presets' `Board_ADC.h`, `KMC_Main.c` and `Precompile.h`, and from the symbols in their map files. Derived 2026-11-14.

## Styles

| Style | Entry points | Thread of each | Shared state | Built by | Status |
|---|---|---|---|---|---|
| **Pull**, a feature reads its slot | `ADC_Conversion_Mark` (request), `ADC_Conversion_GetResult` (read) | the consumer's thread: 1 ms, 16 ms, 1 s blocks, the PWM thread | `ChannelMarkers` (OR), `P_CHANNEL_RESULTS` (read) | both presets | live |
| **Hardware-triggered FIFO**, one set per trigger, no walk | `ADC_ArmMarked` / `ADC_Arm` (thread, once per trigger period); `ADC_OnCompleteArmed_ISR` (completion); the board's map on `ADC_PollComplete` / `ADC_ClearComplete` | PWM ISR between control cycles; ADC ISR | `ChannelMarkers` (filter, clear), `ActiveChannels` / `ActiveChannelCount` (the armed state), `CompleteFlags` (set and taken in the ISR), `MissedTriggers` in debug builds; `HAL_ADC_HW_TRIGGER_FIFO` capability | the KE0x preset (`BOARD_ADC_MODE_HW_TRIGGER`) | live |
| **Hardware-sequenced set with transfer**, `ADC_Sequence_T` on a DMA scan | `ADC_SequenceTrigger_ActivateDma` (init), `ADC_SequenceTrigger_SetNext` (any thread), `ADC_SequenceTrigger_OnCompleteDma_ISR` → `_ADC_MarkComplete`, `ADC_Sequence_Poll` (completion), `_ADC_ActivateDma` → `HAL_ADC_SelectDmaSequence` | PWM ISR selects; DMA ISR completes | `CompleteFlags` (set then taken in the ISR), `ADC_SequenceTrigger_T.p_ActiveSequence` / `p_NextSequence`, `P_CHANNEL_RESULTS` (DMA destination); `ChannelMarkers` and `ActiveChannels` unused; requires `ADC_HW_SEQUENCER_ENABLE` and `HAL_ADC_DMA_ENABLE` | the S32 preset (`BOARD_ADC_MODE_DMA`; the controller's `ADC_COUNT` is 0, so `ADC_Init` and `ADC_ActivateMarked` never run) | live |
| **Software walk, per-channel push**, `ADC_Channel_T` table | `ADC_ActivateMarked` (thread), `ADC_OnComplete_ISR` + `ADC_Channel_OnComplete_ISR` → `ADC_Channel_CaptureEach` (completion), `ADC_Channel_Activate` | PWM ISR at the control top (`StartSet`); ADC ISR walks the rest | `ChannelMarkers` (work queue), `ActiveChannels` / `ActiveChannelCount` (the FIFO mirror), `CompleteFlags` (taken whole by the dispatcher), `IncompleteCycles`, `ErrorCount` in debug builds | neither: the KE0x board compiles it only with `BOARD_ADC_MODE_HW_TRIGGER` undefined (`KMC_PWM_SwActivate_ISR`, `Board_ADC0_SwActivate_ISR`); the channel table is absent from the KE06 map | selectable fallback, not built |
| **Software sequence**, `ADC_Sequence_T` walked | `ADC_Sequence_Activate`, `ADC_SequenceTrigger_OnComplete` after `ADC_OnComplete_ISR` | — | `ChannelMarkers`, `ActiveChannels`, `CompleteFlags`, the trigger pair | neither; `.Test/ADC_Sequence_Test.c` only | test only |
| **Batch over N ADCs**, `ADC_Batch_T` joined on one trigger | `ADC_BatchTrigger_ActivateDma`, `_Select`, `ADC_BatchTrigger_OnComplete_ISR`, `ADC_Batch_Poll` | — | `CompleteFlags` of every part's ADC, `ADC_BatchTrigger_T.p_Active` / `p_Next` | neither; `.Test/ADC_Batch_Test.c` only. `_ADC_Batch_TakeEach` calls `ADC_TakeFlags`, which no header defines (the base's take is `ADC_PollComplete` / `ADC_ClearComplete`): the header does not compile in the tree as it stands | dead until renamed |
| **Polled completion** | `ADC_PollCompleteFifo` (reads COCO, runs `ADC_OnComplete_ISR`) | — | as the software walk | no caller in either preset or the tests | dead |
| **Library PWM-thread activation** | `MotorController_PWM_Thread`: `Motor_ADC_MarkPhases` on the controller's `ControlCounter` parity, `ADC_ActivateMarked` per ADC, `Motor_PWM_Thread` | — | `ChannelMarkers` | no caller: both presets run `KMC_PWM_*_ISR` in `KMC_Main.c` instead | dead |

## State by style

| Word | Pull | Hw-triggered FIFO | Hw-sequenced DMA | Software walk | Software sequence | Batch |
|---|---|---|---|---|---|---|
| `ChannelMarkers` | OR | filter at the arm, clear at the capture | — | queue: OR, clear as each lands | queue | — |
| `CompleteFlags` | — | set and taken in the ADC ISR | set (`_ADC_MarkComplete`) and taken in the DMA ISR | set in the ISR, taken whole by the channel dispatcher | set in the ISR, taken per set | set per part's ISR, taken by the last part's join |
| `ActiveChannels`, `ActiveChannelCount` | — | the armed FIFO, in channel order; count 0 after capture or disarm | — | the active FIFO, refilled by the walk | as the walk | — |
| `P_CHANNEL_RESULTS` | read | written by the capture loop from `R` | written by the transfer | written by the capture loop | as the walk | by the transfers |
| `ActiveSequence` / `Next`, `p_Active` / `p_Next` | — | — | the trigger's selection, one store, applied at the completion | the trigger's selection | the trigger's selection | the trigger's selection |
| debug counters (`#ifndef NDEBUG`) | — | `MissedTriggers`, `FifoMismatch` | — | `IncompleteCycles`, `ErrorCount`, `FifoMismatch` | as the walk | — |

`CompleteFlags` is the one word every live style maintains the same way (set by the completion, consumed by whoever takes), which is what lets a board map read completion identically under either hardware style. Its clear is a read-modify-write against the ISR's OR, so the take and the set must share a context: in both live presets they do, inside the completion ISR.

## Per preset

| | KE0x preset | S32 preset |
|---|---|---|
| Style | Hw-triggered FIFO, 1 ADC, 8-deep FIFO | Hw-sequenced DMA, 2 ADCs: ADC1 on the PWM trigger through the PDB, ADC0 free-running on its own PDB with no ISR |
| Init | `MotorController_Init` → `ADC_Init` (`HAL_ADC_Init`, deactivate); `Board_Init` routes the trigger and the delay | `Board_ADC_DMA_Init`: the transfers, the first sequence (`ADC_SequenceTrigger_ActivateDma`); `ADC_COUNT` 0 in the controller |
| Request | `ADC_Conversion_Mark` by every consumer; the PWM ISR between control cycles arms the marked channels (`Board_ADC_ArmMarked`, `ADC_MASK_ALL`) | `Board_ADC_ActivateDmaGroup` selects I or V by the phase state under `BOARD_ADC_SCAN_INJECTED`; otherwise the whole 8-slot scan every period; ADC0 readers pull |
| Completion | `Board_ADC0_HwTrigger_ISR`: capture, then the board map by set (`ADC_PollComplete`) | `BOARD_ADC1_DMA_ISR`: `ADC_SequenceTrigger_OnCompleteDma_ISR` → the sequence's `COMPLETE` map (`Board_OnGroupAll`, `_I`, `_V`) |
| Completion priority | above the PWM ISR | above the PWM ISR |
| Includes | `ADC_Thread.h`, `ADC_Channel.h` (table dropped), `ADC_Conversion.h` | `ADC_Sequence.h`, `ADC_Conversion.h` |

## Not shown / open

- The ADC_Batch decision (join N ADCs on one trigger, or one `ADC_Sequence_T` per ADC): this map records that the header is unreferenced and does not compile; the choice is the owner's.
- `Notes.md`, `_.md` and `ADC_Batch_JoinVariants.md` in this directory still name `ADC_TriggerState_T`, `CompleteMarkers`, `ADC_TakeFlags` and `SEQUENCE_ID`: stale against `ADC.h`.
- The 2-of-3 low-side set (`PHASE_ADC_I_SENSOR_LOW_SIDE`) needs marked, per-channel conversion; the hardware-sequenced style cannot leave a slot out, and the KE0x board map takes the set whole.
- The S32 ADC0 scan's restart watchdog (`Board_ADC_Poll`, 1 ms) is the board's, outside this module.
- Priorities and the preemption of a take by a completion: the boards' timelines.
