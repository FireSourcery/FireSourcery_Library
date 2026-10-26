# Phase Current Sense: Quick Reference



## The window sets the modulation limit

A phase's current is read only in its low-side window, (1 − d)·T. The shortest window that works today is 3.35 µs, a duty of 93.3 % at 20 kHz. Nothing else in the output path limits duty, so the current loop's circle radius is the modulation limit.

Under midclamp SVPWM at radius r, the highest phase duty reaches ½ + (√3/2)·r and the middle one ½ + (3/4)·r.

| Placement, radius | Shortest window, 3 phases read | Shortest window, 2 of 3 read |
|---|---|---|
| midclamp, Vbus/2 (the default) | 3.35 µs | 6.25 µs |
| midclamp, 0.539·Vbus | 1.67 µs | 4.80 µs |
| midclamp, Vbus/√3 | 0 µs | 3.35 µs |
| ceiling √3/2, Vbus/2 | 6.70 µs | 6.70 µs |
| ceiling √3/2, 0.539·Vbus | 3.35 µs, for 0.3° of the turn | 6.70 µs |
| ceiling √3/2, Vbus/√3 | 0 µs | 6.70 µs |

- **Reading 3 phases, current mode is erratic on hardware at Vbus/√3.** The top phase's window stays at or under 3.35 µs for the 120° it is on top and drops under 1 µs for ±16° around each line-to-line peak, so its hold goes stale while its current moves.
- Voltage mode (`Motor_FOC_ProcAngleV`) runs at Vbus/√3. It doesn't feed the current back; the reading reaches only the overcurrent check.

![Low-side window per radius](../../FireSourcery_Library/.Test/LowSideWindow.png)

## The largest radius for today's window

| Read | Placement | Largest radius |
|---|---|---|
| 3 phases | midclamp | Vbus/2 |
| 3 phases | ceiling √3/2 | 0.539·Vbus |
| 2 of 3 | midclamp or ceiling | Vbus/√3, the linear limit |

**Don't raise `FOC_V_CIRCLE_PER_V_BUS` above Vbus/2 without the option the row needs.**

## Firmware options

Set per board in `Source/HAL/Board/<board>/Precompile.h`. The three switches are off by default, and the radius defaults to Vbus/2.

| Option | What it does | Boards |
|---|---|---|
| `FOC_V_CIRCLE_PER_V_BUS` | The current loop's circle radius per Vbus. Default `FRACT16_1_DIV_2`. `FOC_ProcIFeedback_Base`, `_Decouple`, `_BackLimit` and the field-weakening target all use it | both |
| `PHASE_ANALOG_I_SENSOR_LOW_SIDE` | Reads 2 of 3. Converts every phase but the highest duty, taken from the duty registers at the start. `_Phase_Analog_SolveI` completes the third by i_a + i_b + i_c = 0, so every consumer still gets a full set | KE only; S32 converts by DMA batch |
| `PHASE_SVPWM_MIDCLAMP_CEILING` | Midclamp, with the zero sequence lowered until the highest duty is at or under `PHASE_SVPWM_DUTY_CEILING` (default √3/2), never past the bottom clamp | both |
| `PHASE_SVPWM_CLAMP` | The general clamp at a fixed `PHASE_SVPWM_CLAMP_K`, the share of the null time at all-high: 0, the default, is the bottom clamp; ½ is midclamp | both |

## Zero-sequence placement

- The zero sequence moves every phase's duty together. The line-to-line voltages, all the motor sees, are unchanged.
- **Lowering it adds the same low-side time to every phase.** Only the lowest phase's duty is there to give. At Vbus/√3 that is 0 at each line-to-line peak, so reading 3 phases, no placement reaches Vbus/√3.
- General clamp: `vZero = k·(vMax − 1) + (1 − k)·vMin`, with duty = v − vZero. Its shortest window is (1 − k)·(1 − spread)·T, where spread = vMax − vMin.
- Largest radius for a window w, as a fraction of T: r = (1 − w/(1 − k))/√3. Midclamp gives (1 − 2w)/√3 and the bottom clamp (1 − w)/√3. The √3/2 ceiling gives the bottom clamp's for windows up to 6.7 µs at 20 kHz, and midclamp's past that.
- PWM ripple current, RMS relative to midclamp:

  | Radius | ceiling √3/2 | bottom clamp |
  |---|---|---|
  | below 0.42·Vbus | ×1.00 | ×1.90 at 0.2·Vbus |
  | Vbus/2 | ×1.08 | ×1.22 |
  | Vbus/√3 | ×1.03 | ×1.03 |

- The bottom clamp writes 0/0/0 for a zero command, which `Phase_ReadVOut` reports as V0. The ceiling never shifts a zero command.
- The radius for a window, in `Motor/Motor/Math/svpwm_math.h`:
  - `SVPWM_MIDCLAMP_RADIUS_OF_WINDOW(Window, Period)` and `SVPWM_MIDCLAMP_CEILING_RADIUS_OF_WINDOW(Window, Period, Ceiling)`, with integer arguments in one unit;
  - `SVPWM_CLAMP_RADIUS_OF_WINDOW(Window, Period, K)`, in floating point, for constants only;
  - `svpwm_midclamp_radius` and `svpwm_midclamp_ceiling_radius`, their typed forms.
- `SVPWM_MIDCLAMP_CEILING_RADIUS_OF_WINDOW(3350U, 50000U, FRACT16_SQRT3_DIV_2)` is 17651, 0.539·Vbus. `FOC.h` can't see `PHASE_SVPWM_DUTY_CEILING`, so keep the ceiling argument equal to it.

Phase A's duty, ripple, the shortest window against the radius, and the modelled tracking error for each placement:

![Zero-sequence placement](../../FireSourcery_Library/.Test/LowSideZeroSequence.png)

## Capture path

- **KE.** One ADC with an 8-deep FIFO, converting in channel order. The ADC ISR and the PWM ISR are both priority 2, so a capture never lands inside the PWM thread. `ADC_Channel_OnComplete_ISR` pushes each channel to its handler. The `BOARD_ADC_BOARD_HANDLED` path, which is off, calls `Motor_Analog_CaptureIBatch`, which isn't defined.
- **S32.** DMA. The DMA0 ISR, priority 1, preempts the PWM ISR, priority 2, and captures IA, IB and IC as a batch through `Motor_Analog_CaptureIabc`. `BOARD_ADC_SCAN_INJECTED` takes the per-channel path.

## Not measured

- The hold's time constant τ while tracking.
- Whether the sample switch closes on the switching edge.
- None of the options has run on hardware.

