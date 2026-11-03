# Phase Current Sampling

Where the phase-current sample sits in the PWM period, how its conversion starts, and where the current loop takes it. The duty convention and counter events: [PwmConvention.md](PwmConvention.md). A low-side sensor's window and the modulation limit it sets: [ModulationLimit.md](ModulationLimit.md).

## Placement in the period

- A low-side sensor reads a phase only in its window, (1 − d)·T centered on the counter top.
- Every window's leading edge comes before the top. Its first part is the switching edge: dead time plus the driver's turn-on delay.
- Sampled at the top, every phase is at its window's center, where the PWM ripple crosses its period average. A phase whose half window, (1 − d)·T/2, is shorter than the sample's distance from the top is outside its window; what it reads then depends on the sense circuit.
- The analog cycle is every `MOTOR_CONTROL_FREQ / MOTOR_I_LOOP_FREQ` periods, a power of 2 (`PHASE_ANALOG_DIVIDER_MASK`, `_Motor_IsAnalogCycle`).

## Start

- **Mark before the write.** In the PWM interrupt, `_Motor_Analog_Thread` marks I (V while floating) before `Motor_PWM_Thread` writes this cycle's duties. The duty registers then hold the duties of the window around this top, which `PHASE_ANALOG_I_SENSOR_LOW_SIDE` selects from.
- **Software start.** In the PWM interrupt, after the marks: it converts in the window around this top.
- **Hardware trigger.** `ADC_ArmMarked`, under `ADC_HW_TRIGGER_ENABLE` on a platform with `HAL_ADC_HW_TRIGGER_FIFO` (KE0x), arms the marked channels of one set on the next trigger, one set per trigger: the board alternates the I set with the other marked channels. The completion continues nothing, so nothing walks by software. A set still armed at the next arm had no trigger: it is disarmed, counted in `MissedTriggers`, and stays marked for its next arm.
- **A trigger after the bottom converts at the next write's duties.** The window around the next top runs at the duties written after the marks, so `PHASE_ANALOG_I_SENSOR_LOW_SIDE`'s selection is one write old there. Near a handoff between the two highest duties, the phase left out may not be the highest. Not evaluated.

## Where the current loop takes the sample

| Option | Effect |
|---|---|
| none | The LOOP takes the set at its own angle: `Motor_FOC_TakeI` Parks and clears the flags |
| `MOTOR_I_FEEDBACK_ON_PHASE` | The completion also runs the current loop, in the states whose action table holds `Motor_FOC_ProcTorqueFeedback`. Needs a board completion that calls `Motor_ADC_OnIabc_Thread` |
| `FOC_I_SAMPLE_DELAY` | Every I Park takes θ − ω·delay, the delay in control periods. Independent of the others |

A state that Parks the set at its completion, at the angle the PWM interrupt that started it set, calls `Motor_FOC_CaptureI` in its ON_PHASE action and takes the set with `Motor_FOC_TakeCapturedI`. Functions, not an option: [ConditionalCompilation.md](../../Docs/Conventions/ConditionalCompilation.md).

## Not established

- Whether the FTM's init trigger fires once per period in up-down counting. `MissedTriggers` answers it on the bench.
