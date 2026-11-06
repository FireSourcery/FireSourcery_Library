# 0003 Phase output ownership

**Status** — accepted 2026-10-13, "for now".

**Context** — `Motor_PWM_Thread` wrote duty (SVPWM of `Foc.Va/Vb/Vc`) whenever the gates were on. That
overwrote V0 with T0, so `Fault_Entry` above rated speed re-applied the last Run vector. Deciding in
the thread by reading the duty back is racy: FTM CnV writes are buffered until the next loading
point, so a V0 written in the same ISR reads back as PWM.

**Decision**

- Every state LOOP that produces FOC output calls `Motor_FOC_WriteDuty(p_motor)` right after its
  producer: Run, Intervention TorqueZero and RampDown, OpenLoop AngleAlign, Run and StartUp, Hall
  calibration, Sensorless Align and StartUp, the producing cases of electrical calibration, and the
  Encoder procs.
- `Motor_PWM_Thread` writes no duty.
- Hold states (Fault, Passive, Calibration idle, electrical DONE) write nothing, so
  `Phase_ActivateV0` holds.
- `Motor_FOC_WriteDuty` stays out of line, for flash.

**Consequences**

- A new driving state must call `Motor_FOC_WriteDuty` in its LOOP; a hold state must not.
  `Fault_Entry`'s `FaultFlags.RequestV0` path (OV) relies on this.
- V0, the static low-side short, is the safety short. T0 (50 %) is not equivalent: dead-time bus
  pumping and switching loss.
- Open: resuming from Z at speed enables the gates before the new duty loads (stale CnV, then T0).
  Inferred from the sync configuration, not measured.
- Deferred: a duty-domain output image (`Phase_Data_T`, Flags = gates, Values = duty) written by the
  states and transferred by the thread as the only register writer, then handlers on
  `Motor_Context_T`. The study ("Motor Context Image Study", 2026-10-17,
  https://claude.ai/code/artifact/030a8734-22e8-4bd0-8066-17c49597c701) recommends the image and the
  thread transfer first, and buffered inputs as a separate decision. Its open question: is a
  one-period (≤ 50 µs) gate-off delay acceptable for main-thread software faults? Hardware trips
  bypass this path.
- Started by the user: `Phase_Svpwm_Data` returns a `Phase_Data_T` and `Phase_WriteData` writes it.
  `Phase_Svpwm_Data` must be `static inline` (as a plain header definition the KE06 link fails with
  multiple definitions). Open: whether Flags means gates (the output side) or a consumed fresh-sample
  mark (as on the input side, cleared by `FOC_CaptureIData`).
- Naming (VZ / V0 / T0, Config → Nvm, Runtime) is parked.
