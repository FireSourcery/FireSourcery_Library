# Conditional Compilation

Preprocessor paths are used sparingly. Each one is another build the reader holds in mind, and code
the default build never compiles.

- **Variants are functions.** Each implementation is a named function, compiled in every build and
  callable directly. `FOC_ProcIFeedback_Base` and `FOC_ProcIFeedback_Decouple`, and
  `FOC_MatchIVState_Base` and `FOC_MatchIVState_Decouple`, in `Motor/Motor/Math/FOC.h`.
- **One selection point, where the configuration picks.** `FOC_ProcIFeedback` and `FOC_MatchIVState`
  each hold an `#if defined(FOC_DECOUPLE_ENABLE)` that only dispatches to the variants. The `#if`
  earns its place there because it:
  - adds no code: the build carries one variant, not both and a runtime branch;
  - prevents a cross product: the callers call `FOC_ProcIFeedback`, so no caller is written once per
    variant;
  - is a configuration case: a build has the decoupling or not.
- **Guard the reference, not the definition.** An optional feature's functions compile in every
  build; the `#if` goes around the call or table row that pulls them in, and the linker drops them
  where it is off: [GuardedCall.md](../Patterns/GuardedCall.md).
- **A choice the caller can make is a function, not a macro.** Where the variant follows the state or
  the caller, the caller calls the function it needs. `Motor_FOC_CaptureI` and
  `Motor_FOC_TakeCapturedI`, the Park at the I set's completion, replaced `MOTOR_I_CAPTURE_ON_PHASE`
  (2026-11-01). The state that uses the sample, e.g. a sensorless state's ON_PHASE action, decides.
- **A timing gap is tuning, not an option.** A sequence that misses a load point by a few µs is closed
  with what exists: a trigger delay, or less code before the write. `MOTOR_I_FEEDBACK_FIRST`, which
  reordered four LOOPs to make the load point with 1.8 µs of estimated slack, was removed
  (2026-11-01).

Why: a macro path doubles the code to read and to verify, and a path the default build doesn't compile
goes stale unseen. A selection point that dispatches to named variants keeps each variant readable on
its own, and the `#if` in one place.
