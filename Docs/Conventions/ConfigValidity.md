# Config Validity

Validity is a config invariant, checked once at the config layer. Derivations assume a valid config.

- **Conversions and derives carry no guards.** They are textbook formulas (`motor_electrical_math.h`:
  "no guards"). No `rate > 0` or `l_pu != 0` checks inside them; `_Motor_Electrical_LPuOf` and
  `_Motor_Electrical_RateOfLPu` were dropped for that.
- **Each sub-struct has its own `X_IsValid`**, composed by the outer `Config_IsValid`:
  `Motor_Electrical_IsValid` inside `Motor_Config_IsValid`, which requires `AngleFreqPerVolt` and
  `AngleFreqPerOhmD/Q` > 0.
- **`*_Config_IsValid` faults, `*_Config_Validate` clamps or repairs.** A new invariant goes in one
  of the two, not in the conversion.
- **The resolve step skips an invalid model** and keeps the prior view: `Motor_ResolveFocParams`
  covers var writes, reinit and calibration commits in one place.
- **Wire input is guarded at the var boundary**, `_Motor_Electrical_RateOfSi`.

What to check instead of adding a guard: a derive path that runs before or without an `IsValid`
gate. Boot `Motor_Reset` runs before the Init-state check, and var writes and calibration commits
re-derive at once. On Cortex-M0+ a 64-bit divide by zero returns all ones; it does not trap.
