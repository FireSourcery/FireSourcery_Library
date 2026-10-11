# Todo

## Motor

- [ ] Short-circuit current I_sc as the field-weakening bound when V0 is the safe state
    - Default bound, with float as the safe state: line-to-line BEMF ≤ V_RATED, the derated V_DSS. Phase-peak v_emf ≤ V_RATED / √3, 41.6 V on the 72 V board. In speed_pu at k = 2: (2 / √3) · `Phase_VRated_Pu()`. It replaces the `INT16_MAX` ceiling in the field-weakening branch of `Motor_IsConfigValid`; `_Motor_SpeedFwLimit` is the unused stub for it.
    - With V0 (all low-side switches on) as the safe state, no energy reaches the bus, so the bound becomes the short-circuit current: I_sc = ψ / Ld. The speed base cancels in pu: I_sc_pu = ψ_pu / Ld_pu. From the rates in [Motor_Electrical_T]: I_sc [A] = AngleFreqPerOhmD / AngleFreqPerVolt.
    - Check I_sc against `Phase_IRatedPeak_Pu()` and the motor's demagnetization current. Raise the speed ceiling above the BEMF bound only when the check passes.
    - Requires the Fault state to choose V0 by speed and fault type. `Fault_Entry` and `Fault_Proc` always float. A shorted switch or a desaturation trip must still float.
    - Account for the current overshoot at V0 entry above the steady I_sc, the gate-drive supply holding the low side on while the bus collapses, and the fallback to float if V0 fails.
    - Reference, debug motor (Kv 150, 4 pole pairs, Ld 85 µH): ψ = 7.96 mWb, I_sc ≈ 94 A peak, under the 226 A rated peak.
- [ ] Ls as the per-phase inductance: the phase-to-neutral winding self-inductance in the abc frame
    - `Motor_Electrical_LsTick` is the αβ average (Ld + Lq) / 2 for now, read by the observer gain (`FOC_Sensorless_InitG`).
