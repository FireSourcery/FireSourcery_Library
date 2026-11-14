# Limit Physics

Which bound binds where, and why each frame needs its key. The frames and keys themselves are decided
in [../../Decisions/0005_ILimitFrames.md](../../Decisions/0005_ILimitFrames.md); the V side is in
`Motor/Docs/VLimitPolicy.md`. Worked numbers use a 72 V / 50 A pack, a 90 mΩ winding and a 200 A
phase rating.

## Supply (DC) limit → i_q

Power balance: P = 3/2 · (v_d · i_d + v_q · i_q) = V_dc · I_dc.

- **POWER_BALANCE**, exact: P_q = 2/3 · V_dc · [−I_chg : I_dis] − v_d · i_d, and the i_q window is
  hull(P_q, 0) / v_q.
  - The zero hull is load-bearing. When the d axis alone spends a side's budget, P_q excludes 0; i_q
    cannot meet that window, and enforcing it needs i_d (TI active braking). The scalar form holds
    it as `max(power, 0)` in `foc_iq_of_power`.
  - `foc_iq_limits` is built from the scalar `foc_iq_discharge_limit` / `foc_iq_charge_limit`
    through `interval_of_sign((vq < 0) ? NEG : POS, dis, chg)`, reusing the pieces since the two
    ends share no intermediates. The charge side rounds its power toward zero (≤ 1 LSB of power,
    32768 / |v_q| LSB of i_q).
  - It depends on the operating point (speed, v_q) and binds at nominal V with no status edge, so
    it needs periodic re-evaluation. Voltage-only forms (UV/OV ratio ramps, a Direction-keyed
    2/√3 · I_dc pair) change only on bus-status edges.
- **PHASE_BOUND**, approximate, no v_q dependence: |i_dq| ≤ 2/√3 · I_dc, applied as a q-circle limit
  after i_d. Keyed by the rotor speed sign: along the motion I_dis, against it min(I_dis, I_chg),
  symmetric I_dis at standstill.
  - Bounding i_q alone overshoots under field weakening (4.9×); the circle form is exact to a few
    LSB.
  - The current loop's voltage circle is V_dc / 2, so the phase bound reaches at most
    0.866 · I_dc; a √3/2 · rated cap sits above the reachable 3/4 · rated.
- **AC and DC limits are alternatives, not layers.** An AC limit derived from the same I_dc gives
  min(AC, DC) = AC everywhere; DC limits defaulted to √3/2 · rated reproduce the phase-rated
  behavior.
- **Power balance does not apply to phase current**: 2/3 · V_dc · I / |v_q| ≥ 2/√3 · I never binds.
  The AC pair maps to i_q as a sign pair by `Direction`; the only meaningful DC → AC map is
  `i_phase_of_i_dc` (PHASE_BOUND).

## Rollback under the half-plane

Forward torque during rollback needs v_q = R · i_q + E with E < 0. Once |E| > R · i_q the PI pins
v_q at 0, and the current is |E| / |Z| (90 mΩ: 18 V → 200 A at low speed), outside every I limit.
A symmetric VLimit keeps the limits enforced ([../../Decisions/0009_DirectionPolicy.md](../../Decisions/0009_DirectionPolicy.md)).

## Standstill and stall

- v_q → 0 saturates the power-balance window open. The align d-axis self-reference
  (p_d = Rs · i_d² ≈ 9 W at 10 A) never closes it.
- Near stall v_q ≈ R · i_q, so the bound references its own output and alternates each cycle. With
  a 200 A AC limit, an 800 A/s ramp gives 160 ↔ 167 A (+4 % i_dc); 40 kA/s gives 133 ↔ 200 A (+50 %).
  - The cycle amplitude is the capture period × the ramp slope: a 1 ms capture settles 800 A/s
    exactly, and 40 kA/s still gives +4 %.
  - The EMF form e = v_q − R · i_q settles exactly and absorbs ±30 % R error, since e is
    re-measured. Its return side flips by design at E² = 4R · |P_chg|. Cheaper alternative: LPF v_q.
- Steady-state dq decomposition: v = R · i + u, u = ω × L · i + e.
  - u · i is the air-gap (mechanical) power, reluctance included. The supply EMF form needs u_q.
  - e, the back EMF behind the reactances, serves sensorless (EEMF: ω · Lq on both axes), ψ online
    identification (e_q / ω) and the freewheel V match.
- Stall rating: at standstill one leg carries the peak |i_dq| as DC, so equal dissipation needs
  I_stall = I_rated_peak / √2, the RMS rating.

## Ramp and speed lead

- Clamping only the ramp input means a limit drop reaches the current loop at the ramp slope
  ([../../Decisions/0006_RampLimitsOnInput.md](../../Decisions/0006_RampLimitsOnInput.md)).
- With the torque limit binding, the speed setpoint runs ahead. Model: 2700 rpm/s ramp, the limit
  allows 1000 rpm/s, command 3000 → 1500 rpm at 1.5 s. Overshoot is +386 rpm, settling 0.8 s.
  SINAMICS-style tracking (lead held within a following error while saturated) gives +15 rpm with a
  100 rpm lead, +57 with 300.

## Operating point

Verified on the host against the debug motor model, 2026-10-17.

- Power split p = p_em + p_r, with p_em = (ω · ψ + ω · (L_d − L_q) · i_d) · i_q and p_r = R · |i|².
  Holds in fixed point to 3 LSB; τ · ω = p_em to 1 LSB.
- Voltage capability: at fixed ω and i_d, v(i_q) is a line, and the i_q window is its chord inside
  |v| ≤ V. Fixed point is within 0.06 A of exact; 632 B on the M0+ (64-bit divides). R cannot be
  neglected at low speed: the R-free shortcut overestimates about 2×.
- Rectification onset with the phases open: √3 · |e| = V_dc.

## Bus voltage and energy

- The freewheel speed limit is `SpeedRated` (back EMF = VNominal), so the speed policy alone never
  rectifies into the bus at OV.
- Source impedance swings the terminal voltage by R_int · I_dc, so UV/OV margins are DC-current
  bounds.
- 3.7 V/cell threshold ratios (113.5 % warning) put a 20s pack at 3.6 V/cell (84 V full) in Warning
  High at full charge; a voltage-ramp regen derate would cut regen there.
