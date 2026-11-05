# Sensorless, the detail

The detail behind [Sensorless.md](Sensorless.md): why the observer has its shape, what the float-plant runs found, the options evaluated and not built, and what is open. The math as geometry: [SensorlessMath.md](SensorlessMath.md).

## Predict and correct

- **Predict** θ̂ += ω̂ at `CAPTURE_ANGLE`, every control cycle, then publish: the shape of the Hall sensor's interpolation. The FOC Parks this cycle's sample at the prediction.
- **Correct** ω̂ only (`AnglePll_Correct`). The correction moves only the next prediction, so it runs after the write, off the write's deadline.
- **Before the FOC or after it is the same loop.** With the predict step, a correction before the FOC equals one after it with the InvPark at the stepped θ̂, one cycle ahead. The fresher sample adds Kp·π ≈ 3 % of the phase error per cycle. The order matters for a direct atan2 detector, not for a PLL. Advancing the InvPark angle is delay compensation for every sensor, not an observer choice.
- Rejected: the SMO's z before the FOC and its integration after it.

## The front end

- **EEMF on Lq.** Lq on both axes, the extended EMF: one `Stator_Winding_T` for every front end, SPM or IPM. The αβ SMO's G comes from Lq too, not Ls.
- **The frame θ̂ is the caller's.** The front ends take i_dq, v_dq already in θ̂: i Parked at θ̂, the held command at θ̂ − Δθ̂/2. The observer Parks the held command (`EmfObserver_ParkHold`), and i only when the FOC ran at another angle (`EmfObserver_ParkI`); when it ran at θ̂, i_dq is the FOC's take.
- **The struct keeps outputs.** An input consumed in the call is not stored, so the observer holds no dq image. The one held input is the command, `VOutAlpha`, `VOutBeta`: it spans the period the next sample closes. A stored dq image of v carried a stale value across `EmfObserver_Reset`.
- **The SMO step's order follows its v.** The same steps, one call shifted: `EmfObserver_AlphaBeta_ProcHeld` matches `EmfObserver_AlphaBeta_Proc` shifted one call, bit for bit.

  | v | Order | |
  |---|---|---|
  | The command just made, after the FOC | z, then the model | `_EmfObserver_ProcSmoNext`, `EmfObserver_AlphaBeta_Proc` |
  | The voltage held across the period the sample closes | the model, then z | `_EmfObserver_ProcSmo`, `EmfObserver_Eemf_Proc`, `EmfObserver_AlphaBeta_ProcHeld` |

  The EEMF front end needs the model first: its axis voltage takes the frame's turn on the sample that closes the period. Fed one v in the other's order, the step is a cycle skewed.
- **The frame's turn X·J·i couples on the measured i.** On î, the boundary layer's offset î − i ≈ E·thr/K crosses through X into the other axis, and θ̂ lags by atan(X·thr / (K + R·thr)) exactly: 6.4°, 14°, 27° at 100, 200, 400 Hz on the float plant below. Coupling on î and correcting the output, ê = z + (R + jX)(î − i), also removes it. Measured i is the simpler.
- **z is DC at lock in θ̂**, so the LPF takes the chatter without the phase lag it costs in the stationary frame.
- **The boundary layer adds its own lag**, the model's pole at 1 − G·(R + K/thr). `smo_sigma` saturates σ before `smo_z`: beyond ±1 it had wrapped.

## The tracker

- **One `AnglePll_T` for both detectors.** `AnglePll_ProcVector` takes a rotor vector in the stationary frame (the αβ front end), `AnglePll_ProcRelative` one already in θ̂ (dq). Each normalizes by ‖ê‖, reads 0 at or below `MagFloor`, then runs the lock and the correction.
- **The lock lives in AnglePll**, beside the magnitude it needs: `FilterTuning` {MagFloor, LockErrTol, LockHoldCount, LossHoldCount}, `Magnitude`, `LockCount`, `LossCount`. The scalar `AnglePll_Proc` has no lock: its error reads 0 below the floor, which would read as lock. Split the lock out only for a use without it; the split is then the detector {MagFloor, Magnitude, PhaseError} and the lock {LockErrTol, LockHoldCount, LossHoldCount, LockCount, LossCount}.
- **Direction is sign(ω̂)** (`_EmfObserver_Direction`), ω̂ = 0 read forward so the detector never stalls. The rotor vector −j·sign·ê takes out the EMF's sign flip with direction; without it the αβ front end locked 180° off in reverse. A wrong sign(ω̂) inverts the rotor vector: the start-up failure below.
- **Gains.** The error is normalized, ±1; ω̂ is angle16 per control cycle.
  - Kp 328 ≈ 0.01: crossover ≈ Kp·π rad per cycle, Kp·Fs/2 in Hz. 100 Hz at a 20 kHz control frequency, 50 Hz at 10 kHz.
  - Ki 51472 ≈ 1.57 per second, `KiPerTick` = Ki/SampleFreq: the PI zero at Ki/Kp ≈ 157 rad/s, 25 Hz at any control frequency.
  - LpfCoef from the control frequency: 100 Hz at any.
- **Check the crossover against the LPF first.** A crossover far above the LPF inside the loop leaves no phase margin. An earlier default, Kp 3280 with Ki 330 (≈ 0 per second, so P only), crossed at ~1 kHz, and every front end without a lag of its own failed at ≤ 200 Hz. The î coupling's lag had acted as rate feedback and hidden it: each looked fine alone.
- **Below MagFloor**, 5 % of V_max by default, every front end holds its seed, by design.

## State layout

- **Per-axis filter state is the observer's**: `EmfObserver_Axis_T` {`Stator_Axis_T Stator`, `Z`, `Emf`}, not slots on `Stator_Axis_T`. The measured axes share that type and would carry the slots dead, no Stator function touches them, and Z and Emf mean the same under any filter: Z the correction in place of e, Emf ê (a super-twisting SMO's w is its ê).
- `Stator_Axis_T` {I, IPrev, V} records one closed period; IPrev serves the asynchronous getters (`Stator_Axis_IDelta`).
- One front end runs per observer: a union of {Alpha, Beta} and {D, Q} would halve the axes' 48 B.
- The PID is set by `AnglePll_InitPid`, the tuning by `AnglePll_InitFilterTuning`. `EmfObserver_Config_T` holds `PID_Config_T Pll` beside `AnglePll_FilterTuning_T`, so no `AnglePll_Config_T` nests a `PID_Config_T`. How a Config nests another module's Config is open across the library.

## Start-up

- **The failure.** Seeded only at START_UP's entry, where the ramp is at 0 speed, θ̂ and ω̂ hold at the seed while ‖ê‖ ≤ MagFloor, and the rotor turns away from it. The first errors above the floor land at whatever phase the rotor has. About half drive ω̂ against it; sign(ω̂) then inverts the rotor vector, and the PLL never locks.
- **Built:** `StartUp_Proc` re-seeds from the ramp's angle and speed after the write while `EmfObserver_IsIndeterminate`. The first error above the floor then lands at the ramp's angle, which the rotor follows.
- **Float plant of the chain**, ALIGN → START_UP → lock in the C's order, the default config: an IPM motor, 4 pole pairs, 150 rpm/V, Ld 85 µH, Lq 165 µH, Rs 90 mΩ, on 18 V, 10 kHz control. Seeded at entry only, 22 of 36 runs lock; with the re-seed, 36 of 36.
- **Not built:** the rotor vector's sign from the commanded direction during START_UP, in place of sign(ω̂), also locks in the plant (8 of 8). It needs an observer field.

## The float-plant runs

A host float RL + EMF plant: SPM, R 0.0665, L 16.62, ψ 1.694 per unit per cycle at 20 kHz; deadbeat i_q; a 30° seed; the default gains. The harness is not kept.

| Front end | Result |
|---|---|
| EEMF SMO, `EmfObserver_Eemf_Proc`, wired | Locks at ±200 Hz motoring and braking, 100 and 400 Hz, i_q 0.3 and 0.6, within ±0.9°; sd ≤ 0.3° with ±0.005 noise |
| Voltage model, `EmfObserver_VModel_Proc` | Braking does not lock: sd 70° at LPF 100 Hz, 19.5° at 1 kHz |
| αβ SMO, `EmfObserver_AlphaBeta_Proc` | At LpfCoef 998, the 200 Hz EMF falls under the floor: the LPF passes 44 %. At ~1 kHz it locks, lagging ±23.6° |
| Super-twisting SMO, commented out | α·dt 0.001: ±200 Hz, all quadrants, within 1.6°. α·dt 0.01: within 1°, but the lock never sets: w's chatter trips LockErrTol |

- **The voltage model's braking.** The steady-state model omits L·di/dt, so θ̂'s own motion enters ê_d as L·i_q·(ω̂ − ω), and braking turns its sign against the detector. Stable only while Kp·π·L·|i_q| / E < 1. Options: the full stator EMF with L·Δi (noise gain L, 16.6 here), or a lower PLL bandwidth.
- **The αβ SMO's lag** needs atan(ω̂/ω_c) compensation. LpfCoef is shared, and the voltage model needs the low cutoff.

## The wiring

Built 2026-11-05: the correction on the FOC's take, form A, in an alternate RUN. It replaced `Motor_Sensorless_Thread`, which the application called after the PWM thread in every state, returning at once unless the adapter was the active sensor, and which redid the FOC's Clarke, θ̂'s sine and cosine, and i's Park.

- **The states own the correction.** Each corrects in its LOOP, after its write: `SENSORLESS_RUN` on the take, the start-up chain on the Clarke. Threads keep the procedures common to every state.
- **The take is the correction's i_dq when the FOC Parks at the published θ̂**, without `FOC_I_SAMPLE_DELAY` (with it, the take is at θ̂ − ω·delay). The chain's FOC Parks at the ramp, and shares only the Clarke (`EmfObserver_ParkI`).
- **The pieces**: `Motor_FOC_CaptureI` (SetTheta, Clarke, Park), `Motor_FOC_ProcIControl` (PI, InvPark), `_Motor_FOC_ProcIdReq`; `Motor_FOC_AngleControl` composes them. `EmfObserver_Eemf_ProcHeld` takes i_dq and Parks the command it holds; `EmfObserver_Eemf_ProcVAlphaBeta` takes the αβ command.
- **A, `Motor_Sensorless_CorrectOnTake`, after the write.** The LOOP stays every sensor's torque path (`_Motor_Run_Proc`); v(k−1) comes from the hold, then it holds v(k). 32.9 µs against the thread's 39.6, the arm 17.2 µs ahead of its trigger against 10.4.
- **One copy of the law.** `CorrectDq` is out of line: inlined into the three LOOPs it cost 1,728 B on KE06; out of line, 120 B over the thread.
- **An alternate RUN, not a RUN substate.** A substate runs into the tree's walk. Action inputs dispatch to the leaf only, so a substate's own `ON_PHASE` drops RUN's `ON_SPEED`, unless the input targets the top state (`_StateMachine_ActionInputRootState`). The LOOP walk is leaf first, so RUN's LOOP would run after the substate's, and the correction before the take. `SENSORLESS_RUN` is a top state with RUN's parts (`_Motor_Run_Entry`, `_Motor_Run_Proc`, `MOTOR_RUN_ACTION_TABLE`, `_Motor_TransitionFault`, `_Motor_TransitionFeedbackLost`, exported by `Motor_StateMachine.c`) and its own transition table. `.ID` is RUN's.
- **Scope: the acquisition procedure's states only.** The thread also corrected in PASSIVE, floating, and in INTERVENTION. Covering those would take sensorless variants of shared states and a selector at each of ~22 transitions into them. Instead every exit from `SENSORLESS_RUN` resets the observer (`ZERO_INITIAL`), so no state reads a lock nothing corrects, and its release and intervention requests go to PASSIVE. The other transitions into RUN test the lock first, so none reaches the plain RUN with this sensor.

## Evaluated, not built

### The correction between the take and the PI

**B, `Motor_Sensorless_ProcTorqueReq`.** v(k−1) is still in `Foc.Valpha`, `Vbeta`: no hold. A second copy of the loop. The write comes at 60.9 µs, past the top, and loads at the bottom 25 µs later: sample → load 87 µs, and the modeled voltage leads the applied one by 9.4° at 300 Hz, against A's 6.7°. Same inputs as A, the same ω̂.

Estimates at k = 1 on the sheet's timeline model: [SensorlessSequenceA.png](Simulation/Sensorless/SensorlessSequenceA.png), [SensorlessSequenceB.png](Simulation/Sensorless/SensorlessSequenceB.png), from [sensorless_sequence.py](Simulation/Sensorless/sensorless_sequence.py).

## References

The PLL's loop structure, error sign and seeding were checked against NXP's application notes in [Reference/](Reference/). AN12235 (p. 39) runs its observer on its own estimated position one calculation step delayed, as the hold angle does. A tan⁻¹(−e_d / e_q) detector has period π, a second lock at θ + π; the normalized −y / ‖ê‖ here is sin(θ̂ − θ), period 2π. Its tracking-observer gains, in continuous units: Kp = 4πξf₀, Ki = (2πf₀)².

## Open

- No hardware run. The start-up fix and the wired front end are checked on float plants only; the state wiring compiles and links, and has not run.
- **A coasting rotor after `SENSORLESS_RUN`.** The release resets ω̂, so speed feedback reads 0 while the rotor coasts, and a PWM request starts `SENSORLESS_ALIGN` on a turning rotor. Catch-spin and the intervention states are gone for this sensor until the procedure's DETECT and a sensorless intervention are built.
- **A chain abort keeps ω̂.** `OPEN_LOOP`'s exits are shared and do not reset the observer: PASSIVE then predicts on the ramp's ω̂, and speed feedback holds it. A PWM request waits on zero speed; `Motor_Sensorless_StartRunChain` restarts regardless.
- With `MOTOR_I_FEEDBACK_ON_PHASE`, the take is `ON_PHASE`'s, at the angle the LOOP last set, not this cycle's θ̂.
- The default PLL crossover is 50 Hz at a 10 kHz control frequency; its comment states 100 Hz, true at 20 kHz.
- The voltage model's braking, and the αβ SMO's lag: above.
- The super-twisting SMO is not ported: `StepEemfSta` and `sta_axis_step` are commented out, its config fields dropped.
- K_smo, SmoSat and the PLL gains are not tuned per motor.
- Config vars are read only: no Set, no NVM map entry. `LossHoldCount` has no var id.
- `EMF_ALPHA`, `EMF_BETA` read 0 under the EEMF front end.
- Speed feedback while unlocked.
