# Sensorless, the detail

The detail behind [Sensorless.md](Sensorless.md): why the observer has its shape, what the float-plant runs found, the options evaluated and not built, and what is open. The math as geometry: [SensorlessMath.md](SensorlessMath.md).

## Predict and correct

- **Predict** θ̂ += ω̂ at `CAPTURE_ANGLE`, every control cycle, then publish: the shape of the Hall sensor's interpolation. The FOC Parks this cycle's sample at the prediction.
- **Correct** ω̂ only (`AnglePll_Correct`). The correction moves only the next prediction, so it runs after the write, off the write's deadline.
- **Before the FOC or after it is the same loop.** With the predict step, a correction before the FOC equals one after it with the InvPark at the stepped θ̂, one cycle ahead. The fresher sample adds Kp·π ≈ 3 % of the phase error per cycle. The order matters for a direct atan2 detector, not for a PLL. Advancing the InvPark angle is delay compensation for every sensor, not an observer choice.
- Rejected: the SMO's z before the FOC and its integration after it.

## The front end

- **EEMF on Lq.** Lq on both axes, the extended EMF: one `Stator_Winding_T` for every front end, SPM or IPM. The αβ SMO's G comes from Lq too, not Ls.
- **The frame θ̂ is the caller's.** The front ends take i_dq, v_dq already in θ̂: i Parked at θ̂, the held command at θ̂ − Δθ̂/2. The observer Parks the held command (`EmfObserver_ParkHold`); i_dq is the FOC's take. Where the FOC ran at another angle, `EmfObserver_ParkI` Parks Iα, Iβ at θ̂: not wired.
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

Built 2026-11-05: the correction on the FOC's take, form A, as the sensor's `CAPTURE_STATOR`, after the state machine. It replaced `Motor_Sensorless_Thread`, which the application called after the PWM thread in every state, returning at once unless the adapter was the active sensor, and which redid the FOC's Clarke, θ̂'s sine and cosine, and i's Park.

- **A sensor procedure, after the state machine.** `Motor_PWM_Thread`: `CAPTURE_ANGLE`, the state machine, then `Motor_CaptureStator`, then the speed poll. The predict before and the correction after are both the active sensor's, through its vtable; Hall, Encoder and SinCos define `CAPTURE_STATOR` empty. No state, and no thread, tests which sensor is active.
- **Every cycle, on the caller's constraints.** `Motor_CaptureStator` calls the sensor every cycle, in every state. The correction is valid when `Foc` holds this cycle's take and the command just written, as in every state that drives the FOC through `Motor_FOC_WriteDuty`. PASSIVE floating writes nothing, but its V capture writes `Valpha`, `Vbeta` as the command's would be (`FOC_ProcVBemfClarkePark`), with Id, Iq at 0 after the feedback clear: the observer corrects on the terminal EMF, as DETECT would. PASSIVE shorted, FAULT, DEACTIVATED and INIT capture nothing: the observer corrects on the last Vα, Vβ, with Id, Iq at 0. The constraints are commented on `Motor_CaptureStator`.
- **The take's Id, Iq, in every state.** They are θ̂'s where the FOC Parks at θ̂: RUN and INTERVENTION. Removed 2026-11-05, to return with `FOC_I_SAMPLE_DELAY`: a frame test (`FOC_IsIdqAt`, on `FOC_T.Theta`, the angle `FOC_SetTheta` last held) that Parked Iα, Iβ at θ̂ elsewhere, and a reset of the observer uncorrected past `LossHoldCount` (`EmfObserver_ProcStale`), which the every-cycle call never fired.
- **The pieces**: `Motor_FOC_CaptureI` (SetTheta, Clarke, Park), `Motor_FOC_ProcIControl` (PI, InvPark), `_Motor_FOC_ProcIdReq`; `Motor_FOC_AngleControl` composes them. `EmfObserver_Eemf_ProcHeld` takes i_dq and Parks the command it holds; `EmfObserver_Eemf_ProcVAlphaBeta` takes the αβ command.
- **Cost.** 32.9 µs against the thread's 39.6, estimated; the arm 17.2 µs ahead of its trigger against 10.4. KE06 Release 50,568 B.
- **Not a state's LOOP.** Corrected in the LOOPs, each state that drives at θ̂ needed the call, and an alternate RUN (`SENSORLESS_RUN`) carried it with RUN's parts exported; a RUN substate would run its LOOP before RUN's, the correction before the take. `SENSORLESS_RUN` is kept commented out, for B.

## Evaluated, not built

### The correction between the take and the PI

**B, `Motor_Sensorless_ProcTorqueReq`.** v(k−1) is still in `Foc.Valpha`, `Vbeta`: no hold. A second copy of the loop. The write comes at 60.9 µs, past the top, and loads at the bottom 25 µs later: sample → load 87 µs, and the modeled voltage leads the applied one by 9.4° at 300 Hz, against A's 6.7°. Same inputs as A, the same ω̂.

Estimates at k = 1 on the sheet's timeline model: [SensorlessSequenceA.png](Simulation/Sensorless/SensorlessSequenceA.png), [SensorlessSequenceB.png](Simulation/Sensorless/SensorlessSequenceB.png), from [sensorless_sequence.py](Simulation/Sensorless/sensorless_sequence.py).

## References

The PLL's loop structure, error sign and seeding were checked against NXP's application notes in [Reference/](Reference/). AN12235 (p. 39) runs its observer on its own estimated position one calculation step delayed, as the hold angle does. A tan⁻¹(−e_d / e_q) detector has period π, a second lock at θ + π; the normalized −y / ‖ê‖ here is sin(θ̂ − θ), period 2π. Its tracking-observer gains, in continuous units: Kp = 4πξf₀, Ki = (2πf₀)².

## Open

- No hardware run. The start-up fix and the wired front end are checked on float plants only; the state wiring compiles and links, and has not run.
- **The observer outside a writing state.** PASSIVE takes the set that lands (`Motor_FOC_ProcCaptureAngleVBemf`): floating, the terminal EMF with I cleared; shorted, the current with V cleared. The feedback clear now zeroes the αβ image with the rest, so DEACTIVATED and a state's entry read 0. None of it has run, on hardware or a plant. Accepted: the held v is the last sample's, Parked at θ̂ − Δθ̂/2, Δθ̂/2 off that sample's own angle (5.4° at 300 Hz electrical, 10 kHz); the first PASSIVE cycle holds the last RUN command. FAULT keeps its last image, uncleared, and leaves through DEACTIVATED.
- **Speed feedback reads 0 while ‖ê‖ is at or below the floor** (`Sensorless_Sensor_CaptureSpeed`), where ω̂ holds where the error stopped. Above it, a lock lost at speed keeps the speed for the freewheel decision. A rotor coasting below the observable speed reads 0, so a PWM request starts `SENSORLESS_ALIGN` on it: the procedure's SETTLE region. During START_UP below the floor, speed reads 0, so a direction input is accepted.
- **The closed loop's minimum, built.** `MagRun` (default 10 % of VBus, twice the floor) gates `IS_FEEDBACK_AVAILABLE`: RUN leaves while θ̂ is still the rotor's, START_UP hands over only above it, PASSIVE catches only above it. A standstill lock can't pass it. It is ‖ê‖, not ω̂: no ψ. Read only (`EMF_OBSERVER_CONFIG_VAR_RUN_EMF_MIN`, not in the app). It must sit below ‖ê‖ at the ramp's end speed, or START_UP never hands over.
- **The open-loop fallback** is a build option as field weakening is: `MOTOR_OPEN_LOOP_FALLBACK_ENABLE` (default off, `Motor.h`; the `OpenLoopFallback` version flag) compiles its one reference, and `MagRun` is its runtime control as `IdLimit` is field weakening's: 0 releases the minimum and the fallback with it. It imposes the ramp's speed, not the command: a speed-held load's, not traction's. A command below `MagRun`'s speed cycles RUN, fallback, ramp up, RUN. Entering OPEN_LOOP clears the feedback state and holds V0 until the first write, one control cycle.
- **A coast before ALIGN.** Speed reads 0 below the floor, turning or not, so a PWM request in PASSIVE can't tell a stopped rotor from a coasting one. `Motor_Sensorless_StartUpState` enters `SENSORLESS_COAST`, a PASSIVE substate, until PASSIVE has lasted `MOTOR_SENSORLESS_COAST_CYCLES` (default 1 s, `#ifndef` in `Motor_Sensorless.h`), counted on `ControlTimerBase` from PASSIVE's entry, which entering and leaving the substate don't repeat. No entry or LOOP: PASSIVE's phase state and capture carry on. Its NEXT cancels on a cleared direction, goes to RUN where the observer reaches feedback, else ALIGN at the coast time. Its own PHASE_OUTPUT row overrides PASSIVE's: Z or 0 cancels, back to PASSIVE with PASSIVE's phase action. Commands keyed to PASSIVE (`Motor_Disable`) reach it. `Motor_Sensorless_StartRunChain`, a command, goes straight to ALIGN. A request after a longer PASSIVE goes straight to ALIGN. The request is taken and waited out, not refused: the application sends it once, on the throttle's edge. A constant, not a config value: a coast that has to follow the load is a config field.
- **The start-up chain's i is in the ramp's frame.** Once ‖ê‖ holds above the floor and θ̂ leaves the ramp, the correction reads the take off θ̂ by the load angle, while it acquires the lock. `EmfObserver_ParkI` is the fix, deferred with the sample delay.
- **Calibration and open loop correct too.** They write through `Motor_FOC_WriteDuty`. At standstill with a wrong R, ê = (R − R̂)·i can read above the floor and lock.
- With `MOTOR_I_FEEDBACK_ON_PHASE`, the take is `ON_PHASE`'s, at the angle the LOOP last set, not this cycle's θ̂.
- The default PLL crossover is 50 Hz at a 10 kHz control frequency; its comment states 100 Hz, true at 20 kHz.
- The voltage model's braking, and the αβ SMO's lag: above.
- The super-twisting SMO is not ported: `StepEemfSta` and `sta_axis_step` are commented out, its config fields dropped.
- K_smo, SmoSat and the PLL gains are not tuned per motor.
- Config vars are read only: no Set, no NVM map entry. `LossHoldCount` has no var id.
- `EMF_ALPHA`, `EMF_BETA` read 0 under the EEMF front end.
- Speed feedback while unlocked above the floor: ω̂ as the PLL holds it, until the lock's loss.
