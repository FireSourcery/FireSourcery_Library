"""
The sensorless timeline of sensorless_sequence.py with its two fixes, not built, to compare with SensorlessSequence.png:

    1. The correction after the arm. At the 20 kHz PWM the PWM ISR between control cycles runs it, after its marks and its arm;
       at the 10 kHz PWM, one PWM ISR per control cycle, the arm already comes before it.
    2. The hold angle by the load. The command loads L = sample -> load after the sample, so the period the sample closes holds
       v(k-2) for L and v(k-1) for P - L, P the control period. v(k-1) Parked at θ̂ + (L/P − ½)·Δθ̂ in place of θ̂ − Δθ̂/2: its mean
       frame is then the applied one's, for a Vdq steady across the two commands.

Writes SensorlessSequenceFixed.png beside this file and prints the checks. Run from any directory.
"""
import os

from sensorless_sequence import (HERE, SPEED_POLL_US, CORRECTION, F_E, US_PER_INSTR, SEQ_BEFORE, T, SensorlessSim, arm_margin, cycles,
                                 margin_text, period_angles, plot_timeline, seg_times)


def next_cycle(sim):
    """Cycle k+1, behind the correction: how late its PWM ISR starts, when it writes from its top, and the load point that makes"""
    top = 2 * T
    write = seg_times(sim, "PWM ISR", "out", top)[1] - top
    return sim.pwm_delay.get(top, 0.0), write, "its next top" if write <= T else "the bottom after, 25 us later"


def main():
    sim = SensorlessSim(SEQ_BEFORE, 1.0, between=True).run()
    c = cycles(sim)
    c0 = c[0]
    lead = c0["load"] - c0["sample"]          # L, the sample -> load
    pa = period_angles(c, lead)
    runs = {(k, speed): SensorlessSim(SEQ_BEFORE, k, extra={0.0: SPEED_POLL_US} if speed else None, between=True).run()
            for k in (1.0, 1.2) for speed in (False, True)}
    k_late = next((k / 100 for k in range(100, 151) if next_cycle(SensorlessSim(SEQ_BEFORE, k / 100, between=True).run())[1] > T), None)

    def cycle_text(k, speed):
        delay, write, load = next_cycle(runs[(k, speed)])
        return f"cycle k+1 starts {delay:.1f} us late and writes {write:.1f} us in, for {load}"

    lines = [f"1. The correction after the arm, in the PWM ISR between control cycles: {c0['correct']:.1f} to {c0['hold']:.1f} us, "
             f"{sum(n for _, _, n in CORRECTION) * US_PER_INSTR:.1f} us at k 1, through the next set's ADC ISR.",
             f"   The arm keeps {margin_text(arm_margin(runs[(1.0, False)], 0.0))} of its trigger at k 1, {margin_text(arm_margin(runs[(1.2, False)], 0.0))} at k 1.2; "
             f"on the 1 ms speed tick {margin_text(arm_margin(runs[(1.0, True)], 0.0))}, {margin_text(arm_margin(runs[(1.2, True)], 0.0))}.",
             f"   The cost moves to the next control cycle: at k 1 {cycle_text(1.0, False)}; at k 1.2, {cycle_text(1.2, False)}.",
             f"   On the speed tick: at k 1 {cycle_text(1.0, True)}; at k 1.2, {cycle_text(1.2, True)}."
             + (f" Without it, the write passes the top from k {k_late:.2f}." if k_late else ""),
             f"2. The hold angle by the load: v(k-1) Parked at theta^(k) + (L/P - 1/2) * dtheta^, L the sample -> load {lead:.0f} us, P {2 * T:.0f} us: L/P = {lead / (2 * T):.2f}.",
             f"   The modeled voltage's mean frame meets the applied one's: {pa['mean_modeled'] - pa['mean_applied']:.1f} deg at {F_E:.0f} Hz, from 6.7. "
             "Exact for a Vdq steady across v(k-2) and v(k-1).",
             "   A Vdq step between them is modeled in v(k-1)'s for the L it isn't applied. Holding both and their L-weighted mean is exact, 2 more fields: not drawn.",
             "Unchanged: the FOC, its write and load, the prediction. Not drawn: the sense chain's age."]
    plot_timeline(sim, os.path.join(HERE, "SensorlessSequenceFixed.png"),
                  "Sensorless with the fixes, not built: the correction after the arm, the hold angle by the load\n"
                  "KE06 timing ('before', ../ISample): the 20 kHz PWM, a 100 µs control cycle, the set on the trigger 16.5 µs before the top",
                  lead, "correct: i(k) at θ̂(k), v(k−1) at θ̂(k) + (L/P − ½)·Δθ̂",
                  f"the observer's voltage for that period: v(k−1) Parked at θ̂(k) + (L/P − ½)·Δθ̂, L/P = {lead / (2 * T):.2f}",
                  "the hold angle turned by the sample → load", lines)


if __name__ == "__main__":
    main()
