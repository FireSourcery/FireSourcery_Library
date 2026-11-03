"""
KE current-loop sequences: where the I sample is captured, Parked and run through the PI, and when its output loads.
The record of how the built sequence, one set per control cycle on the trigger, was chosen: CurrentLoopSequencing.exp.md
(Kelly_KMC/Docs), Derivation. The forms and their ids: isample_sequences.py. The model and the scheduler: isample_model.py.
The figures: isample_plots.py.

Run from this directory. Writes one timeline per sequence (PwmSampleSequence_<name>.png), the panels (LoadTopPanels: the built
form and the off cycle's write; LoadBottomPanels: the requests on the off cycle; OtherChannels), the start calibration and the
k sweep summary (PwmSampleSequenceSummary.png) beside this file, and the console tables to pwm_sample_sequences.txt. Archive/
holds the renders from before 2026-11-04, when the triggered forms were named 3A and 3B.
"""
import os
import sys

from isample_model import BASE, HOLD_HALF_US, AGE_US, Sim, delay_clock, k_range, start_text, variant, walk_conv
from isample_plots import plot_order_compare, plot_sequence, plot_start_calibration, plot_summary
from isample_sequences import (HT_ROWS, K_EVERY, LEAD_AFTER_FAST, LEAD_AFTER_VW, LEAD_BOTTOM_CLEAR, LEAD_BOTTOM_KE, LEAD_BOTTOM_LATEST, LEAD_BUILT, LEAD_EVERY_W,
                               ORDERS, ORDER_DEFAULTS, R_AFTER, R_AFTER_MISS, R_AFTER_V_MISS, R_BEFORE, R_BEFORE_V, R_BOTTOM, R_BOTTOM_WALK,
                               SEQ_AFTER, SEQ_AFTER_V, SEQ_BEFORE, SEQ_BEFORE_V, SEQ_BOTTOM, SEQ_BOTTOM_V, SEQ_BOTTOM_WALK, SEQ_EVERY, SEQS, VBUS_OFF)


class Tee:
    def __init__(self, path):
        self.file, self.out = open(path, "w", encoding="utf-8"), sys.stdout

    def write(self, s):
        self.file.write(s)
        self.out.write(s)

    def flush(self):
        self.file.flush()
        self.out.flush()


here = os.path.dirname(os.path.abspath(__file__))
sys.stdout = Tee(os.path.join(here, "pwm_sample_sequences.txt"))
nominal = {s.key: Sim(s, 1.0).run() for s in SEQS}

print("Nominal durations (k = 1), us: " + ", ".join(f"{n} {v}" for n, v in BASE.items()))
print(f"{'seq':9s} {'sample':>7s} {'Park from':>9s} {'offset':>7s} {'PI end':>7s} {'write':>7s} {'load':>6s} {'smp->load':>9s} {'actuation':>9s} {'CPU analog':>10s} {'off':>5s} {'PWM delay':>9s}")
for s in SEQS:
    r = nominal[s.key].result()
    print(f"{s.key:9s} {r['sample']:7.1f} {r['park_tick']:9.0f} {r['park_offset']:+7.1f} {r['pi_end']:7.1f} {r['write']:7.1f} {r['load']:6.0f}"
          f" {r['sample_to_load']:9.1f} {AGE_US + r['sample_to_load'] + HOLD_HALF_US:9.0f} {r['busy_analog']:10.1f} {r['busy_off']:5.1f} {r['pwm_delay']:9.1f}")

for s in SEQS:
    if s.file:
        plot_sequence(s, nominal[s.key], os.path.join(here, f"PwmSampleSequence_{s.file}.png"))
plot_sequence(SEQ_BOTTOM_V, Sim(SEQ_BOTTOM_V, 1.0).run(), os.path.join(here, f"PwmSampleSequence_{SEQ_BOTTOM_V.file}.png"))
plot_sequence(SEQ_EVERY, Sim(SEQ_EVERY, K_EVERY).run(), os.path.join(here, f"PwmSampleSequence_{SEQ_EVERY.file}.png"), k=K_EVERY)

# Sweep the speed factor
ks = [0.4 + 0.02 * i for i in range(61)]
sweep = {s.key: [Sim(s, k).run().result() for k in ks] for s in SEQS}
print("\nsample -> load (us) by speed factor k")
print("k     " + " ".join(f"{s.key:>9s}" for s in SEQS))
for i, k in enumerate(ks):
    if i % 5 == 0:
        print(f"{k:4.2f}  " + " ".join(f"{sweep[s.key][i]['sample_to_load']:9.1f}" for s in SEQS))

# The LOOP's order, and VBus's divide moved out of the ADC ISR
order_ks = (0.6, 0.8, 1.0, 1.2, 1.4)
print("\nsample -> load (us) by LOOP order; the write at k = 1")
print(f"{'':28s}" + " ".join(f"k {k:<4}" for k in order_ks) + "  write")
for name, s in ORDER_DEFAULTS + ORDERS + VBUS_OFF:
    print(f"{name:28s}" + " ".join(f"{Sim(s, k).run().result()['sample_to_load']:6.1f}" for k in order_ks) + f"  {Sim(s, 1.0).run().result()['write']:5.1f}")

# Triggered, each variant at the start its load point allows
print(f"\nthe latest starts at k 1. The bottom: bottom {start_text(LEAD_BOTTOM_LATEST)}, bottom VBus off {start_text(SEQ_BOTTOM_V.lead)},"
      f" every {start_text(SEQ_EVERY.lead)} and every, wait {start_text(LEAD_EVERY_W)} at k {K_EVERY}."
      f" The next top: after {start_text(SEQ_AFTER.lead)}, {start_text(LEAD_AFTER_FAST)} at k 0.8; after+V, wait {start_text(LEAD_AFTER_VW)}")

# Loads at the next top: the built form, its start after the top, the miss, and the off cycle's write
plot_order_compare([(f"Built at the 20 kHz PWM: the set {start_text(LEAD_BUILT)}, the PWM ISR between control cycles marks and arms it, {R_BEFORE['arm_set']:.1f} us ahead of its"
                     f" trigger, and runs no LOOP. S32's form, the setup in software.\nThe LOOP takes the set after the requests; the write at {R_BEFORE['write']:.1f} us loads at the"
                     f" next top: {R_BEFORE['sample_to_load']:.1f} us, to the sweep's end at k {k_range(SEQ_BEFORE)[1]:.1f}. Measured 2026-11-03: the write at 30.6 us", SEQ_BEFORE),
                    (f"The set after the top, {start_text(SEQ_AFTER.lead)}, the latest the take finds at k 1 (the delay at {delay_clock(SEQ_AFTER.lead)}): {R_AFTER['sample_to_load']:.1f} us,"
                     f" to k {k_range(SEQ_AFTER)[1]:.2f}. The between ISR as above.\nFaster code misses: the margin runs the other way from a start before the top", SEQ_AFTER),
                    (f"The miss: the same start, a cycle 20 % faster (k 0.8). The take runs before the set lands, the between ISR takes nothing, so the next control cycle takes it"
                     f" at its own angle: {R_AFTER_MISS['sample_to_load']:.1f} us", SEQ_AFTER, 0.8),
                    (f"The miss with the off cycle's LOOP (after+V): the off cycle takes the set, runs the PI and writes: {R_AFTER_V_MISS['sample_to_load']:.1f} us."
                     " The LOOP every 50 us catches a late set 50 us sooner; its arm is not drawn", SEQ_AFTER_V, 0.8),
                    (f"Not adopted, built 2026-10-25 to 11-02 (before+V): the set {start_text(LEAD_BUILT)}, the off cycle's LOOP writes the held V at its own angle: {R_BEFORE_V['sample_to_load']:.1f} us."
                     f" Each write is applied one PWM period, the angle sawtooth halved;\nthe PI and |V| still at 10 kHz. CPU per 100 us {R_BEFORE_V['busy_analog'] + R_BEFORE_V['busy_off']:.0f}"
                     f" against {R_BEFORE['busy_analog'] + R_BEFORE['busy_off']:.0f}, and the I-loop divider in the library", SEQ_BEFORE_V)],
                   os.path.join(here, "PwmSampleSequence_LoadTopPanels.png"),
                   title="Triggered, loads at the next top: the set's start, and what the PWM ISR between control cycles does", t_hi=160.0)

# Loads at the bottom: the requests on the off cycle, the start set by the ADC ISR's end
_r = {s: Sim(s, 1.0).run().result() for s in (SEQ_BOTTOM, variant(SEQ_BOTTOM, lead=LEAD_BOTTOM_CLEAR), SEQ_BOTTOM_V)}
plot_order_compare([(f"Built 2026-11-02 to 11-04: the requests on the off cycle, started {start_text(LEAD_BOTTOM_KE)}, in the bus clock's reach."
                     f" The ADC ISR ends {_r[SEQ_BOTTOM]['pwm_delay']:.1f} us past the top; the write makes the bottom to k {k_range(SEQ_BOTTOM)[1]:.2f}", SEQ_BOTTOM),
                    (f"Started {start_text(LEAD_BOTTOM_CLEAR)}: the ADC ISR ends before the top. 1.5 us older, to k {k_range(variant(SEQ_BOTTOM, lead=LEAD_BOTTOM_CLEAR))[1]:.2f}",
                     variant(SEQ_BOTTOM, lead=LEAD_BOTTOM_CLEAR)),
                    (f"VBus's divide in the off cycle, the ADC ISR {BASE['vsource'] - BASE['vbus_store']:.1f} us shorter: started {start_text(SEQ_BOTTOM_V.lead)},"
                     f" the latest for the bottom (the delay at {delay_clock(SEQ_BOTTOM_V.lead)}), to k {k_range(SEQ_BOTTOM_V)[1]:.2f}", SEQ_BOTTOM_V),
                    (f"VBus's divide in the off cycle, started {start_text(LEAD_BOTTOM_KE)}, the built start: the write makes the bottom to k {k_range(variant(SEQ_BOTTOM_V, lead=LEAD_BOTTOM_KE))[1]:.2f}",
                     variant(SEQ_BOTTOM_V, lead=LEAD_BOTTOM_KE))],
                   os.path.join(here, "PwmSampleSequence_LoadBottomPanels.png"),
                   title="Triggered, loads at the bottom: the requests on the off cycle, the start set by the ADC ISR's end. Each write holds one PWM period, and must land within half of one:"
                         " KE06's came at 29 us", t_hi=110.0)

# The main-loop channels: walked after the set until 2026-11-03, on the next trigger since, in the one FIFO since 11-04
plot_order_compare([(f"Until 2026-11-03: the set alone on the trigger, its completion walks the throttle and brake by software."
                     f" Their completion delays the write to {R_BOTTOM_WALK['write']:.1f} us, past the bottom", variant(SEQ_BOTTOM_WALK, walk=2)),
                    (f"11-03 to 11-04: the sets alternate. The taking cycle arms the throttle and brake after its write, {R_BOTTOM['arm_others']:.1f} us ahead of their trigger;"
                     f" the write at {R_BOTTOM['write']:.1f} us", SEQ_BOTTOM),
                    (f"The same, all 7 main-loop channels marked: {walk_conv(7):.1f} us of conversions, done {SEQ_BOTTOM.lead - walk_conv(7):.1f} us before the off cycle's top,"
                     " where nothing waits on them", variant(SEQ_BOTTOM, others=7))],
                   os.path.join(here, "PwmSampleSequence_OtherChannels.png"),
                   title="Triggered, loads at the bottom, on KE06: the main-loop channels, walked after the set, or on the off cycle's trigger. Since 11-04 they ride behind the set in the one FIFO",
                   t_hi=110.0)

print("\nalternating sets, each arm ahead of its trigger (us), all 7 others marked, by k")
for k in (1.0, 1.2, 1.4, 1.5):
    r = Sim(variant(SEQ_BOTTOM, others=7), k).run().result()
    print(f"  k {k:<4}  the I set's arm {r['arm_set']:5.1f}  the others' arm {r['arm_others']:5.1f}")

# Triggered by load point, sample -> load by k. * the take missed; a later cycle took the set
HT_KS = (0.8, 0.9, 1.0, 1.1, 1.2)
print(f"\nTriggered by load point: sample -> load (us) by k; * the take missed. 'every' runs at k x {K_EVERY}, a stand-in for S32")
print(f"{'':58s}{'delay':>10s} " + " ".join(f"k {k:<5}" for k in HT_KS) + "  holds, k     write  CPU, taking / next cycle")
_group = None
for group, name, s, kf in HT_ROWS:
    if group != _group:
        print(f"loads at {group}")
        _group = group
    cells = []
    for k in HT_KS:
        r = Sim(s, k * kf).run().result()
        cells.append(f"{r['sample_to_load']:6.1f}{' ' if r['hit'] else '*'}")
    r1 = Sim(s, kf).run().result()
    lo, hi = k_range(s, kf)
    print(f"  {name:56s}{('S32, PDB' if s.divider == 1 else delay_clock(s.lead)):>10s} " + " ".join(cells) + f"  {lo:.2f}-{hi:.2f}  {r1['write']:5.1f}  {r1['busy_analog']:4.1f} / {r1['busy_off']:4.1f}")

plot_start_calibration(os.path.join(here, "PwmSampleSequence_StartCalibration.png"),
                       [("bottom: the requests on the off cycle", SEQ_BOTTOM, "tab:blue", LEAD_BOTTOM_LATEST, "the latest for the bottom"),
                        ("before / after: the requests every cycle, the between ISR arms", SEQ_BEFORE, "tab:brown", SEQ_AFTER.lead, "the latest the take finds, the next top")],
                       LEAD_BUILT)

styles = {"1a": ("tab:purple", "-"), "1b": ("tab:purple", ":"), "1c": ("tab:green", "-"), "2a": ("tab:orange", "-"), "2b": ("tab:red", "--"),
          "before": ("tab:blue", "-"), "after": ("tab:brown", "-"), "before+V": ("tab:blue", "--"), "after+V": ("tab:brown", "--"), "bottom": ("tab:cyan", "-")}
plot_summary(SEQS, ks, sweep, styles, os.path.join(here, "PwmSampleSequenceSummary.png"))
print("\nfigures written beside this file")
