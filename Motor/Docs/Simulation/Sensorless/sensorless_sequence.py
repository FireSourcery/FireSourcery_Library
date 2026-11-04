"""
The sensorless observer on the built current-loop timeline: the value each step reads and writes, when it was produced, and when
the duty it leads to is applied. The ISR timing is ISample's built form ('before': KE06, the 20 kHz PWM, a 100 us control cycle),
with the observer's correction (Motor_Sensorless_Thread) added after the write and its prediction in place of the Hall capture.

    isample_model.py, isample_sequences.py, isample_plots.py   the scheduler, the built sequence, the lanes' drawing (../ISample)
    Motor/Docs/Sensorless.md, Timeline                         the doc that states the results

Writes SensorlessSequence.png beside this file and prints the checks. Run from any directory.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ISample"))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch, Polygon  # noqa: E402

from isample_model import LOAD, SAMPLE_OFFSET, T, Sim  # noqa: E402
from isample_plots import KIND_COLOR, KIND_LABEL, LANES, _conversions, _counter, _trace  # noqa: E402
from isample_sequences import SEQ_BEFORE  # noqa: E402

US_PER_INSTR = 0.032   # isample_model.py's estimate: 50 MHz, CPI ~1.6
PREDICT_US = 0.8       # CAPTURE_ANGLE: AnglePll_Predict and the publish, in place of the Hall capture's 3.0
# The correction, by instructions on the driven path of the linked KE06 image (Motor_Sensorless_Thread and its callees)
CORRECTION = (
    ("frames", "obs_frame", 278),   # the active-sensor test, Clarke, sin/cos of θ̂ and of θ̂ − Δθ̂/2, the 2 Parks
    ("SMO d, q", "obs_smo", 615),   # X = ω̂·L, then per axis: the model, z, the LPF. 6 64-bit multiplies, 2 divides
    ("detector, lock", "obs_det", 261),  # the rotor vector, its magnitude (a square root), the normalizing divide, the lock
    ("PI", "pi", 45),               # AnglePll_Correct: PID_ProcPI
    ("hold", "obs_hold", 40),       # Phase_ReadVOut, EmfObserver_HoldV
)
SPEED_POLL_US = 17.0   # the 1 ms speed poll's tick, in Motor_PWM_Thread before the correction: the board's note
F_E = 300.0            # electrical Hz, for the angles
DEG_PER_US = 360.0 * F_E * 1e-6

KIND_COLOR.update({"obs_frame": "#80deea", "obs_smo": "#f48fb1", "obs_det": "#ffe082", "obs_hold": "#c5e1a5"})
KIND_LABEL.update({"obs_frame": "correction: Clarke, the 2 Parks", "obs_smo": "correction: the SMO, d and q",
                   "obs_det": "correction: detector, lock", "obs_hold": "correction: hold the command"})


class SensorlessSim(Sim):
    """The built sequence with the observer selected: the prediction at CAPTURE_ANGLE, the correction after the write"""

    def __init__(self, seq, k, extra=None, between=False, **kw):
        super().__init__(seq, k, **kw)
        self.d["sensor"] = PREDICT_US * k
        self.extra = extra or {}  # tick -> us of one more segment ahead of the correction: the speed poll
        self.between = between    # the correction in the between ISR, after its arm: an option, not built

    def pwm_isr(self, tick, analog):
        for sg in super().pwm_isr(tick, analog):
            if sg["label"] == "exit":
                if analog and tick in self.extra:
                    yield self.seg("speed", "theta", self.extra[tick] * self.k)
                if analog != self.between:
                    for label, kind, n in CORRECTION:
                        yield self.seg(label, kind, n * US_PER_INSTR * self.k)
            yield sg


def seg_times(sim, lane, label, tag):
    rows = [r for r in sim.trace if r[0] == lane and r[1] == label and r[5] == tag]
    return (rows[0][3], rows[-1][4]) if rows else (math.nan, math.nan)


def cycle_times(sim, tick):
    """The instants of control cycle [tick]: its sample, the capture, the prediction, the take, the write, the load, the correction"""
    ct = tick + (T if sim.between else 0.0)  # the PWM ISR the correction runs in
    t = dict(sample=sim.batches[tick]["start"] + SAMPLE_OFFSET, capture=seg_times(sim, "ADC ISR", "I", tick)[1],
             predict=seg_times(sim, "PWM ISR", "sensor", tick)[1], take=seg_times(sim, "PWM ISR", "Park", tick)[0],
             write=seg_times(sim, "PWM ISR", "out", tick)[1], correct=seg_times(sim, "PWM ISR", "frames", ct)[0],
             smo=seg_times(sim, "PWM ISR", "SMO d, q", ct)[1], pi=seg_times(sim, "PWM ISR", "PI", ct)[1],
             hold=seg_times(sim, "PWM ISR", "hold", ct)[1])
    t["load"] = (math.floor(t["write"] / LOAD + 1e-9) + 1) * LOAD
    return t


def arm_margin(sim, tick):
    """The between ISR's arm after control cycle [tick], ahead of the trigger it arms for. Negative: it misses, the set waits a control cycle"""
    return (tick + 2 * T - sim.seq.lead) - sim.arm_end.get(tick + T, math.nan)


def margin_text(m):
    return f"{m:.1f} us ahead" if m >= 0 else f"misses by {-m:.1f} us"


def unicode_text(ascii_text):
    """The notes print in ASCII, for the console's cp1252, and draw in their symbols"""
    for a, u in (("deg", "°"), ("k-1", "k−1"), ("k-2", "k−2"), ("->", "→"), ("dtheta^", "Δθ̂"), ("theta^", "θ̂"), ("1/2", "½"), (" * ", "·")):
        ascii_text = ascii_text.replace(a, u)
    return ascii_text


def idx(j):
    return "k" if j == 0 else (f"k+{j}" if j > 0 else f"k−{-j}")


# Value lanes, top to bottom: (name, color)
VLANES = [("PhaseInput.I", "#bbdefb"), ("θ̂  Pll.Base.Angle,\nSensorState", "#c8e6c9"), ("ω̂  Pll.Base.Delta", "#a5d6a7"),
          ("command  Foc.Vd,q → Vα,β", "#ffe0b2"), ("applied  the duty", "#ffcc80"), ("held  VOutAlpha, VOutBeta", "#f8bbd0"),
          ("observer  î, z, ê", "#f48fb1")]
VY = {name: len(VLANES) - 1 - i for i, (name, _) in enumerate(VLANES)}
VNAME = [name for name, _ in VLANES]
I_, TH, OM, CMD, APP, HELD, OBS = VNAME


def bus(ax, a, b, y, label, color, strong):
    d, h = 0.9, 0.32
    ax.add_patch(Polygon([(a, y), (a + d, y + h), (b - d, y + h), (b, y), (b - d, y - h), (a + d, y - h)], closed=True, facecolor=color,
                         edgecolor="black" if strong else "#9e9e9e", linewidth=1.0 if strong else 0.5))
    if b - a > 6:
        ax.text((a + b) / 2, y, label, ha="center", va="center", fontsize=7, fontweight="bold" if strong else "normal")


def cycles(sim):
    return {j: cycle_times(sim, j * 2 * T) for j in (-2, -1, 0, 1)}


def rel(t_ref, t):
    """The angle at t_ref from the rotor at t, locked"""
    return DEG_PER_US * (t_ref - t)


def period_angles(c, hold_lead):
    """
    The period i(k) closes, from sample k−1 to sample k: v(k−2) applied until v(k−1) loads. The observer models v(k−1) across all of it,
    turned back by [hold_lead] us of rotation: 0 for the hold angle θ̂ − Δθ̂/2. The means of the applied and the modeled voltage's frame.
    """
    p0, p1 = c[-1]["sample"], c[0]["sample"]
    n_pts = 400
    ts = [p0 + (p1 - p0) * (i + 0.5) / n_pts for i in range(n_pts)]
    applied = [rel(c[-2]["sample"] if t < c[-1]["load"] else c[-1]["sample"], t) for t in ts]
    modeled = [rel(c[-1]["sample"] - hold_lead, t) for t in ts]
    return dict(p0=p0, p1=p1, split=c[-1]["load"] - p0, mean_applied=sum(applied) / n_pts, mean_modeled=sum(modeled) / n_pts)


def plot_timeline(sim, path, title, hold_lead, correct_label, modeled_label, gap_label, lines):
    """The figure: the counter, the ISRs, the values each step reads and writes, the angles. [lines] the notes"""
    c = cycles(sim)
    t_lo, t_hi = -125.0, 165.0

    # Value histories: (start, end, label, cycle) per lane. A value lasts from its write to the next
    color = dict(VLANES)
    hist = {name: [] for name in VNAME}

    def track(name, key, label):
        for j in (-2, -1, 0, 1):
            nxt = c[j + 1][key] if j + 1 in c else t_hi + 10
            hist[name].append((c[j][key], nxt, label(j), j))
    track(I_, "capture", lambda j: f"i({idx(j)})")
    track(TH, "predict", lambda j: f"θ̂({idx(j)}) = θ̂({idx(j - 1)}) + ω̂({idx(j - 1)})")
    track(OM, "pi", lambda j: f"ω̂({idx(j)})")
    track(CMD, "write", lambda j: f"v({idx(j)}): Vdq at θ̂({idx(j)})")
    track(APP, "load", lambda j: f"v({idx(j)})")
    track(HELD, "hold", lambda j: f"v({idx(j)})")
    track(OBS, "smo", lambda j: f"î, ê ({idx(j)})")
    for name in VNAME:  # the value before the first tracked write
        first = hist[name][0]
        hist[name].insert(0, (t_lo - 10, first[0], "", first[3] - 1))

    fig, (ax_c, ax_l, ax_v, ax_a) = plt.subplots(4, 1, figsize=(15, 13.2), sharex=True, gridspec_kw={"height_ratios": [0.55, 1.35, 3.1, 2.2]})
    _counter(ax_c, t_lo, t_hi, ("top", "bottom", -0.3))
    ax_c.set_title(title, fontsize=11)

    # Execution
    _conversions(ax_l, sim, t_lo, t_hi, label_tick0_only=False)
    _trace(ax_l, sim, t_lo, t_hi, clip=True)
    for j in (-1, 0, 1):
        x = j * 2 * T
        ax_l.axvline(x, color="black", linewidth=0.6, linestyle=":")
        ax_l.text(x + 0.6, 3.55, f"cycle {idx(j)}", fontsize=7)
        ax_l.text(x + T + 0.6, 3.55, "between: marks, arms", fontsize=7, color="gray")
    for j in (-1, 0, 1):
        ax_l.plot(c[j]["sample"], LANES["ADC"] + 0.42, marker="v", color="#1565c0", markersize=6)
        ax_l.text(c[j]["sample"], LANES["ADC"] + 0.62, f"i({idx(j)})", ha="center", fontsize=7, color="#1565c0")
    # A PWM ISR pending behind the one before: cycle k's between ISR, cycle k+1's. Then the arm ahead of the trigger
    y_p = LANES["PWM ISR"] + 0.45
    for top in (T, 2 * T):
        b_start = seg_times(sim, "PWM ISR", "entry", top)[0]
        if b_start - top > 0.5:
            ax_l.plot([top, b_start], [y_p, y_p], color="#c62828", linewidth=2.0)
            ax_l.text((top + b_start) / 2, y_p + 0.08, f"pending {b_start - top:.1f} µs", ha="center", fontsize=7, color="#c62828")
    trigger = 2 * T - sim.seq.lead
    ax_l.annotate("", (trigger, LANES["ADC"] - 0.5), (sim.arm_end[T], LANES["ADC"] - 0.5), arrowprops=dict(arrowstyle="->", color="#00695c"))
    ax_l.text((trigger + sim.arm_end[T]) / 2, LANES["ADC"] - 0.75, f"arm {trigger - sim.arm_end[T]:.1f} µs ahead of the trigger", ha="center", fontsize=7, color="#00695c")
    ax_l.set_yticks(list(LANES.values()))
    ax_l.set_yticklabels(list(LANES.keys()))
    ax_l.set_ylim(0.4, 3.9)

    # Values
    for name in VNAME:
        for a, b, label, j in hist[name]:
            a2, b2 = max(a, t_lo - 2), min(b, t_hi + 2)
            if b2 > a2:
                bus(ax_v, a2, b2, VY[name], label, color[name], j in (-1, 0) if name in (APP, HELD) else j == 0)

    # Cycle k's steps: reads (open), writes (filled), on one vertical
    c0, c1 = c[0], c[1]
    steps = sorted([(c0["capture"], "capture i(k)", [], [I_]),
             (c0["predict"], "predict θ̂(k)", [TH, OM], [TH]),
             (c0["take"], "LOOP: Park i(k) at θ̂(k), PI", [I_, TH], []),
             (c0["write"], "write v(k)", [], [CMD]),
             (c0["correct"] + 0.8, correct_label, [I_, TH, OM, HELD], []),
             (c0["load"], "load v(k)", [CMD], [APP]),
             (c0["smo"], "SMO: ê(k)", [], [OBS]),
             (c0["pi"], "PI: ω̂(k)", [], [OM]),
             (c0["hold"], "hold v(k)", [CMD], [HELD]),
             (c1["predict"], "predict θ̂(k+1)", [TH, OM], [TH])], key=lambda st: st[0])
    rows_end = []  # each label row's last label end, to stack the labels that would overlap
    us_per_char = (t_hi - t_lo) / 205.0
    for n, (x, label, reads, writes) in enumerate(steps):
        text = f"{n + 1} {label}"
        half = len(text) * us_per_char / 2
        row = next((r for r, end in enumerate(rows_end) if end < x - half), None)
        if row is None:
            row = len(rows_end)
            rows_end.append(x + half)
        else:
            rows_end[row] = x + half
        y_text = len(VLANES) - 0.55 + 0.42 * row
        ys = [VY[v] for v in reads + writes]
        ax_v.plot([x, x], [min(ys) - 0.35, y_text], color="#455a64", linewidth=0.7, linestyle="--")
        for v in reads:
            ax_v.plot(x, VY[v], marker="o", markersize=7, markerfacecolor="white", markeredgecolor="black", zorder=5)
        for v in writes:
            ax_v.plot(x, VY[v], marker="o", markersize=7, color="black", zorder=5)
        ax_v.text(x, y_text, text, ha="center", va="bottom", fontsize=7, bbox=dict(boxstyle="round,pad=0.15", facecolor="white", edgecolor="none"))
    ax_v.set_yticks([VY[v] for v in VNAME])
    ax_v.set_yticklabels(VNAME, fontsize=8)
    ax_v.set_ylim(-0.6, len(VLANES) - 0.2 + 0.42 * len(rows_end))
    ax_v.plot([], [], marker="o", linestyle="", markerfacecolor="white", markeredgecolor="black", label="reads")
    ax_v.plot([], [], marker="o", linestyle="", color="black", label="writes")
    ax_v.legend(loc="lower left", fontsize=7, ncol=2)

    # Angles, relative to the rotor, locked: θ̂(j) stands for the rotor at its sample, and the command v(j) is turned by θ̂(j)
    for j in (-2, -1, 0, 1):
        a, b = c[j]["predict"], c[j + 1]["predict"] if j + 1 in c else t_hi
        ax_a.plot([a, b], [rel(c[j]["sample"], a), rel(c[j]["sample"], b)], color="#2e7d32", linewidth=1.2, zorder=4, label="θ̂, published" if j == -2 else None)
        a, b = c[j]["load"], c[j + 1]["load"] if j + 1 in c else t_hi
        ax_a.plot([a, b], [rel(c[j]["sample"], a), rel(c[j]["sample"], b)], color="#ef6c00", linewidth=1.6,
                  label="the applied voltage's frame: θ̂ of the cycle that wrote it" if j == -2 else None)
    for j in (-1, 0, 1):
        ax_a.plot(c[j]["sample"], 0.0, marker="v", color="#1565c0", markersize=7)
    ax_a.axhline(0.0, color="black", linewidth=0.6)
    ax_a.text(t_hi - 1, 0.5, "the rotor, θ", ha="right", fontsize=7)
    ax_a.text(c[1]["sample"] + 2, 0.6, "▼ the instant θ̂(j) stands for: its sample", fontsize=7, color="#1565c0")

    # The period i(k) closes: what was applied across it, against the observer's model of it
    pa = period_angles(c, hold_lead)
    p0, p1, split, mean_applied, mean_modeled = pa["p0"], pa["p1"], pa["split"], pa["mean_applied"], pa["mean_modeled"]
    ax_a.axvspan(p0, p1, color="#e3f2fd", zorder=0)
    ax_a.text((p0 + p1) / 2, 2.2, "the period i(k) closes", ha="center", fontsize=8, color="#1565c0")
    ax_a.plot([p0, p1], [rel(c[-1]["sample"] - hold_lead, p0), rel(c[-1]["sample"] - hold_lead, p1)], color="#ad1457", linewidth=4.0, alpha=0.45, zorder=3,
              label=modeled_label)
    if abs(mean_modeled - mean_applied) > 0.5:
        for y, col, txt in ((mean_applied, "#ef6c00", "applied, mean"), (mean_modeled, "#ad1457", "modeled, mean")):
            ax_a.plot([p0, p1], [y, y], color=col, linewidth=0.8, linestyle=":")
            ax_a.text(p1 - 1, y + 0.35, f"{txt} {y:.1f}°", fontsize=7, color=col, ha="right")
        ax_a.annotate("", (p0 + 3, mean_applied), (p0 + 3, mean_modeled), arrowprops=dict(arrowstyle="<->", color="black"))
        ax_a.text(p0 + 4.5, (mean_applied + mean_modeled) / 2, f"{mean_modeled - mean_applied:.1f}°: {gap_label}", fontsize=7, va="center")
    else:  # the means meet: one line
        ax_a.plot([p0, p1], [mean_applied, mean_applied], color="black", linewidth=0.8, linestyle=":")
        ax_a.text(p1 - 1, mean_applied + 0.35, f"applied and modeled, mean {mean_applied:.1f}°", fontsize=7, ha="right")
        ax_a.text(p1 - 1, mean_applied - 1.0, f"{mean_modeled - mean_applied:.1f}°: {gap_label}", fontsize=7, ha="right")
    ax_a.text(c[-1]["load"], -19.3, f"v(k−2) {split:.0f} µs | v(k−1) {p1 - c[-1]['load']:.0f} µs", ha="center", fontsize=7, color="#ef6c00")
    ax_a.set_ylabel(f"angle from the rotor\n(° at {F_E:.0f} Hz electrical)")
    ax_a.set_ylim(-20.5, 3.2)
    ax_a.legend(loc="lower right", fontsize=7)
    ax_a.set_xlim(t_lo, t_hi)
    ax_a.set_xlabel("Time from the top of control cycle k (µs)")
    for ax in (ax_v, ax_a):
        for j in (-1, 0, 1):
            ax.axvline(j * 2 * T, color="black", linewidth=0.6, linestyle=":")
            ax.axvline(j * 2 * T + T, color="gray", linewidth=0.4, linestyle=":")

    ax_a.text(0.0, -0.2, unicode_text("\n".join(lines)), transform=ax_a.transAxes,
              ha="left", va="top", fontsize=7.5, bbox=dict(boxstyle="round", facecolor="white", edgecolor="#999999"))
    kinds = ["shell", "theta", "req", "capture", "pi", "out", "arm", "obs_frame", "obs_smo", "obs_det", "obs_hold"]
    fig.legend(handles=[Patch(facecolor=KIND_COLOR[k], edgecolor="#555555", label=KIND_LABEL[k] if k != "theta" else "angle: predict, SetTheta") for k in kinds],
               loc="upper right", bbox_to_anchor=(0.995, 0.108), fontsize=7, ncol=2)
    fig.text(0.01, 0.003, "Durations are estimates (k = 1), not measured. The angles assume lock, θ̂(j) = θ at sample j, ω̂ = ω.", fontsize=7, color="gray")
    fig.tight_layout(rect=(0, 0.11, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)
    print(f"cycle k: sample {c0['sample']:.1f}, capture {c0['capture']:.1f}, predict {c0['predict']:.1f}, take {c0['take']:.1f}, write {c0['write']:.1f}, "
          f"load {c0['load']:.1f}, correct {c0['correct']:.1f}-{c0['hold']:.1f} us")
    for line in lines:
        print(line)
    print("wrote", path)


def main():
    sim = SensorlessSim(SEQ_BEFORE, 1.0).run()
    c = cycles(sim)
    c0 = c[0]
    pa = period_angles(c, 0.0)
    p1, split, mean_applied, mean_modeled = pa["p1"], pa["split"], pa["mean_applied"], pa["mean_modeled"]
    corr_us = sum(n for _, _, n in CORRECTION) * US_PER_INSTR
    margins = {k: arm_margin(SensorlessSim(SEQ_BEFORE, k).run(), 0.0) for k in (1.0, 1.2)}
    margins_speed = {k: arm_margin(SensorlessSim(SEQ_BEFORE, k, extra={0.0: SPEED_POLL_US}).run(), 0.0) for k in (1.0, 1.2)}
    base_margin = arm_margin(Sim(SEQ_BEFORE, 1.0).run(), 0.0)
    k_miss = next(k / 100 for k in range(100, 151) if arm_margin(SensorlessSim(SEQ_BEFORE, k / 100).run(), 0.0) < 0)
    # Option, not built: the correction in the between ISR, after its arm
    opt = {k: SensorlessSim(SEQ_BEFORE, k, between=True).run() for k in (1.0, 1.2)}
    opt_lines = [f"k {k:g}: arm {margin_text(arm_margin(o, 0.0))}; correction {seg_times(o, 'PWM ISR', 'frames', T)[0]:.1f} to "
                 f"{seg_times(o, 'PWM ISR', 'hold', T)[1]:.1f} us; cycle k+1's PWM ISR starts {o.pwm_delay.get(2 * T, 0.0):.1f} us late, "
                 f"writes at {seg_times(o, 'PWM ISR', 'out', 2 * T)[1] - 2 * T:.1f} us, still before its next top"
                 for k, o in opt.items()]
    lines = [f"The correction, estimated {corr_us:.1f} us at k 1 ({sum(n for _, _, n in CORRECTION)} instructions of the linked image, not measured): "
             f"{c0['correct']:.1f} to {c0['hold']:.1f} us, past the next top.",
             f"The between ISR waits for it, then arms the next set {margin_text(margins[1.0])} of its trigger ({base_margin:.1f} without the observer); "
             f"from k {k_miss:.2f} it misses, {margin_text(margins[1.2])} at k 1.2. A missed arm: the set waits a control cycle.",
             f"On the 1 ms speed tick, about {SPEED_POLL_US:.0f} us more ahead of it: {margin_text(margins_speed[1.0])} at k 1 ({margin_text(margins_speed[1.2])} at k 1.2).",
             f"The period i(k) closes holds v(k-2) for {split:.0f} us and v(k-1) for {p1 - c[-1]['load']:.0f}: the write loads {c[0]['load'] - c[0]['write']:.0f} us after it, "
             f"{c[0]['load'] - c[0]['sample']:.0f} after the sample.",
             f"The observer models v(k-1) across all of it: its voltage leads the applied one by {mean_modeled - mean_applied:.1f} deg at {F_E:.0f} Hz, "
             f"the sample -> load in rotation.",
             f"The FOC's own command lags the rotor by {-mean_applied:.1f} deg on average over its hold: sample -> load plus half the hold, absorbed by the PI.",
             "Not drawn: the sense chain's age, by which the current read lags the current (the board's filter), on top of the same offset."]
    plot_timeline(sim, os.path.join(HERE, "SensorlessSequence.png"),
                  "Sensorless on the built current-loop sequence: what each step reads and writes, and when the duty is applied\n"
                  "KE06 timing ('before', ../ISample): the 20 kHz PWM, a 100 µs control cycle, the set on the trigger 16.5 µs before the top",
                  0.0, "correct: i(k) at θ̂(k), v(k−1) at θ̂(k) − Δθ̂/2",
                  "the observer's voltage for that period: v(k−1) across all of it, Parked at θ̂(k) − Δθ̂/2",
                  f"the sample → load, {c[0]['load'] - c[0]['sample']:.0f} µs", lines)
    print("Option, not built: the correction in the between ISR, after its arm. Drawn in sensorless_sequence_fixed.py")
    for line in opt_lines:
        print("  " + line)


if __name__ == "__main__":
    main()
