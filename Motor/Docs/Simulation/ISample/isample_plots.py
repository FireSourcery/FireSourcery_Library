"""The figures: one timeline per sequence, sequences compared on one axis, the start calibration, and the k sweep summary."""
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from isample_model import AGE_US, HOLD_HALF_US, LOAD, SAMPLE_OFFSET, T, Sim, at_start, walk_conv  # noqa: E402

KIND_COLOR = {"shell": "#d9d9d9", "theta": "#81c784", "req": "#b39ddb", "capture": "#64b5f6", "pi": "#e57373", "out": "#ffb74d", "spin": "#f5f5f5",
              "walk": "#a1887f", "arm": "#4db6ac"}
KIND_LABEL = {"shell": "entry, dispatch, exit", "theta": "angle: sensor, SetTheta", "req": "requests: ramp, field weakening",
              "capture": "VSOURCE, I capture, Park", "pi": "PI", "out": "inverse Park, SVPWM, write", "spin": "spin, waiting for the ADC",
              "walk": "the other channels' completion: main-loop channels", "arm": "marks, arm the next trigger's set"}
WALK_COLOR = "#d7ccc8"
LANES = {"ADC": 3.0, "ADC ISR": 2.0, "PWM ISR": 1.0}


def conv_color(name):
    return "#cfd8dc" if name == "VSOURCE" else "#90a4ae"


def cycle_label(seq, n):
    """The cycle at tick n: the one scheduled to take batch n's set, or the off cycle between"""
    if n % seq.divider != 0:
        return "between: marks, arms" if seq.between == "arm" else "off cycle"
    return "takes the set" if seq.trigger == "hw" else "starts the set"


def _counter(ax, t_lo, t_hi, labels):
    ts = [t_lo + i * 0.1 for i in range(int((t_hi - t_lo) / 0.1) + 1)]
    ax.plot(ts, [1 - abs(2 * (((t + 25) % 50) / 50) - 1) for t in ts], color="black", linewidth=1)
    for n in range(int(t_lo // LOAD), int(t_hi // LOAD) + 1):
        lp = n * LOAD
        if t_lo <= lp <= t_hi:
            top = (lp % T == 0)
            ax.plot(lp, 1.0 if top else 0.0, marker="v" if top else "^", color="gray", markersize=6)
            if lp > t_lo:
                ax.text(lp, 1.12 if top else labels[2], labels[0] if top else labels[1], ha="center", fontsize=7, color="gray")
    ax.set_ylim(-0.45, 1.3)
    ax.set_yticks([])
    ax.set_ylabel("FTM2\ncounter")


def _conversions(ax, sim, t_lo, t_hi, label_tick0_only=True):
    for name, start, width in sim.conv:
        for tick, b in sim.batches.items():
            x = b["start"] + start
            if t_lo <= x <= t_hi:
                ax.broken_barh([(x, width)], (LANES["ADC"] - 0.3, 0.6), facecolors=conv_color(name), edgecolor="white")
                if tick == 0 or not label_tick0_only:
                    ax.text(x + width / 2, LANES["ADC"], {"VSOURCE": "VS"}.get(name, name), ha="center", va="center", fontsize=6)
                if name == "IB" and label_tick0_only:
                    ax.plot(x, LANES["ADC"] + 0.42, marker="v", color="black", markersize=5)
    for x, n, name in sim.walks:
        ax.broken_barh([(x, walk_conv(n))], (LANES["ADC"] - 0.3, 0.6), facecolors=WALK_COLOR, edgecolor="white")
        ax.text(x + walk_conv(n) / 2, LANES["ADC"], f"{name} {n}", ha="center", va="center", fontsize=6)


def _trace(ax, sim, t_lo, t_hi, clip=False):
    for lane, label, kind, a, b, tag in sim.trace:
        if b < t_lo or a > t_hi:
            continue
        a2, b2 = (max(a, t_lo), min(b, t_hi)) if clip else (a, b)
        ax.broken_barh([(a2, b2 - a2)], (LANES[lane] - 0.3, 0.6), facecolors=KIND_COLOR[kind], edgecolor="#555555", linewidth=0.4,
                       hatch="///" if kind == "spin" else None)
        if b2 - a2 >= 2.4:
            ax.text((a2 + b2) / 2, LANES[lane], label, ha="center", va="center", fontsize=6)


def _legend(fig, loc, anchor, ncol):
    fig.legend(handles=[Patch(facecolor=c, edgecolor="#555555", hatch="///" if k == "spin" else None, label=KIND_LABEL[k]) for k, c in KIND_COLOR.items()],
               loc=loc, bbox_to_anchor=anchor, fontsize=7, ncol=ncol)


def plot_sequence(seq, sim, path, k=1.0):
    r = sim.result()
    t_lo, t_hi = -30.0, 130.0
    fig, (ax_c, ax_l) = plt.subplots(2, 1, figsize=(13, 6.8), sharex=True, gridspec_kw={"height_ratios": [1, 2.2]})
    _counter(ax_c, t_lo, t_hi, ("TOF", "bottom", -0.3))
    if not math.isnan(r["load"]):
        ax_c.plot(r["load"], 1.0 if r["load"] % T == 0 else 0.0, marker="*", color="green", markersize=16)
        ax_c.annotate("batch 0's PI output loads", (r["load"], 0.5), (r["load"] + 2, 0.55), fontsize=8, color="green")
    ax_c.set_title(f"{seq.key}. {seq.title}", fontsize=11)

    _conversions(ax_l, sim, t_lo, t_hi)
    _trace(ax_l, sim, t_lo, t_hi)
    for n in range(int(t_lo // T), int(t_hi // T) + 1):
        if t_lo <= n * T <= t_hi:
            ax_l.axvline(n * T, color="black", linewidth=0.6, linestyle=":")
            ax_l.axvline(n * T + LOAD, color="gray", linewidth=0.4, linestyle=":")
            ax_l.text(n * T + 0.5, 3.55, cycle_label(seq, n), fontsize=7)

    # Batch 0: sample -> Park -> PI -> load
    y = 0.15
    if not math.isnan(r["load"]):
        ax_l.annotate("", (r["load"], y), (r["sample"], y), arrowprops=dict(arrowstyle="->", color="green", linewidth=1.5))
        ax_l.text((r["sample"] + r["load"]) / 2, y - 0.25, f"batch 0: sample -> load {r['sample_to_load']:.1f} us", ha="center", fontsize=8, color="green")
    if not math.isnan(r["park_at"]):
        y_park = LANES[r["park_lane"]] + 0.38
        ax_l.plot(r["park_at"], y_park, marker="D", color="#1565c0", markersize=5)
        ax_l.text(r["park_at"] + 0.8, y_park + 0.05, f"batch 0 Parked at the angle of t = {r['park_tick']:.0f}", fontsize=7, color="#1565c0")
    ax_l.set_yticks(list(LANES.values()))
    ax_l.set_yticklabels(list(LANES.keys()))
    ax_l.set_ylim(-0.3, 3.8)
    ax_l.set_xlim(t_lo, t_hi)
    ax_l.set_xlabel("Time from the counter top of the cycle scheduled to take batch 0 (us)")

    total = AGE_US + r["sample_to_load"] + HOLD_HALF_US
    lines = [f"sample -> load {r['sample_to_load']:.1f} us; delay to actuation {total:.0f} us at hold tau 10 us",
             f"Park angle from t = {r['park_tick']:.0f}: {r['park_offset']:+.1f} us from the sample",
             f"CPU in batch 0's cycle {r['busy_analog']:.1f} of 50 us, the next {r['busy_off']:.1f}; PWM ISR start delay up to {r['pwm_delay']:.1f} us"] + seq.notes
    ax_l.text(0.995, -0.3, "\n".join(lines), transform=ax_l.transAxes, ha="right", va="top", fontsize=7.5,
              bbox=dict(boxstyle="round", facecolor="white", edgecolor="#999999"))
    _legend(fig, "lower left", (0.01, 0.035), 2)
    fig.text(0.01, 0.005, f"Durations are estimates (k = {k:g}), not measured. See PwmSampleSequenceSummary.png for the sweep.", fontsize=7, color="gray")
    fig.tight_layout(rect=(0, 0.2, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)


def plot_order_compare(pairs, path, title="Triggered, the set timed ahead of the top: the LOOP's order", t_hi=60.0):
    """Sequences on one time axis around the take's top, one panel each. A row is (label, seq) at k = 1, or (label, seq, k)."""
    t_lo = -25.0
    fig, axes = plt.subplots(len(pairs) + 1, 1, figsize=(12 if t_hi <= 60.0 else 15, 1.6 + 2.6 * len(pairs)), sharex=True,
                             gridspec_kw={"height_ratios": [0.7] + [2.0] * len(pairs)})
    _counter(axes[0], t_lo, t_hi, ("top: loads", "bottom: loads", -0.32))
    axes[0].set_title(title, fontsize=11)
    for ax, row in zip(axes[1:], pairs):
        label, seq, k = (tuple(row) + (1.0,))[:3]
        sim = Sim(seq, k).run()
        r = sim.result()
        _conversions(ax, sim, t_lo, t_hi)
        _trace(ax, sim, t_lo, t_hi, clip=True)
        for n in range(-1, int(t_hi // T) + 1):
            ax.axvline(n * T, color="black", linewidth=0.6, linestyle=":")
            ax.axvline(n * T + LOAD, color="gray", linewidth=0.4, linestyle=":")
            if t_hi > 60.0 and 0 <= n * T < t_hi:
                ax.text(n * T + 0.5, 3.42, cycle_label(seq, n), fontsize=7, color="gray")
        ax.plot([r["write"]] * 2, [LANES["PWM ISR"] - 0.45, LANES["PWM ISR"] + 0.45], color="black", linewidth=1.5)
        ax.text(r["write"] + 0.6, LANES["PWM ISR"] + 0.42, f"write {r['write']:.1f} us", fontsize=7)
        ax.annotate("", (r["load"], 0.25), (r["sample"], 0.25), arrowprops=dict(arrowstyle="->", color="green", linewidth=1.5))
        ax.plot(r["load"], 0.25, marker="*", color="green", markersize=13)
        ax.text((r["sample"] + r["load"]) / 2, -0.05, f"sample -> load {r['sample_to_load']:.1f} us: loads at the {'bottom' if r['load'] % T else 'next top'}",
                ha="center", fontsize=8, color="green")
        ax.set_yticks(list(LANES.values()))
        ax.set_yticklabels(list(LANES.keys()))
        ax.set_ylim(-0.3, 3.6)
        ax.set_title(label, fontsize=9, loc="left")
    axes[-1].set_xlim(t_lo, t_hi)
    axes[-1].set_xlabel("Time from the top whose LOOP takes the set (us)")
    _legend(fig, "lower center", (0.5, 0.0), 4)
    fig.text(0.01, 0.005, "Durations are estimates, not measured. k is a factor on all of them, 1 the estimate.", fontsize=7, color="gray")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)


def plot_start_calibration(path, cases, built_lead):
    """The wait the take would need, and sample -> load without it, by the set's start. [cases]: (name, seq, color, calibrated lead, its text).
    Each variant's latest start for its load point marked; [built_lead] the start as built."""
    starts = [-25.0 + i for i in range(41)]  # from the init trigger at the bottom, delay 0, to 15 us after the top
    k_styles = ((0.8, ":"), (1.0, "-"), (1.2, "--"))
    fig, (ax_w, ax_l) = plt.subplots(2, 1, figsize=(11, 8.5), sharex=True)
    for i, (name, seq, color, cal, cal_text) in enumerate(cases):
        for k, ls in k_styles:
            results = [at_start(seq, k, -x) for x in starts]
            ax_w.plot(starts, [r["wait"] for r in results], color=color, linestyle=ls, label=f"{name}, k {k}")
            ax_l.step(starts, [r["sample_to_load"] for r in results], where="mid", color=color, linestyle=ls, label=f"{name}, k {k}")
        for ax in (ax_w, ax_l):
            ax.axvline(-cal, color=color, linewidth=1.2, alpha=0.6)
        ax_w.text(-cal + 0.3, 1.0 + i, f"{cal_text} at k 1: {-cal:+.1f} us", color=color, fontsize=8)
    for ax in (ax_w, ax_l):
        ax.axvline(0.0, color="black", linewidth=0.6, linestyle=":")
        ax.axvline(-built_lead, color="gray", linewidth=0.8, linestyle="-.")
        ax.axvspan(-25.0, -25.0 + 10.2, color="#e8f5e9", zorder=0)   # SIM_SOPT0[DELAY], 255 ticks at the 25 MHz bus clock
        ax.axvspan(-25.0 + 10.2, -25.0 + 20.4, color="#fff8e1", zorder=0)  # BUSREF / 2
        ax.axvspan(-25.0 + 20.4, 15.0, color="#fbe9e7", zorder=0)    # BUSREF / 4 and beyond
        ax.grid(alpha=0.4)
    ax_w.text(-24.5, ax_w.get_ylim()[1] * 0.9, "the delay at the bus clock", fontsize=7, color="green")
    ax_w.text(-14.3, ax_w.get_ylim()[1] * 0.9, "BUSREF / 2", fontsize=7, color="#b8860b")
    ax_w.text(-4.1, ax_w.get_ylim()[1] * 0.9, "BUSREF / 4", fontsize=7, color="#bf360c")
    ax_w.text(-built_lead + 0.3, ax_w.get_ylim()[1] * 0.75, f"built, {-built_lead:.1f} us", fontsize=7, color="gray")
    ax_w.set_ylabel("Wait the take would need (us)\nthe -w variations spin it")
    ax_w.set_title("Triggered: the start against each variant's load point", fontsize=11)
    ax_l.set_ylabel("Sample -> load without the wait (us)\na late set is taken on the next cycle")
    ax_l.set_xlabel("The set's start, from the top whose LOOP takes it (us). The init trigger is at the bottom, -25")
    for load, label in ((25, "the bottom"), (50, "the next top"), (75, "the next bottom"), (100, "the top after")):
        ax_l.axhline(load - SAMPLE_OFFSET, color="#cccccc", linewidth=0.5)
    ax_l.legend(fontsize=7, ncol=2, loc="upper left")
    fig.text(0.01, 0.005, "Durations are estimates, not measured. k is a factor on all of them. The delay counts at the bus clock (measured 2026-11-03).",
             fontsize=7, color="gray")
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)


def plot_summary(seqs, ks, sweep, styles, path):
    """Sample -> load, CPU and the Park angle's offset against the speed factor k, every sequence in [seqs]"""
    fig, (ax_a, ax_b, ax_c) = plt.subplots(3, 1, figsize=(11, 13.5), sharex=True)
    for s in seqs:
        color, ls = styles[s.key]
        label = f"{s.key}. {s.title}"
        ax_a.step(ks, [r["sample_to_load"] for r in sweep[s.key]], where="mid", color=color, linestyle=ls, label=label)
        ax_b.plot(ks, [r["busy_analog"] for r in sweep[s.key]], color=color, linestyle=ls)
        ax_c.plot(ks, [r["park_offset"] for r in sweep[s.key]], color=color, linestyle=ls)
    ax_a.axvline(1.0, color="gray", linewidth=0.8)
    for load, name in ((25, "same period's bottom"), (50, "next top"), (75, "next bottom"), (100, "top after")):
        ax_a.text(1.605, load - SAMPLE_OFFSET, f"loads at the {name}", fontsize=7, va="center", color="gray")
    ax_a.set_ylabel("Sample -> load (us)")
    ax_a.set_title("Current-loop sequences on KE, by a factor on every ISR duration (1 = the estimate)")
    ax_a.grid()
    fig.legend(*ax_a.get_legend_handles_labels(), loc="lower center", ncol=1, fontsize=8, bbox_to_anchor=(0.5, 0.0))
    ax_b.axhline(T, color="red", linewidth=0.8)
    ax_b.axvline(1.0, color="gray", linewidth=0.8)
    ax_b.text(0.41, T + 0.5, "the period: past it the next PWM ISR starts late", fontsize=7, color="red")
    ax_b.set_ylabel("CPU in batch 0's cycle (us)")
    ax_b.grid()
    ax_c.axvline(1.0, color="gray", linewidth=0.8)
    ax_c.set_ylabel("Park angle time - sample time (us)")
    ax_c.set_xlabel("Speed factor k on the estimated durations (lower is faster)")
    ax_c.text(0.41, -48, "-50: the previous tick's angle, Parked before this tick's SetTheta", fontsize=7)
    ax_c.grid()
    fig.tight_layout(rect=(0, 0.17, 0.86, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)
