import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from svpwm_3phase_ceiling import DUTY_CEILING, phase_voltages, REGIME_COLORS, finish

"""
Two blends from midclamp to the bottom clamp, both svpwm_clamp fed a k: 1/2 up to a start, falling linearly to 0 at an end.
    |v| blend       one k for the whole cycle, from the magnitude |v|: midclamp up to VBus/2, the bottom clamp at VBus/sqrt3
    duty-max blend  k per vector, from midclamp's highest duty 1/2 + spread/2, which svpwm already has from vMax and vMin
Midclamp's highest duty over a cycle is 1/2 + (sqrt3/2)*|v|, at each line-to-line peak, so the |v| blend's ends are a duty max
of 93.3 % and 100 %. With those ends the two give the same k at each line-to-line peak, and differ between peaks.
"""
RADIUS_START = 0.5                                  # |v| blend: midclamp up to VBus/2, sqrt3/2 of the linear limit
RADIUS_END = 1 / np.sqrt(3)                         # the bottom clamp at the linear limit
DUTY_START = 0.5 + np.sqrt(3) / 2 * RADIUS_START    # duty-max blend: the same ends on the duty, 93.3 %, the window that works today
DUTY_END = 0.5 + np.sqrt(3) / 2 * RADIUS_END        # 100 %

def blend_k(x, start, end):
    """ The share of the null time at O111: 1/2, midclamp, up to start, falling linearly to 0, the bottom clamp, at end """
    return 0.5 * np.clip((end - x) / (end - start), 0.0, 1.0)

def svpwm_clamp(vA, vB, vC, k):
    """
    Simulate svpwm_clamp (Motor/Motor/Math/svpwm_math.h): the general clamp vZero = k*(vMax - 1) + (1 - k)*vMin.
    k = 1/2 is midclamp, k = 0 the bottom clamp. The line-to-line voltages are unchanged for any k.
    """
    vMax = max(vA, vB, vC)
    vMin = min(vA, vB, vC)
    vZero = k * (vMax - 1) + (1 - k) * vMin
    return vZero, vA - vZero, vB - vZero, vC - vZero

def k_of_magnitude(vA, vB, vC):
    """ The |v| blend's k: |v| = sqrt(2/3 * sum v^2) of the zero-sum set, the same at every vector of a cycle """
    return blend_k(np.sqrt(2.0 / 3.0 * (vA * vA + vB * vB + vC * vC)), RADIUS_START, RADIUS_END)

def k_of_duty_max(vA, vB, vC):
    """ The duty-max blend's k: midclamp's highest duty at this vector, 1/2 + (vMax - vMin)/2 """
    return blend_k(0.5 + (max(vA, vB, vC) - min(vA, vB, vC)) / 2.0, DUTY_START, DUTY_END)

BLENDS = {"|v| blend": k_of_magnitude, "duty-max blend": k_of_duty_max}

"""
Per vector, against the spread s = vMax - vMin, for reference. Midclamp's highest duty is 1/2 + s/2.
    duty-max blend                      blend_k(1/2 + s/2, 93.3 %, 100 %)
    duty-max blend on the ceiling's ends  blend_k(1/2 + s/2, D, 93.3 %): spread 2D - 1 to D, linear in s
    the ceiling, svpwm_midclamp_ceiling  (D - s)/(1 - s), held to [0, 1/2]: concave in s, over the same ends
"""
def k_of_spread_duty_max(spread, dutyStart=DUTY_START, dutyEnd=DUTY_END):
    return blend_k(0.5 + spread / 2, dutyStart, dutyEnd)

def k_of_spread_ceiling(spread):
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.clip(np.nan_to_num((DUTY_CEILING - spread) / (1 - spread), nan=0.0, neginf=0.0), 0.0, 0.5)

VZERO, VZERO_MIDCLAMP, K = 0, 1, 2
PHASE_V, DUTY = slice(3, 6), slice(6, 9)

def simulate(radius, k_of, time):
    """ One electrical cycle at radius per VBus under svpwm_clamp fed k_of. Columns: vZero, vZeroMidclamp, k, vA, vB, vC, dutyA, dutyB, dutyC """
    rows = []
    for a, b, c in zip(*phase_voltages(radius, 2 * np.pi * time)):
        k = k_of(a, b, c)
        vZero, dutyA, dutyB, dutyC = svpwm_clamp(a, b, c, k)
        rows.append((vZero, (max(a, b, c) + min(a, b, c)) / 2.0 - .5, k, a, b, c, dutyA, dutyB, dutyC))
    return np.array(rows)

def regime_of(run):
    """ 0 midclamp, k = 1/2; 1 the ramp between; 2 the bottom clamp, k = 0 """
    return np.select([run[:, K] <= 1e-9, run[:, K] < 0.5 - 1e-9], [2, 1], default=0)

"""
The inflections. Over one cycle at radius r, the spread vMax - vMin runs from 1.5*r, at a phase's own peak, to sqrt3*r, at a line-to-line peak.
    |v| blend       at RADIUS_START the ramp starts, and at RADIUS_END the bottom clamp, each over the whole cycle at once
    duty-max blend  the ramp runs over spreads 2*DUTY_START - 1 = sqrt3/2 to 2*DUTY_END - 1 = 1. It starts at each line-to-line peak
                    at (sqrt3/2)/sqrt3 = VBus/2, and covers the whole cycle at (sqrt3/2)/1.5 = VBus/sqrt3, where k reaches 0 at each
                    line-to-line peak. The bottom clamp over the whole cycle would be at 1/1.5, past the linear limit.
"""
SPREAD_START = 2 * DUTY_START - 1
SPREAD_END = 2 * DUTY_END - 1
INFLECTIONS = {
    "|v| blend": (
        (RADIUS_START, "VBus/2", "ramp starts, over the whole cycle"),
        (RADIUS_END, "VBus/sqrt3", "bottom clamp, over the whole cycle"),
    ),
    "duty-max blend": (
        (SPREAD_START / np.sqrt(3), "(sqrt3/2)/sqrt3", "ramp starts, at each line-to-line peak"),
        (SPREAD_START / 1.5, "(sqrt3/2)/1.5", "ramp over the whole cycle, k = 0 at each line-to-line peak"),
    ),
}

REGIMES = ("Midclamp", "Ramp", "Bottom clamp")
K_COLORS = LinearSegmentedColormap.from_list("k", REGIME_COLORS[::-1])  # the ceiling's hue, light to dark: k from 1/2 to 0
MIDCLAMP, CEILING, MAGNITUDE, DUTY_MAX, DUTY_MAX_CEILING_ENDS = "#eb6834", "#2a78d6", "#1baf7a", "#eda100", "#e87ba4"  # as low_side_zero_sequence_blend.py
FW_RADIUS = 0.95 / np.sqrt(3)  # field weakening's |v| on a VBus/sqrt3 circle, FOC_FIELD_WEAKENING_V_MARGIN

CASES = (0.45, RADIUS_START, 0.52, FW_RADIUS, 0.56, RADIUS_END)  # radius per VBus, the same in both figures

def describe(name, r, run):
    """ The panel's heading: the radius, then what k does over the cycle """
    k, regime = run[:, K], regime_of(run)
    radius = {RADIUS_START: "VBus/2", RADIUS_END: "VBus/√3, the linear limit", FW_RADIUS: f"{r:.3f}·VBus, field weakening's |v| on a VBus/√3 circle"}.get(r, f"{r:.2f}·VBus")
    if k.min() >= 0.5 - 1e-9:
        what = "midclamp over the whole cycle, k = ½"
    elif name == "|v| blend":
        what = f"k = {k[0]:.2f} over the whole cycle" + (": the bottom clamp, vZero = vMin" if k[0] <= 1e-9 else "")
    else:
        what = f"k from {k.min():.2f} at each line-to-line peak to {k.max():.2f}; midclamp over {np.mean(regime == 0):.0%} of the cycle"
    return f"|v| = {radius}\n{what}"

def plot_cycles(name, filename):
    time = np.linspace(0, 1, 1000)  # one electrical cycle
    edges = np.linspace(0, 1, len(time) + 1) - 0.5 / len(time)  # the strip's cell edges, a cell centred on each sample
    fig = plt.figure(figsize=(16, 15.5), layout="constrained")
    axd = fig.subplot_mosaic([["c0", "c1"], ["c2", "c3"], ["c4", "c5"]])
    for key, r in zip(("c0", "c1", "c2", "c3", "c4", "c5"), CASES):
        ax = axd[key]
        run = simulate(r, BLENDS[name], time)
        ax.plot(time, run[:, 6], label="Duty A", color="red")
        ax.plot(time, run[:, 7], label="Duty B", color="green")
        ax.plot(time, run[:, 8], label="Duty C", color="blue")
        ax.plot(time, run[:, 3], label="Mag A", color="red", alpha=.1)
        ax.plot(time, run[:, 4], label="Mag B", color="green", alpha=.1)
        ax.plot(time, run[:, 5], label="Mag C", color="blue", alpha=.1)
        ax.plot(time, run[:, VZERO], label="vZero", color="black")
        ax.plot(time, run[:, VZERO_MIDCLAMP], label="vZero, midclamp", color="black", alpha=.4, linestyle="--")
        if name == "duty-max blend":  # the blend's input, and where it starts
            ax.plot(time, 0.5 + np.ptp(run[:, PHASE_V], axis=1) / 2, label="Midclamp's highest duty, the input", color="gray", linewidth=1, linestyle=(0, (6, 2)))
            ax.axhline(DUTY_START, label=f"Ramp start {DUTY_START:.1%}", color="gray", linestyle=":")
        strip = ax.pcolormesh(edges, [1.04, 1.09], run[None, :, K], cmap=K_COLORS, vmin=0, vmax=0.5, shading="flat")
        finish(ax, describe(name, r, run), "Time (normalized to one electrical cycle)", "Duty Cycle")
        ax.title.set_fontsize(10)
        ax.set_xlim(0, 1)
        ax.set_ylim(-0.7, 1.1)

    handles, labels = axd["c0"].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=6, fontsize=9)
    fig.colorbar(strip, ax=list(axd.values()), location="bottom", shrink=0.4, aspect=40,
                 label="Strip: k, the share of the null time at O111. ½ midclamp, 0 the bottom clamp")
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), filename), dpi=100, bbox_inches="tight")

if __name__ == "__main__":
    time = np.linspace(0, 1, 1000)
    for name, k_of in BLENDS.items():
        print(f"{name}, over one cycle at each radius:")
        for r in CASES:
            run = simulate(r, k_of, time)
            duties = np.sort(run[:, DUTY], axis=1)
            shift = run[:, VZERO] - run[:, VZERO_MIDCLAMP]
            share = np.bincount(regime_of(run), minlength=3) / len(run)
            print(f"  r {r:.4f} VBus: k {run[:, K].min():.3f} to {run[:, K].max():.3f}; highest duty up to {duties[:, 2].max():.3f}, middle up to {duties[:, 1].max():.3f},"
                  f" lowest down to {duties[:, 0].min():.3f}; vZero above midclamp's by {shift.min():.3f} to {shift.max():.3f};"
                  f" cycle on midclamp {share[0]:.0%}, ramp {share[1]:.0%}, bottom clamp {share[2]:.0%}")

    plot_cycles("|v| blend", "SVPWM_3Phase_Blend_Magnitude.png")
    plot_cycles("duty-max blend", "SVPWM_3Phase_Blend_DutyMax.png")
    plt.show()
