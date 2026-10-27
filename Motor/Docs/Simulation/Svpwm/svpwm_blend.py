import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from svpwm_ceiling import DUTY_CEILING, phase_voltages, REGIME_COLORS, finish

"""
The |v| blend, from midclamp to the bottom clamp: svpwm_clamp fed k from the magnitude |v|, 1/2 up to a start, falling linearly to 0 at an end.
One k for the whole cycle. The ends are two radii of one low-side window w, today's 3.35 us of 50 us: midclamp's, VBus/2, and the bottom clamp's,
(1 - w)/sqrt3 = 0.539, the radius SVPWM_CLAMP_RADIUS_OF_WINDOW selects.
Without the sqrt: |v|^2 = 2/3 * sum v^2 against the squared ends, k_of_magnitude_squared.
"""
DUTY_PEAK = 1 - 3.35 / 50.0             # the highest duty that leaves today's low-side window, 1 - w: 93.3 %
RADIUS_START = 0.5                      # VBus/2: midclamp's radius for that window, 1/2 + (sqrt3/2)*r = DUTY_PEAK
RADIUS_END = DUTY_PEAK / np.sqrt(3)     # 0.539: the bottom clamp's, sqrt3*r = DUTY_PEAK, SVPWM_CLAMP_RADIUS_OF_WINDOW(3.35F, 50.0F, 0.0F)

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

def k_of_magnitude_squared(vA, vB, vC):
    """ The |v| blend without the sqrt: |v|^2 against the squared ends. Linear in |v|^2, within 0.005 of the |v| blend over these ends """
    return blend_k(2.0 / 3.0 * (vA * vA + vB * vB + vC * vC), RADIUS_START ** 2, RADIUS_END ** 2)

"""
Per vector, against the spread s = vMax - vMin, for reference: the ceiling, svpwm_midclamp_ceiling, (D - s)/(1 - s) held to [0, 1/2].
The blend is not a function of the spread alone; it depends on the radius.
"""
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

INFLECTIONS = (  # r, formula, what: each over the whole cycle at once
    (RADIUS_START, "VBus/2", "ramp starts"),
    (RADIUS_END, "(1 - w)/sqrt3", "bottom clamp"),
)

REGIMES = ("Midclamp", "Ramp", "Bottom clamp")
K_COLORS = LinearSegmentedColormap.from_list("k", REGIME_COLORS[::-1])  # the ceiling's hue, light to dark: k from 1/2 to 0
MIDCLAMP, CEILING, BLEND = "#eb6834", "#2a78d6", "#1baf7a"  # as low_side_zero_sequence_blend.py
FW_RADIUS = 0.95 * RADIUS_END  # field weakening's |v| on a RADIUS_END circle, FOC_FIELD_WEAKENING_V_MARGIN
LINEAR_LIMIT = 1 / np.sqrt(3)

CASES = (0.45, RADIUS_START, FW_RADIUS, 0.525, RADIUS_END, LINEAR_LIMIT)  # radius per VBus, the same in every figure

def plot_text(text):
    """ A name or formula for a figure """
    return text.replace("sqrt3", "√3")

def describe(r, run):
    """ The panel's heading: the radius, then k over the cycle """
    k = run[0, K]
    radius = {
        RADIUS_START: "VBus/2",
        RADIUS_END: f"(1 − w)/√3 = {r:.3f}·VBus, the bottom clamp's radius for the window",
        FW_RADIUS: f"{r:.3f}·VBus, field weakening's |v| on that circle",
        LINEAR_LIMIT: "VBus/√3, the linear limit",
    }.get(r, f"{r:.3f}·VBus")
    if k >= 0.5 - 1e-9:
        what = "midclamp, k = ½"
    elif k <= 1e-9:
        what = "the bottom clamp, k = 0: vZero = vMin"
    else:
        what = f"k = {k:.2f}"
    return f"|v| = {radius}\n{what} over the whole cycle"

def plot_cycles():
    time = np.linspace(0, 1, 1000)  # one electrical cycle
    edges = np.linspace(0, 1, len(time) + 1) - 0.5 / len(time)  # the strip's cell edges, a cell centred on each sample
    fig = plt.figure(figsize=(16, 15.5), layout="constrained")
    axd = fig.subplot_mosaic([["c0", "c1"], ["c2", "c3"], ["c4", "c5"]])
    for key, r in zip(("c0", "c1", "c2", "c3", "c4", "c5"), CASES):
        ax = axd[key]
        run = simulate(r, k_of_magnitude, time)
        ax.plot(time, run[:, 6], label="Duty A", color="red")
        ax.plot(time, run[:, 7], label="Duty B", color="green")
        ax.plot(time, run[:, 8], label="Duty C", color="blue")
        ax.plot(time, run[:, 3], label="Mag A", color="red", alpha=.1)
        ax.plot(time, run[:, 4], label="Mag B", color="green", alpha=.1)
        ax.plot(time, run[:, 5], label="Mag C", color="blue", alpha=.1)
        ax.plot(time, run[:, VZERO], label="vZero", color="black")
        ax.plot(time, run[:, VZERO_MIDCLAMP], label="vZero, midclamp", color="black", alpha=.4, linestyle="--")
        strip = ax.pcolormesh(edges, [1.04, 1.09], run[None, :, K], cmap=K_COLORS, vmin=0, vmax=0.5, shading="flat")
        finish(ax, describe(r, run), "Time (normalized to one electrical cycle)", "Duty Cycle")
        ax.title.set_fontsize(10)
        ax.set_xlim(0, 1)
        ax.set_ylim(-0.7, 1.1)

    handles, labels = axd["c0"].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=6, fontsize=9)
    fig.colorbar(strip, ax=list(axd.values()), location="bottom", shrink=0.4, aspect=40,
                 label="|v| blend. Strip: k, the share of the null time at O111. ½ midclamp, 0 the bottom clamp")
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Blend_Magnitude.png"), dpi=100, bbox_inches="tight")

if __name__ == "__main__":
    time = np.linspace(0, 1, 1000)
    print("|v| blend, over one cycle at each radius:")
    for r in CASES:
        run = simulate(r, k_of_magnitude, time)
        duties = np.sort(run[:, DUTY], axis=1)
        shift = run[:, VZERO] - run[:, VZERO_MIDCLAMP]
        print(f"  r {r:.4f} VBus: k {run[0, K]:.3f}; highest duty up to {duties[:, 2].max():.3f}, middle up to {duties[:, 1].max():.3f},"
              f" lowest down to {duties[:, 0].min():.3f}; vZero above midclamp's by {shift.min():.3f} to {shift.max():.3f}")

    plot_cycles()
    plt.show()
