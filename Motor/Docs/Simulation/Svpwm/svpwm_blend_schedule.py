import os
import numpy as np
import matplotlib.pyplot as plt
from svpwm_ceiling import INK, finish
from svpwm_blend import RADIUS_START, RADIUS_END, FW_RADIUS, k_of_magnitude, k_of_spread_ceiling, phase_voltages, MIDCLAMP, CEILING, BLEND
from svpwm_blend_analysis import T_US

"""
Per vector, every placement splits the null time n = 1 - spread between the lowest phase's on time k*n and the top phase's window (1 - k)*n.
A per-vector placement on the spread is a curve against it. The |v| blend is one k per radius: a segment over the cycle's spreads, 1.5*r to sqrt3*r.
"""
EDGE_US = 1.0  # gate driver turn-on, 680 ns typ incl. 520 ns dead time: a shorter pulse may be swallowed
spread = np.linspace(0.70, 1.0, 601)
SCHEDULES = (  # name, k against the spread, color, line style
    ("Midclamp", np.full_like(spread, 0.5), MIDCLAMP, "-"),
    ("Ceiling √3/2: (D − s)/(1 − s), held to [0, ½]", k_of_spread_ceiling(spread), CEILING, "-"),
    ("Bottom clamp", np.zeros_like(spread), INK, ":"),
)
SEGMENT_RADII = (RADIUS_START, FW_RADIUS, 0.52, 0.53, RADIUS_END)
QUANTITIES = (  # title, per (k, n), y label
    ("k against the spread", lambda k, n: k, "k, the share of the null time at O111"),
    ("The top phase's low-side window, (1 − k)·n·T:\nat a line-to-line peak, the 3 phases read window", lambda k, n: (1 - k) * n * T_US, "Window (µs)"),
    ("The lowest phase's on time, k·n·T:\nunder 1 µs the gate driver may swallow the pulse", lambda k, n: k * n * T_US, "On time (µs)"),
)

fig, axes = plt.subplots(1, 3, figsize=(18, 6.5), layout="constrained")
for ax, (title, quantity, ylabel) in zip(axes, QUANTITIES):
    for name, k, color, style in SCHEDULES:
        ax.plot(spread, quantity(k, 1 - spread), color=color, linewidth=2, linestyle=style, label=name)
    for j, r in enumerate(SEGMENT_RADII):
        # the |v| blend at radius r: one k, over the cycle's spreads
        s = np.array([1.5 * r, np.sqrt(3) * r])
        k = k_of_magnitude(*phase_voltages(r, 0.0))
        ax.plot(s, quantity(np.full(2, k), 1 - s), color=BLEND, linewidth=3, solid_capstyle="round", label="|v| blend at r, one k: from a phase's peak (○) to a line-to-line peak (●)" if j == 0 else None)
        ax.plot(s[0], quantity(k, 1 - s[0]), marker="o", markersize=7, color="white", markeredgecolor=BLEND, markeredgewidth=2, zorder=4)
        ax.plot(s[1], quantity(k, 1 - s[1]), marker="o", markersize=7, color=BLEND, markeredgecolor="white", markeredgewidth=1.5, zorder=4)
        if ax is axes[0]:  # the radius, above the segment, where the segments don't cross
            ax.annotate(f"r = {r:.3f}", (s.mean(), k), xytext=(0, 4), textcoords="offset points", ha="center", va="bottom", fontsize=8, color=INK)
    finish(ax, title, "Spread vMax − vMin (per VBus)", ylabel)
    ax.title.set_fontsize(11)
    ax.set_xlim(0.70, 1.0)
axes[2].axhline(EDGE_US, color=INK, linewidth=1, linestyle=":")
axes[2].annotate("1 µs", (0.70, EDGE_US), xytext=(4, 3), textcoords="offset points", fontsize=8, color=INK)
axes[0].set_ylim(-0.02, 0.6)
handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc="outside upper center", ncol=3, fontsize=9)

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Blend_Schedule.png"), dpi=100)
plt.show()
