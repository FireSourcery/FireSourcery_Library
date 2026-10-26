import os
import numpy as np
import matplotlib.pyplot as plt
from svpwm_ceiling import SPREAD_CEILING, SPREAD_BOTTOM, INFLECTIONS, REGIME_COLORS, INK, finish
from svpwm_ceiling_analysis import radii, ceiling, SPREAD_MIN, SPREAD_MAX

# Why the inflections fall where they do: the spread band crosses the 2 thresholds
fig, ax = plt.subplots(figsize=(8, 5.5), layout="constrained")
ax.fill_between(radii, ceiling[:, SPREAD_MIN], ceiling[:, SPREAD_MAX], color=INK, alpha=0.12, linewidth=0)
ax.plot(radii, ceiling[:, SPREAD_MAX], color=INK, linewidth=1.5)
ax.plot(radii, ceiling[:, SPREAD_MIN], color=INK, linewidth=1.5)
ax.annotate("√3·r, at a line-to-line peak", (0.455, np.sqrt(3) * 0.455), xytext=(-6, 4), textcoords="offset points", ha="right", va="bottom", fontsize=8, color=INK)
ax.annotate("1.5·r, at a phase's own peak", (0.52, 1.5 * 0.52), xytext=(6, -4), textcoords="offset points", ha="left", va="top", fontsize=8, color=INK)
ax.axhline(SPREAD_CEILING, color=REGIME_COLORS[1], linewidth=1.5, linestyle="--")
ax.annotate(f"2D − 1 = {SPREAD_CEILING:.3f}: midclamp's highest duty at D", (0.583, SPREAD_CEILING), xytext=(0, -4), textcoords="offset points",
            ha="right", va="top", fontsize=8)
ax.axhline(SPREAD_BOTTOM, color=REGIME_COLORS[2], linewidth=1.5, linestyle="--")
ax.annotate(f"D = {SPREAD_BOTTOM:.3f}: on the ceiling, the lowest duty at 0", (0.362, SPREAD_BOTTOM), xytext=(0, 4), textcoords="offset points",
            ha="left", va="bottom", fontsize=8)
for r, _, name in INFLECTIONS:  # onsets on the line-to-line peak's edge, throughouts on the phase peak's
    level = SPREAD_CEILING if "ceiling" in name else SPREAD_BOTTOM
    ax.plot([r, r], [0.5, level], color=INK, linewidth=0.8, linestyle=":")
    ax.plot(r, level, marker="o", markersize=7, color="black", markeredgecolor="white", markeredgewidth=1.5, zorder=3)
    ax.annotate(f"{r:.3f}", (r, level), xytext=(-5, 5), textcoords="offset points", ha="right", va="bottom", fontsize=8)
ax.set_xlim(0.36, 0.585)
ax.set_ylim(0.5, 1.02)
finish(ax, "Why there: the spread vMax − vMin over a cycle crosses 2D − 1, then D", "Voltage vector magnitude r (per VBus)", "Spread (per VBus)")

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Ceiling_Spread.png"), dpi=100)
plt.show()
