import os
import matplotlib.pyplot as plt
from svpwm_ceiling import REGIMES, REGIME_COLORS, finish
from svpwm_ceiling_analysis import radii, share, mark_inflections

# Where in the cycle: the share of each branch, exact; the sampled shares match it to the sample (svpwm_ceiling_analysis.py check)
fig, ax = plt.subplots(figsize=(8, 5.5), layout="constrained")
ax.stackplot(radii, share.T, colors=REGIME_COLORS, labels=REGIMES)
mark_inflections(ax, color="white")
for x, y, name in ((0.39, 0.5, "midclamp"), (0.462, 0.82, "ceiling"), (0.556, 0.82, "bottom clamp")):
    ax.text(x, y, name, ha="center", va="center", fontsize=10, color="white", fontweight="bold")
ax.set_xlim(0.36, 0.585)
ax.set_ylim(0, 1)
finish(ax, "Where in the cycle: the share of the cycle on each branch", "Voltage vector magnitude r (per VBus)", "Share of the cycle")

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Ceiling_Regime.png"), dpi=100)
plt.show()
