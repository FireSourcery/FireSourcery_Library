import os
import numpy as np
import matplotlib.pyplot as plt
from svpwm_ceiling import finish
from svpwm_blend import BLEND, CEILING, k_of_spread_ceiling
from svpwm_blend_analysis import radii, blend, magnitudeSquared, K_MIN, mark_inflections

# k over a cycle: one value for the |v| blend, a range for the ceiling, lowest at a line-to-line peak, spread sqrt3*r, and highest at a phase's peak, spread 1.5*r
fig, ax = plt.subplots(figsize=(8, 5.5), layout="constrained")
ax.fill_between(radii, k_of_spread_ceiling(np.sqrt(3) * radii), k_of_spread_ceiling(1.5 * radii), color=CEILING, alpha=0.12, linewidth=0)
ax.plot(radii, k_of_spread_ceiling(np.sqrt(3) * radii), color=CEILING, linewidth=1, label="Ceiling √3/2, for reference: k at each line-to-line peak")
ax.plot(radii, k_of_spread_ceiling(1.5 * radii), color=CEILING, linewidth=1, linestyle=(0, (6, 2)), label="Ceiling √3/2: k at each phase's peak")
ax.plot(radii, blend[:, K_MIN], color=BLEND, linewidth=3, label="|v| blend: one k over the whole cycle")
ax.plot(radii, magnitudeSquared[:, K_MIN], color="black", linewidth=1, linestyle=(0, (2, 2)), label="|v| blend on |v|², no sqrt")
mark_inflections(ax, labelY=0.25)
ax.set_xlim(0.36, 0.585)
ax.set_ylim(-0.02, 0.6)
ax.legend(loc="lower left", fontsize=8)
finish(ax, "k over a cycle", "Voltage vector magnitude r (per VBus)", "k, the share of the null time at O111")

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Blend_K.png"), dpi=100)
plt.show()
