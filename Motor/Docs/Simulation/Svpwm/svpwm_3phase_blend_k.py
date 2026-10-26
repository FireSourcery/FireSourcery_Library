import os
import numpy as np
import matplotlib.pyplot as plt
from svpwm_3phase_ceiling import finish
from svpwm_3phase_blend import MAGNITUDE, DUTY_MAX, CEILING, k_of_spread_ceiling
from svpwm_3phase_blend_analysis import radii, magnitude, dutyMax, K_MIN, K_MAX, mark_inflections

# k over a cycle: one value for the |v| blend, a range for the per-vector placements. Each is lowest at a line-to-line peak, spread sqrt3*r,
# and highest at a phase's peak, spread 1.5*r
fig, ax = plt.subplots(figsize=(8, 5.5), layout="constrained")
ax.fill_between(radii, k_of_spread_ceiling(np.sqrt(3) * radii), k_of_spread_ceiling(1.5 * radii), color=CEILING, alpha=0.12, linewidth=0)
ax.plot(radii, k_of_spread_ceiling(np.sqrt(3) * radii), color=CEILING, linewidth=1, label="Ceiling √3/2, for reference: k at each line-to-line peak")
ax.plot(radii, k_of_spread_ceiling(1.5 * radii), color=CEILING, linewidth=1, linestyle=(0, (6, 2)), label="Ceiling √3/2: k at each phase's peak")
ax.fill_between(radii, dutyMax[:, K_MIN], dutyMax[:, K_MAX], color=DUTY_MAX, alpha=0.2, linewidth=0)
ax.plot(radii, magnitude[:, K_MIN], color=MAGNITUDE, linewidth=5, label="|v| blend: one k over the whole cycle")  # wider, under: the duty-max blend's lowest k is on it
ax.plot(radii, dutyMax[:, K_MIN], color=DUTY_MAX, linewidth=2, label="Duty-max blend: k at each line-to-line peak, the |v| blend's")
ax.plot(radii, dutyMax[:, K_MAX], color=DUTY_MAX, linewidth=2, linestyle=(0, (6, 2)), label="Duty-max blend: k at each phase's peak, ½ to the linear limit")
mark_inflections(ax, labelY=0.25)
ax.set_xlim(0.36, 0.585)
ax.set_ylim(-0.02, 0.6)
ax.legend(loc="lower left", fontsize=8)
finish(ax, "k over a cycle: the same at each line-to-line peak, apart between", "Voltage vector magnitude r (per VBus)", "k, the share of the null time at O111")

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_3Phase_Blend_K.png"), dpi=100)
plt.show()
