import os
import matplotlib.pyplot as plt
from svpwm_ceiling import finish
from svpwm_blend import BLEND, DUTY_PEAK
from svpwm_blend_analysis import radii, blend, midclamp, HIGHEST_MIN, HIGHEST_MAX, MIDDLE_MAX, LOWEST_MIN, LOWEST_MAX, T_US, mark_inflections

# What the blend does to the duties: each duty's range over a cycle, midclamp's peaks for reference
fig, ax = plt.subplots(figsize=(8, 6.5), layout="constrained")
ax.plot(radii, midclamp[:, HIGHEST_MAX], color="gray", linewidth=1, label="Midclamp: highest duty's peak")
ax.plot(radii, midclamp[:, MIDDLE_MAX], color="gray", linewidth=1, linestyle=(0, (6, 2)), label="Midclamp: middle duty's peak")
ax.fill_between(radii, blend[:, HIGHEST_MIN], blend[:, HIGHEST_MAX], color=BLEND, alpha=0.2, linewidth=0)
ax.plot(radii, blend[:, HIGHEST_MAX], color=BLEND, linewidth=2, label="Highest duty's peak: 3 phases read's shortest window")
ax.plot(radii, blend[:, HIGHEST_MIN], color=BLEND, linewidth=2, linestyle=(0, (6, 2)), label="Highest duty's low = middle duty's peak: 2 of 3 read's shortest window")
ax.fill_between(radii, blend[:, LOWEST_MIN], blend[:, LOWEST_MAX], color=BLEND, alpha=0.2, linewidth=0)
ax.plot(radii, blend[:, LOWEST_MAX], color=BLEND, linewidth=2, label="Lowest duty's peak")
ax.plot(radii, blend[:, LOWEST_MIN], color=BLEND, linewidth=2, linestyle=(0, (6, 2)), label="Lowest duty's low: 0 where k reaches 0")
ax.axhline(DUTY_PEAK, color="gray", linestyle=":", label=f"{DUTY_PEAK:.1%}, {(1 - DUTY_PEAK) * T_US:.2f} µs at 20 kHz: the window that works today")
mark_inflections(ax, labelY=0.45)
ax.set_xlim(0.3, 0.585)
ax.set_ylim(-0.02, 1.05)
ax.legend(loc="center left", fontsize=8)  # in the gap between the highest and the lowest duties
finish(ax, "|v| blend: each duty's range over a cycle", "Voltage vector magnitude r (per VBus)", "Duty Cycle")

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Blend_Duty.png"), dpi=100)
plt.show()
