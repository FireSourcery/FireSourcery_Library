import os
import matplotlib.pyplot as plt
from svpwm_3phase_ceiling import DUTY_CEILING, finish
from svpwm_3phase_ceiling_analysis import radii, ceiling, HIGHEST_MIN, HIGHEST_MAX, LOWEST_MIN, LOWEST_MAX, T_US, CEILING, mark_inflections

# What it does to the duties: each duty's range over a cycle
fig, ax = plt.subplots(figsize=(16, 6), layout="constrained")
ax.fill_between(radii, ceiling[:, HIGHEST_MIN], ceiling[:, HIGHEST_MAX], color=CEILING, alpha=0.15, linewidth=0)
ax.plot(radii, ceiling[:, HIGHEST_MAX], color=CEILING, linewidth=2, label="Highest duty, its peak: the shortest window, 3 phases read")
ax.plot(radii, ceiling[:, HIGHEST_MIN], color=CEILING, linewidth=2, linestyle=(0, (6, 2)), label="Highest duty, its low = the middle duty's peak: the shortest window, 2 of 3 read")
ax.fill_between(radii, ceiling[:, LOWEST_MIN], ceiling[:, LOWEST_MAX], color=CEILING, alpha=0.15, linewidth=0)
ax.plot(radii, ceiling[:, LOWEST_MAX], color=CEILING, linewidth=2, label="Lowest duty, its peak: 0 once the bottom clamp holds throughout")
ax.plot(radii, ceiling[:, LOWEST_MIN], color=CEILING, linewidth=2, linestyle=(0, (6, 2)), label="Lowest duty, its low: 0 once the bottom clamp starts")
ax.axhline(DUTY_CEILING, color="gray", linestyle=":", label=f"Ceiling √3/2: {(1 - DUTY_CEILING) * T_US:.2f} µs at 20 kHz")
mark_inflections(ax, labelY=0.5)
ax.set_xlim(0, 0.6)
ax.set_ylim(-0.02, 1.2)
ax.legend(loc="upper left", ncol=3, fontsize=9)
finish(ax, "What it does to the duties: each duty's range over a cycle, svpwm_midclamp_ceiling", "Voltage vector magnitude r (per VBus)", "Duty Cycle")

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_3Phase_Ceiling_Duty.png"), dpi=100)
plt.show()
