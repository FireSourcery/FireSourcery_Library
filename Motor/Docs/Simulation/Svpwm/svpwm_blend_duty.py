import os
import matplotlib.pyplot as plt
from svpwm_ceiling import finish
from svpwm_blend import MAGNITUDE, DUTY_MAX, DUTY_START
from svpwm_blend_analysis import radii, magnitude, dutyMax, midclamp, HIGHEST_MIN, HIGHEST_MAX, MIDDLE_MAX, LOWEST_MIN, LOWEST_MAX, T_US, mark_inflections

# What each blend does to the duties: each duty's range over a cycle, midclamp's peaks for reference
fig, axes = plt.subplots(1, 2, figsize=(16, 6.5), sharey=True, layout="constrained")
for ax, (name, run, color) in zip(axes, (("|v| blend", magnitude, MAGNITUDE), ("Duty-max blend", dutyMax, DUTY_MAX))):
    ax.plot(radii, midclamp[:, HIGHEST_MAX], color="gray", linewidth=1, label="Midclamp: highest duty's peak")
    ax.plot(radii, midclamp[:, MIDDLE_MAX], color="gray", linewidth=1, linestyle=(0, (6, 2)), label="Midclamp: middle duty's peak")
    ax.fill_between(radii, run[:, HIGHEST_MIN], run[:, HIGHEST_MAX], color=color, alpha=0.2, linewidth=0)
    ax.plot(radii, run[:, HIGHEST_MAX], color=color, linewidth=2, label="Highest duty's peak: 3 phases read's shortest window")
    ax.plot(radii, run[:, HIGHEST_MIN], color=color, linewidth=2, linestyle=(0, (6, 2)), label="Highest duty's low = middle duty's peak: 2 of 3 read's shortest window")
    ax.fill_between(radii, run[:, LOWEST_MIN], run[:, LOWEST_MAX], color=color, alpha=0.2, linewidth=0)
    ax.plot(radii, run[:, LOWEST_MAX], color=color, linewidth=2, label="Lowest duty's peak")
    ax.plot(radii, run[:, LOWEST_MIN], color=color, linewidth=2, linestyle=(0, (6, 2)), label="Lowest duty's low: 0 where k reaches 0")
    ax.axhline(DUTY_START, color="gray", linestyle=":", label=f"{DUTY_START:.1%}, {(1 - DUTY_START) * T_US:.2f} µs at 20 kHz: the window that works today")
    mark_inflections(ax, labelY=0.45)
    ax.set_xlim(0.3, 0.585)
    ax.set_ylim(-0.02, 1.05)
    ax.legend(loc="center left", fontsize=8)  # in the gap between the highest and the lowest duties
    finish(ax, f"{name}: each duty's range over a cycle", "Voltage vector magnitude r (per VBus)", "Duty Cycle")
axes[1].tick_params(labelleft=True)

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Blend_Duty.png"), dpi=100)
plt.show()
