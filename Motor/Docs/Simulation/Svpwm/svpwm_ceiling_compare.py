import os
import matplotlib.pyplot as plt
from svpwm_3phase_ceiling import DUTY_CEILING, finish
from svpwm_3phase_ceiling_analysis import radii, ceiling, midclamp, HIGHEST_MAX, MIDDLE_MAX, T_US, MIDCLAMP, CEILING, mark_inflections

# Against midclamp: the peaks that set the read window
fig, ax = plt.subplots(figsize=(16, 6), layout="constrained")
for run, color, width, name in ((midclamp, MIDCLAMP, 4, "svpwm_midclamp"), (ceiling, CEILING, 1.75, "svpwm_midclamp_ceiling")):  # midclamp wider, under: below the onset the two coincide
    ax.plot(radii, run[:, HIGHEST_MAX], color=color, linewidth=width, label=f"{name}: highest duty's peak, 3 phases read")
    ax.plot(radii, run[:, MIDDLE_MAX], color=color, linewidth=width, linestyle=(0, (6, 2)), label=f"{name}: middle duty's peak, 2 of 3 read")
ax.axhline(DUTY_CEILING, color="gray", linestyle=":", label=f"Ceiling √3/2: {(1 - DUTY_CEILING) * T_US:.2f} µs at 20 kHz")
mark_inflections(ax, labelY=0.68)
ax.set_xlim(0, 0.6)
ax.set_ylim(0.5, 1.02)
ax.legend(loc="upper left", ncol=3, fontsize=9)
finish(ax, "Against midclamp: the peak duty over a cycle, the highest for 3 phases read, the middle for 2 of 3",
       "Voltage vector magnitude r (per VBus)", "Duty Cycle")

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_3Phase_Ceiling_Compare.png"), dpi=100)
plt.show()
