import os
import numpy as np
import matplotlib.pyplot as plt
from svpwm_ceiling import INK, finish
from svpwm_blend import RADIUS_END, FW_RADIUS, k_of_magnitude, simulate, VZERO, VZERO_MIDCLAMP, K

# Where in the cycle the blend lowers the zero sequence: one sector, 0 to 60 degrees, repeats over the cycle
RADII = (FW_RADIUS, 0.52, 0.53, RADIUS_END)
RADIUS_COLORS = ("#86b6ef", "#3987e5", "#1c5cab", "#0d366b")  # one hue, light to dark: the radius
degrees = np.linspace(0, 60, 601)

fig, ax = plt.subplots(figsize=(8, 6), layout="constrained")
for r, color in zip(RADII, RADIUS_COLORS):
    run = simulate(r, k_of_magnitude, degrees / 360)
    ax.plot(degrees, run[:, VZERO] - run[:, VZERO_MIDCLAMP], color=color, linewidth=2, label=f"r = {r:.3f}·VBus, k = {run[0, K]:.2f}")
ax.set_xticks((0, 30, 60), ("0°\nA's peak\nspread 1.5·r", "30°\nline-to-line peak\nspread √3·r", "60°\nC's negative peak\nspread 1.5·r"))
ax.set_xlim(0, 60)
ax.legend(loc="upper center", fontsize=9)
finish(ax, "|v| blend: one k for the cycle, so the shift (½ − k)·(1 − spread)\nis largest where the spread is smallest, at each phase's peak",
       "Voltage vector angle within one sector", "vZero above midclamp's (duty): every duty drops by it")
ax.title.set_fontsize(11)
fig.supxlabel("The shortest window over the sector: 3 phases read at 30°; 2 of 3 read at 60°, where A and B tie at the top", fontsize=10, color=INK)

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Blend_Shift.png"), dpi=100)
plt.show()
