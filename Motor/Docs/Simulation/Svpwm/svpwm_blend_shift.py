import os
import numpy as np
import matplotlib.pyplot as plt
from svpwm_ceiling import INK, finish
from svpwm_blend import BLENDS, RADIUS_END, DUTY_START, FW_RADIUS, simulate, VZERO, VZERO_MIDCLAMP, K

# Where in the cycle each blend lowers the zero sequence: one sector, 0 to 60 degrees, repeats over the cycle
RADII = (0.52, FW_RADIUS, 0.56, RADIUS_END)
RADIUS_COLORS = ("#86b6ef", "#3987e5", "#1c5cab", "#0d366b")  # one hue, light to dark: the radius
degrees = np.linspace(0, 60, 601)

fig, axes = plt.subplots(1, 2, figsize=(16, 6), sharey=True, layout="constrained")
titles = {
    "|v| blend": "|v| blend: one k for the cycle, so the shift (½ − k)·(1 − spread)\nis largest where the spread is smallest, at each phase's peak",
    "duty-max blend": f"Duty-max blend: k per vector, so the shift is only where midclamp's\nhighest duty passes {DUTY_START:.1%}, around each line-to-line peak",
}
for ax, (name, k_of) in zip(axes, BLENDS.items()):
    for r, color in zip(RADII, RADIUS_COLORS):
        run = simulate(r, k_of, degrees / 360)
        k = f"k = {run[0, K]:.2f}" if name == "|v| blend" else f"k {run[:, K].min():.2f} to {run[:, K].max():.2f}"
        ax.plot(degrees, run[:, VZERO] - run[:, VZERO_MIDCLAMP], color=color, linewidth=2, label=f"r = {r:.3f}·VBus, {k}")
    ax.set_xticks((0, 30, 60), ("0°\nA's peak\nspread 1.5·r", "30°\nline-to-line peak\nspread √3·r", "60°\nC's negative peak\nspread 1.5·r"))
    ax.set_xlim(0, 60)
    ax.legend(loc="upper center", fontsize=9)
    finish(ax, titles[name], "Voltage vector angle within one sector", "vZero above midclamp's (duty): every duty drops by it")
    ax.title.set_fontsize(11)
axes[1].tick_params(labelleft=True)
fig.supxlabel("The shortest window over the sector: 3 phases read at 30°; 2 of 3 read at 60°, where A and B tie at the top", fontsize=10, color=INK)

plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Blend_Shift.png"), dpi=100)
plt.show()
