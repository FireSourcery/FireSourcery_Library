"""
Low-side window per PWM period under midclamp SVPWM, through the integer output path:
foc_inv_park -> foc_inv_clarke -> svpwm_norm_vbus_inv -> svpwm_midclamp.

Low-side current sensing reads a phase only while its low-side switch conducts, (1 - d)*T per period.
    3 phases read:   the highest duty sets the shortest window, 1/2 + (sqrt3/2)*r
    2 of 3 read:     the highest duty is left out, the middle sets it, 1/2 + (3/4)*r
r = 1/2 reading 3 phases and r = 1/sqrt3 reading 2 of 3 both bottom out at 93.3 %.

Also checks the third phase solved by i_a + i_b + i_c = 0 [_Phase_Analog_SolveI] through foc_clarke.
Writes LowSideWindow.png beside this file.
"""
import math
import os

import matplotlib.pyplot as plt
import numpy as np

T_US = 50.0  # 20 kHz PWM
V_BUS = 25098  # VBus_Pu, 72 V of 94 V
EDGE_US = 1.0  # IR2104 turn-on, 680 ns typ incl. 520 ns dead time, then the FET

FRACT16_SCALE = 32768
FRACT16_MAX = 32767
FRACT16_1_DIV_2 = 0x4000
FRACT16_1_DIV_3 = 0x2AAA
FRACT16_1_DIV_SQRT3 = 0x49E7
FRACT16_SQRT3_DIV_2 = 0x6EDA


def fract16_mul(a, b):
    return (a * b) >> 15


def fract16_sat(x):
    return max(-FRACT16_MAX, min(FRACT16_MAX, x))


def div_trunc(n, d):  # C integer division
    q = abs(n) // abs(d)
    return q if (n >= 0) == (d >= 0) else -q


def duties(vd, vq, theta, v_bus):
    """[Phase_WriteSvpwm] duties in fract16 for the dq command at theta."""
    sin = fract16_sat(round(math.sin(theta) * FRACT16_SCALE))
    cos = fract16_sat(round(math.cos(theta) * FRACT16_SCALE))
    alpha = fract16_mul(vd, cos) - fract16_mul(vq, sin)
    beta = fract16_mul(vd, sin) + fract16_mul(vq, cos)
    va = fract16_sat(alpha)
    vb = fract16_sat(-fract16_mul(alpha, FRACT16_1_DIV_2) + fract16_mul(beta, FRACT16_SQRT3_DIV_2))
    vc = fract16_sat(-fract16_mul(alpha, FRACT16_1_DIV_2) - fract16_mul(beta, FRACT16_SQRT3_DIV_2))
    v_bus_inv = (FRACT16_SCALE << 15) // v_bus
    na, nb, nc = (div_trunc(v * v_bus_inv, FRACT16_SCALE) for v in (va, vb, vc))
    v_zero = div_trunc(max(na, nb, nc) + min(na, nb, nc), 2) - FRACT16_1_DIV_2
    return [min(max(x - v_zero, 0), FRACT16_MAX) for x in (na, nb, nc)]


def window_us(duty):
    return (1.0 - duty / FRACT16_SCALE) * T_US


def foc_clarke(a, b, c):
    return (fract16_mul(2 * a - b - c, FRACT16_1_DIV_3), fract16_mul(b - c, FRACT16_1_DIV_SQRT3))


angles = np.linspace(0.0, 2 * math.pi, 3600, endpoint=False)
degrees = np.degrees(angles)
radii = {"Vbus/2": FRACT16_1_DIV_2, "Vbus/sqrt3": FRACT16_1_DIV_SQRT3}

phase_duty = {}  # [n, 3] per radius
for name, k in radii.items():
    r = fract16_mul(V_BUS, k)
    phase_duty[name] = np.array([duties(r, 0, th, V_BUS) for th in angles]) / FRACT16_SCALE  # v on d: the vector angle is theta

top = {name: np.sort(d, axis=1)[:, 2] for name, d in phase_duty.items()}
mid = {name: np.sort(d, axis=1)[:, 1] for name, d in phase_duty.items()}
for name in radii:
    print(f"{name:8s}  3 phases: max duty {top[name].max():6.1%}, min window {(1 - top[name].max()) * T_US:4.2f} us"
          f"   2 of 3: max duty {mid[name].max():6.1%}, min window {(1 - mid[name].max()) * T_US:4.2f} us")

dead = math.degrees(math.acos(1 - 2 * EDGE_US / T_US))
print(f"Vbus/sqrt3, 3 phases: top window under {EDGE_US} us for +/-{dead:.1f} deg about each line-to-line peak,"
      f" {2 * dead / 360 * 1e6 / T_US / 300:.1f} PWM periods at 300 Hz, {2 * dead / 360 * 1e6 / T_US / 100:.1f} at 100 Hz")

# The solved phase through the integer Clarke, against a measured three-wire triplet
worst = 0
for th in angles[::3]:
    i = [int(round(26000 * math.cos(th - p))) for p in (0.0, 2 * math.pi / 3, 4 * math.pi / 3)]
    i[2] = -(i[0] + i[1])
    for skip in range(3):
        m = list(i)
        m[skip] = fract16_sat(-(m[(skip + 1) % 3] + m[(skip + 2) % 3]))
        worst = max(worst, max(abs(x - y) for x, y in zip(foc_clarke(*m), foc_clarke(*i))))
print(f"foc_clarke of the solved triplet vs the measured: max |diff| {worst} LSB")

# Plot
colors = ("red", "green", "blue")
fig, (ax_duty, ax_win) = plt.subplots(2, 1, figsize=(10, 9), sharex=True)
for x, (label, color) in enumerate(zip("ABC", colors)):
    ax_duty.plot(degrees, phase_duty["Vbus/sqrt3"][:, x], color=color, label=f"Duty {label}, Vbus/√3")
    ax_duty.plot(degrees, phase_duty["Vbus/2"][:, x], color=color, alpha=0.3, linestyle="--", label=f"Duty {label}, Vbus/2")
ax_duty.axhline(1.0 - 3.35 / T_US, color="gray", linestyle=":", label="93.3 %")
ax_duty.set_title("Midclamp SVPWM duty, current-loop circle at Vbus/√3 and Vbus/2")
ax_duty.set_ylabel("Duty")
ax_duty.legend(loc="lower right", ncol=2, fontsize=8)
ax_duty.grid()

ax_win.plot(degrees, (1 - top["Vbus/2"]) * T_US, color="tab:green", label="Vbus/2, 3 phases read: highest duty")
ax_win.plot(degrees, (1 - top["Vbus/sqrt3"]) * T_US, color="tab:red", label="Vbus/√3, 3 phases read: highest duty")
ax_win.plot(degrees, (1 - mid["Vbus/sqrt3"]) * T_US, color="tab:blue", label="Vbus/√3, 2 of 3 read: middle duty")
ax_win.axhline(3.35, color="gray", linestyle=":", label="3.35 µs, the Vbus/2 worst case")
ax_win.axhspan(0, EDGE_US, color="tab:red", alpha=0.1, label=f"< {EDGE_US} µs, lost to the switching edge")
ax_win.set_title(f"Shortest low-side window the read needs, {1e3 / T_US:.0f} kHz")
ax_win.set_xlabel("Voltage vector angle (electrical degrees)")
ax_win.set_ylabel("Window (µs)")
ax_win.set_xticks(range(0, 361, 30))
ax_win.set_ylim(0, 8)
ax_win.legend(loc="upper right", fontsize=8)
ax_win.grid()

fig.tight_layout()
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "LowSideWindow.png"), dpi=100)
plt.show()
