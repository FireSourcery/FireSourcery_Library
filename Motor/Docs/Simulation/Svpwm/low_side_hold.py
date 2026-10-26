"""
Track-and-hold model of a low-side phase current sense: KE R_DS(on) sense, Q5x/Q6x switch closed on PWM low, C40x 1 uF hold.
Per PWM period each phase's hold moves toward its true current by 1 - exp(-max(0, window - edge) / tau), window = (1 - d)*T,
and holds otherwise. Duties from midclamp SVPWM at circle radius r per V_bus. Current lags the voltage vector by LAG_DEG.

Reads compared, current-vector error / peak current:
    3 phases:   Clarke of the three holds
    2 of 3:     the highest-duty phase solved by i_a + i_b + i_c = 0 [_Phase_Analog_SolveI], optionally with handoff hysteresis
Also reports the error's harmonics in the current's rotating frame, the ripple the current loop sees.
tau (switch on-resistance x 1 uF) is not measured; 5 and 10 us bracket it.
Writes LowSideHold.png beside this file.
"""
import math
import os

import matplotlib.pyplot as plt
import numpy as np

F_PWM = 20000
T_US = 1e6 / F_PWM
EDGE_US = 1.0  # IR2104 turn-on, 680 ns typ incl. 520 ns dead time, then the FET
LAG_DEG = 30.0
PHASES = (0.0, 2 * math.pi / 3, 4 * math.pi / 3)

READS = {  # label: (radius per V_bus, two of three)
    "Vbus/2, 3 phases (today)": (0.5, False),
    "Vbus/2, 2 of 3": (0.5, True),
    "Vbus/sqrt3, 2 of 3": (1 / math.sqrt(3), True),
    "Vbus/sqrt3, 3 phases": (1 / math.sqrt(3), False),
}
COLORS = ("tab:green", "tab:purple", "tab:blue", "tab:red")
CASES = ((100, 5.0), (100, 10.0), (300, 5.0), (300, 10.0), (500, 5.0), (500, 10.0))


def duties(r, theta):
    v = [r * math.cos(theta - p) for p in PHASES]
    zero = (max(v) + min(v)) / 2
    return [0.5 + x - zero for x in v]


def clarke(a, b, c):
    return (2 * a - b - c) / 3, (b - c) / math.sqrt(3)


def simulate(r, f_e, tau, two_of_three, delta=0.0, settle_cycles=5):
    """Error over a whole number of electrical cycles after settling: angle, |e|, e in the current's dq frame."""
    dtheta = 2 * math.pi * f_e / F_PWM
    n = F_PWM // math.gcd(F_PWM, f_e)  # periods spanning whole cycles
    settle = settle_cycles * (F_PWM // f_e + 1)
    lag = math.radians(LAG_DEG)
    held, excl = [0.0] * 3, 0
    theta_out, e_abs, e_dq = [], [], []
    for k in range(settle + n):
        theta = k * dtheta
        d = duties(r, theta)
        i = [math.cos(theta - lag - p) for p in PHASES]
        for x in range(3):
            held[x] += (1 - math.exp(-max(0.0, (1 - d[x]) * T_US - EDGE_US) / tau)) * (i[x] - held[x])
        m = list(held)
        if two_of_three:
            top = max(range(3), key=lambda x: d[x])
            if d[top] > d[excl] + delta: excl = top
            m[excl] = -(m[(excl + 1) % 3] + m[(excl + 2) % 3])
        if k >= settle:
            ea, eb = (p - q for p, q in zip(clarke(*m), clarke(*i)))
            ti = theta - lag
            theta_out.append(theta % (2 * math.pi))
            e_abs.append(math.hypot(ea, eb))
            e_dq.append(complex(ea * math.cos(ti) + eb * math.sin(ti), -ea * math.sin(ti) + eb * math.cos(ti)))
    return np.array(theta_out), np.array(e_abs), np.array(e_dq), n * f_e / F_PWM


def harmonics(e_dq, cycles, count=3):
    """Largest components of the dq error, by order of the electrical frequency (0 = steady offset)."""
    spectrum = np.fft.fft(e_dq) / len(e_dq)
    orders = np.fft.fftfreq(len(e_dq), 1.0 / len(e_dq)) / cycles
    pick = np.argsort(np.abs(spectrum))[::-1][:count]
    return ", ".join(f"{orders[j]:+.0f}x {abs(spectrum[j]):.3f}" for j in pick)


print(f"current-vector error / I_peak, max / rms; current lags V by {LAG_DEG:.0f} deg, {EDGE_US} us lost per window")
print(f"{'f_e, tau':>12s}  " + "  ".join(f"{name:>26s}" for name in READS))
results = {}
for f_e, tau in CASES:
    row = []
    for name, (r, two) in READS.items():
        _, e_abs, _, _ = simulate(r, f_e, tau, two)
        results[(f_e, tau, name)] = (e_abs.max(), math.sqrt(np.mean(e_abs ** 2)))
        row.append("{:.3f} / {:.3f}".format(*results[(f_e, tau, name)]))
    print(f"{f_e:5d} Hz {tau:3.0f}us  " + "  ".join(f"{c:>26s}" for c in row))

print("\ndq error harmonics at 300 Hz, tau 10 us (order x f_e, magnitude):")
for name, (r, two) in READS.items():
    _, _, e_dq, cycles = simulate(r, 300, 10.0, two)
    print(f"  {name:26s} {harmonics(e_dq, cycles)}")

print("\n2 of 3 at Vbus/sqrt3, handoff hysteresis delta (duty), max / rms:")
print(f"{'f_e, tau':>12s}  " + "  ".join(f"{'delta ' + str(d):>14s}" for d in (0.0, 0.02, 0.05, 0.1)))
for f_e, tau in CASES:
    cells = []
    for delta in (0.0, 0.02, 0.05, 0.1):
        _, e_abs, _, _ = simulate(1 / math.sqrt(3), f_e, tau, True, delta)
        cells.append(f"{e_abs.max():.3f} / {math.sqrt(np.mean(e_abs ** 2)):.3f}")
    print(f"{f_e:5d} Hz {tau:3.0f}us  " + "  ".join(f"{c:>14s}" for c in cells))

# Plot
labels = {name: name.replace("sqrt3", "√3") for name in READS}
fig, (ax_trace, ax_bars) = plt.subplots(2, 1, figsize=(10, 9))
for (name, (r, two)), color in zip(READS.items(), COLORS):
    theta, e_abs, _, _ = simulate(r, 300, 10.0, two)
    order = np.argsort(theta)
    ax_trace.plot(np.degrees(theta[order]), e_abs[order], color=color, label=labels[name])
ax_trace.set_title("Current-vector error over one electrical cycle, 300 Hz, τ = 10 µs")
ax_trace.set_xlabel("Voltage vector angle (electrical degrees)")
ax_trace.set_ylabel("|i_measured − i| / I_peak")
ax_trace.set_xticks(range(0, 361, 30))
ax_trace.legend(loc="upper right", fontsize=8)
ax_trace.grid()

width = 0.2
x = np.arange(len(CASES))
for j, ((name, _), color) in enumerate(zip(READS.items(), COLORS)):
    rms = [results[(f_e, tau, name)][1] for f_e, tau in CASES]
    peak = [results[(f_e, tau, name)][0] for f_e, tau in CASES]
    ax_bars.bar(x + (j - 1.5) * width, rms, width, color=color, label=f"{labels[name]}, rms")
    ax_bars.plot(x + (j - 1.5) * width, peak, linestyle="none", marker="_", markersize=12, color=color)
ax_bars.set_xticks(x, [f"{f_e} Hz\nτ {tau:.0f} µs" for f_e, tau in CASES])
ax_bars.set_title("RMS (bars) and peak (ticks) error by electrical frequency and hold time constant")
ax_bars.set_ylabel("Error / I_peak")
ax_bars.legend(loc="upper left", fontsize=8)
ax_bars.grid(axis="y")

fig.tight_layout()
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "LowSideHold.png"), dpi=100)
plt.show()
