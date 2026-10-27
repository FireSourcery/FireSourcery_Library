"""
Zero-sequence placement: the duty ceiling against a blend scheduled on the voltage magnitude.
All are the general clamp, duty = v - v_zero, v_zero = k*(v_max - 1) + (1 - k)*v_min, k the share of the null time at O111.
    midclamp            k = 1/2                                                         [svpwm_midclamp]
    ceiling sqrt3/2     k per vector: the highest duty held to sqrt3/2, k in [0, 1/2]   [svpwm_midclamp_ceiling]
                        k moves within the turn: midclamp away from each line-to-line peak, the ceiling about it
    |v| blend           k per magnitude: 1/2 up to |v| = Vbus/2, falling linearly to 0 at 0.539, the bottom clamp's radius for the window
                        midclamp leaves at Vbus/2 [SVPWM_CLAMP_RADIUS_OF_WINDOW]. One k for the whole turn: [svpwm_clamp] fed k(|v|)

Per placement and circle radius r (per Vbus):
    continuity: the largest duty step between neighbouring angles 0.01 deg apart, and the corners in v_zero per turn
    the shortest low-side window: the highest duty (3 phases read), the middle duty (2 of 3 read)
    narrow pulses: angle per turn with a high or a low time under 1 us, which the gate driver may swallow (520 ns dead time, 680 ns turn-on)
    clamped: angle per turn with a phase at 0 % (no switching, its low side on throughout)
    the PWM ripple current, RMS over the turn relative to midclamp's
    the track-and-hold current-vector error, the model of low_side_hold.py
Writes LowSideZeroSequenceBlend.png beside this file.
"""
import math
import os

import matplotlib.pyplot as plt
import numpy as np

F_PWM = 20000
T_US = 1e6 / F_PWM
EDGE_US = 1.0  # gate driver turn-on, 680 ns typ incl. 520 ns dead time, then the FET
LAG_DEG = 30.0
PHASES = (0.0, 2 * math.pi / 3, 4 * math.pi / 3)
R_SQRT3 = 1 / math.sqrt(3)
D = math.sqrt(3) / 2  # PHASE_SVPWM_DUTY_CEILING default
R_BLEND = 0.5         # |v| = sqrt3/2 of the linear limit
U_TODAY = 0.5 + D / 2  # midclamp's highest duty at Vbus/2, 93.3 %: the window that works today
R_WINDOW = U_TODAY / math.sqrt(3)  # 0.539, the bottom clamp's radius for that window [SVPWM_CLAMP_RADIUS_OF_WINDOW], and the ceiling's [svpwm_midclamp_ceiling_radius]


def magnitude(v):
    """|v| per Vbus from the three phase voltages, a zero-sum set: sqrt(2/3 * sum v^2)"""
    return math.sqrt(2.0 / 3.0 * sum(x * x for x in v))


def zero_k(v, k):
    return k * (max(v) - 1) + (1 - k) * min(v)


def zero_midclamp(v):
    return zero_k(v, 0.5)


def zero_ceiling(v):
    return min(max(max(v) - D, zero_midclamp(v)), min(v))


def k_ramp(x, x0, x1):
    """1/2 up to x0, falling linearly to 0 at x1"""
    return 0.5 * min(max((x1 - x) / (x1 - x0), 0.0), 1.0)


def k_blend(r):
    return k_ramp(r, R_BLEND, R_WINDOW)


def zero_blend(v):
    return zero_k(v, k_blend(magnitude(v)))


BLEND =f"|v| blend, Vbus/2–{R_WINDOW:.3f}"
PLACEMENTS = {
    "midclamp": zero_midclamp,
    "ceiling √3/2": zero_ceiling,
    BLEND: zero_blend,
}
ASCII = dict(zip(PLACEMENTS, ("midclamp", "ceiling sqrt3/2", "|v| blend")))


def phase_voltages(r, theta):
    return [r * math.cos(theta - p) for p in PHASES]


def turn(r, zero, n=36000):
    """Duties [n, 3] and v_zero [n] over one turn at radius r"""
    theta = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    v = np.array([phase_voltages(r, th) for th in theta])
    z = np.array([zero(list(row)) for row in v])
    return theta, np.clip(v - z[:, None], 0.0, 1.0), z


def corners(z, theta):
    """Slope breaks in v_zero per turn: second differences well above the smooth curvature"""
    dz = np.diff(np.append(z, z[0]))
    d2 = np.abs(np.diff(np.append(dz, dz[0])))
    step = theta[1] - theta[0]
    return int(np.sum(d2 > 20 * np.median(d2) + 1e-3 * step))


def clarke(a, b, c):
    return (2 * a - b - c) / 3, (b - c) / math.sqrt(3)


def simulate(r, f_e, tau, two_of_three, zero, settle_cycles=5):
    """Current-vector error |e| / I_peak over whole electrical cycles after settling, as low_side_hold.py"""
    dtheta = 2 * math.pi * f_e / F_PWM
    n = F_PWM // math.gcd(F_PWM, f_e)
    settle = settle_cycles * (F_PWM // f_e + 1)
    lag = math.radians(LAG_DEG)
    held, excl = [0.0] * 3, 0
    e_abs = []
    for k in range(settle + n):
        theta = k * dtheta
        v = phase_voltages(r, theta)
        z = zero(v)
        d = [x - z for x in v]
        i = [math.cos(theta - lag - p) for p in PHASES]
        for x in range(3):
            held[x] += (1 - math.exp(-max(0.0, (1 - d[x]) * T_US - EDGE_US) / tau)) * (i[x] - held[x])
        m = list(held)
        if two_of_three:
            top = max(range(3), key=lambda x: d[x])
            if d[top] > d[excl]: excl = top
            m[excl] = -(m[(excl + 1) % 3] + m[(excl + 2) % 3])
        if k >= settle:
            ea, eb = (p - q for p, q in zip(clarke(*m), clarke(*i)))
            e_abs.append(math.hypot(ea, eb))
    return np.array(e_abs)


def ripple_rms(r, zero, n_theta=120, n_t=1000):
    """RMS of the alpha-beta PWM ripple current, center-aligned, V_bus = T = L = 1. A third of the turn repeats the rest"""
    theta = np.linspace(0.0, 2 * math.pi / 3, n_theta, endpoint=False)
    v = r * np.cos(theta[:, None] - np.array(PHASES)[None, :])
    d = np.clip(v - np.array([zero(list(row)) for row in v])[:, None], 0.0, 1.0)
    t = (np.arange(n_t) + 0.5) / n_t
    on = (np.abs(t[None, :, None] - 0.5) < d[:, None, :] / 2).astype(float)
    v_n = on - on.mean(axis=2, keepdims=True)
    i = np.cumsum(v_n - (d - d.mean(axis=1, keepdims=True))[:, None, :], axis=1) / n_t
    i -= i.mean(axis=1, keepdims=True)
    alpha, beta = clarke(i[..., 0], i[..., 1], i[..., 2])
    return math.sqrt(np.mean(alpha ** 2 + beta ** 2))


def metrics(r, zero):
    theta, d, z = turn(r, zero, n=7200)
    s = np.sort(d, axis=1)
    high, low = d * T_US, (1 - d) * T_US
    narrow = np.any(((high > 1e-9) & (high < EDGE_US)) | ((low > 1e-9) & (low < EDGE_US)), axis=1)
    _, d_fine, _ = turn(r, zero, n=36000)
    return dict(
        jump=np.max(np.abs(np.diff(np.vstack([d_fine, d_fine[:1]]), axis=0))),
        corners=corners(z, theta),
        top=(1 - s[:, 2].max()) * T_US,
        middle=(1 - s[:, 1].max()) * T_US,
        narrow=360 * np.mean(narrow),
        clamped=360 * np.mean(np.any(d <= 1e-9, axis=1)),
    )


FW_MARGIN = 0.95  # FOC_FIELD_WEAKENING_V_MARGIN: field weakening holds |v| at this fraction of the circle
R_FW = {"Vbus/2": FW_MARGIN * 0.5, "0.539": FW_MARGIN * R_WINDOW, "Vbus/√3": FW_MARGIN * R_SQRT3}
RADII = (0.40, 0.45, R_FW["Vbus/2"], 0.50, R_FW["0.539"], 0.52, R_WINDOW, R_FW["Vbus/√3"], 0.56, R_SQRT3)
print("per radius: largest duty step 0.01 deg apart (%), corners in v_zero per turn, shortest window us (3 phases / 2 of 3),")
print("angle per turn with a pulse under 1 us, angle per turn with a phase clamped at 0 %, ripple / midclamp's")
print(f"  {'placement':20s} {'r':>6s} {'k':>5s} {'step %':>7s} {'corners':>7s} {'top us':>7s} {'mid us':>7s} {'narrow':>7s} {'clamped':>8s} {'ripple':>7s}")
table = {}
for r in RADII:
    base = ripple_rms(r, zero_midclamp)
    for name, zero in PLACEMENTS.items():
        m = metrics(r, zero)
        m["ripple"] = ripple_rms(r, zero) / base
        table[(name, r)] = m
        k = {"midclamp": "1/2", BLEND: f"{k_blend(r):.2f}"}.get(name, "var")
        print(f"  {ASCII[name]:20s} {r:6.3f} {k:>5s} {100 * m['jump']:7.3f} {m['corners']:7d} {m['top']:7.2f} {m['middle']:7.2f}"
              f" {m['narrow']:6.1f}d {m['clamped']:7.1f}d {m['ripple']:7.3f}")

CASES = ((300, 10.0), (500, 10.0))
HOLD = (("3 phases", 0.50, False), ("3 phases", R_WINDOW, False), ("2 of 3", 0.54, True), ("2 of 3", R_SQRT3, True))
print(f"\ncurrent-vector error / I_peak, max / rms; current lags V by {LAG_DEG:.0f} deg, {EDGE_US} us lost per window")
print(f"  {'read, radius':18s} {'placement':20s}" + "".join(f"{f'{f} Hz {t:.0f}us':>15s}" for f, t in CASES))
errors = {}
for read, r, two in HOLD:
    for name, zero in PLACEMENTS.items():
        row = []
        for f_e, tau in CASES:
            e = simulate(r, f_e, tau, two, zero)
            errors[(read, r, name, f_e)] = (e.max(), math.sqrt(np.mean(e ** 2)))
            row.append("{:.3f}/{:.3f}".format(*errors[(read, r, name, f_e)]))
        print(f"  {read + f', {r:.3f}':18s} {ASCII[name]:20s}" + "".join(f"{c:>15s}" for c in row))

# Plot: color is the placement
COLORS = dict(zip(PLACEMENTS, ("#eb6834", "#2a78d6", "#1baf7a")))
INK, GRID = "#6b6b6b", "#e4e4e4"


def finish(ax, title, xlabel, ylabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def mark_fw(ax, labels=True):
    """Dotted: the |v| field weakening holds at a Vbus/2, a 0.539 and a Vbus/sqrt3 circle"""
    for j, (name, r) in enumerate(R_FW.items()):
        ax.axvline(r, color=INK, linestyle=":", linewidth=1, label=None if labels or j else "field weakening's |v|, Vbus/2, 0.539 and Vbus/√3 circles")
        if labels:
            ax.annotate(f"FW |v|, {name} circle", (r, 1.0), xycoords=("data", "axes fraction"), xytext=(-3, -4), textcoords="offset points",
                        rotation=90, ha="right", va="top", fontsize=7, color=INK)


fig, axes = plt.subplots(3, 2, figsize=(15, 13.5))
(ax_duty, ax_shift), (ax_win, ax_rip), (ax_pulse, ax_err) = axes

R_TURN = 0.54
for name, zero in PLACEMENTS.items():
    theta, d, z = turn(R_TURN, zero, n=3600)
    ax_duty.plot(np.degrees(theta), d[:, 0], color=COLORS[name], linewidth=2, label=name + (f", k = {k_blend(R_TURN):.2f}" if name.startswith("|v|") else ""))
ax_duty.axhline(D, color=INK, linestyle=":", linewidth=1, label="√3/2")
ax_duty.set_xticks(range(0, 361, 60))
ax_duty.set_ylim(-0.02, 1.02)
ax_duty.legend(loc="upper center", fontsize=8)
finish(ax_duty, f"Phase A duty over one turn, r = {R_TURN}·Vbus", "Voltage vector angle (electrical degrees)", "Duty")

R_SHIFT = R_FW["0.539"]
_, _, z_mid = turn(R_SHIFT, zero_midclamp, n=3600)
for name, zero in list(PLACEMENTS.items())[1:]:
    theta, _, z = turn(R_SHIFT, zero, n=3600)
    ax_shift.plot(np.degrees(theta), z - z_mid, color=COLORS[name], linewidth=2, label=name)
ax_shift.set_xticks(range(0, 361, 60))
ax_shift.legend(loc="upper right", fontsize=8)
finish(ax_shift, f"Zero sequence below midclamp's, r = {R_SHIFT:.3f}·Vbus: every duty drops by it", "Voltage vector angle (electrical degrees)", "Shift (duty)")

r_sweep = np.linspace(0.30, R_SQRT3, 56)
sweep = {name: [metrics(r, zero) for r in r_sweep] for name, zero in PLACEMENTS.items()}
for name in PLACEMENTS:
    ax_win.plot(r_sweep, [m["top"] for m in sweep[name]], color=COLORS[name], linewidth=2, label=f"{name}, 3 phases read")
    ax_win.plot(r_sweep, [m["middle"] for m in sweep[name]], color=COLORS[name], linewidth=1.5, linestyle="--", label=f"{name}, 2 of 3 read")
ax_win.axhline(3.35, color=INK, linestyle=":", linewidth=1, label="3.35 µs, the Vbus/2 worst case today")
ax_win.set_ylim(0, 14)
ax_win.legend(loc="lower left", fontsize=7)
finish(ax_win, "Shortest low-side window over a turn", "Circle radius r (per Vbus)", "Window (µs)")

r_rip = np.linspace(0.30, R_SQRT3, 28)
for name, zero in PLACEMENTS.items():
    ax_rip.plot(r_rip, [ripple_rms(r, zero) / ripple_rms(r, zero_midclamp) for r in r_rip], color=COLORS[name], linewidth=2, label=name)
ax_rip.set_ylim(0.98, 1.12)
ax_rip.legend(loc="upper left", fontsize=8)
finish(ax_rip, "PWM ripple current, RMS relative to midclamp", "Circle radius r (per Vbus)", "Ripple / midclamp's")

for name in PLACEMENTS:
    ax_pulse.plot(r_sweep, [m["narrow"] for m in sweep[name]], color=COLORS[name], linewidth=2, label=f"{name}: a pulse under 1 µs")
    ax_pulse.plot(r_sweep, [m["clamped"] for m in sweep[name]], color=COLORS[name], linewidth=1.5, linestyle="--", label=f"{name}: a phase clamped at 0 %")
mark_fw(ax_pulse, labels=False)
ax_pulse.legend(loc="upper left", fontsize=7)
finish(ax_pulse, "Pulses under 1 µs (gate driver dead time 520 ns), and phases clamped at 0 %", "Circle radius r (per Vbus)", "Angle per turn (degrees)")
mark_fw(ax_win)

width = 0.8 / len(PLACEMENTS)
labels = [f"{read}\nr = {r:.3f}\n{f} Hz" for read, r, _ in HOLD for f, _ in CASES]
x = np.arange(len(labels))
for j, name in enumerate(PLACEMENTS):
    rms = [errors[(read, r, name, f)][1] for read, r, _ in HOLD for f, _ in CASES]
    peak = [errors[(read, r, name, f)][0] for read, r, _ in HOLD for f, _ in CASES]
    offset = (j - (len(PLACEMENTS) - 1) / 2) * width
    ax_err.bar(x + offset, rms, width * 0.92, color=COLORS[name], label=name)
    ax_err.plot(x + offset, peak, linestyle="none", marker="_", markersize=8, markeredgewidth=2, color=COLORS[name])
ax_err.set_xticks(x, labels, fontsize=7)
ax_err.legend(loc="upper left", fontsize=8)
finish(ax_err, f"Track-and-hold error, RMS (bars) and peak (ticks), τ {CASES[0][1]:.0f} µs", "", "Error / I_peak")

fig.tight_layout()
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "LowSideZeroSequenceBlend.png"), dpi=100)
plt.show()
