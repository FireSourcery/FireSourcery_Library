"""
Zero-sequence placement for a low-side phase current sense: KE R_DS(on) read in the low-side window, (1 - d)*T.
Lowering the zero sequence adds the same low-side time to every phase and leaves the line-to-line voltages unchanged.
The room is the bottom phase's duty, 1/2 - spread/2, which is 0 at the line-to-line peak at the linear limit Vbus/sqrt3:
with 3 phases read, a top duty held to D reaches only r = D/sqrt3.

Placements, duty = v - v_zero:
    general clamp k     v_zero = k*(v_max - 1) + (1 - k)*v_min              [svpwm_clamp], k of the null time at O111
        k = 1/2         midclamp, v_zero = (v_max + v_min)/2 - 1/2          [svpwm_midclamp]
        k = 0           bottom clamp, v_zero = v_min, the O000 null only
    ceiling D           v_zero = clamp(v_max - D, midclamp, bottom clamp)   [svpwm_midclamp_ceiling], D = sqrt3/2 [PHASE_SVPWM_DUTY_CEILING]
                        the general clamp with k = (D - spread)/(1 - spread) per vector, held to [0, 1/2]

The ceiling keeps midclamp's window at Vbus/2 to r = (1/2 + sqrt3/4)/sqrt3 = 0.539 [svpwm_midclamp_ceiling_radius]

Per placement, circle radius, and read (3 phases, or 2 of 3 with the highest duty solved [_Phase_Analog_SolveI]):
    the shortest low-side window, through the integer output path as in low_side_window.py
    the track-and-hold current-vector error, the model of low_side_hold.py
    the PWM ripple current, RMS over the turn relative to midclamp at the same radius and switching frequency
Writes LowSideZeroSequence.png beside this file.
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
CASES = ((100, 5.0), (100, 10.0), (300, 5.0), (300, 10.0), (500, 5.0), (500, 10.0))

V_BUS = 25098  # VBus_Pu, 72 V of 94 V
FRACT16_SCALE = 32768
FRACT16_MAX = 32767
FRACT16_1_DIV_2 = 0x4000
FRACT16_1_DIV_SQRT3 = 0x49E7
FRACT16_SQRT3_DIV_2 = 0x6EDA

R_HALF, R_SQRT3 = 0.5, 1 / math.sqrt(3)
D_SQRT3_DIV_2 = FRACT16_SQRT3_DIV_2 / FRACT16_SCALE  # PHASE_SVPWM_DUTY_CEILING default
D_TODAY = 0.5 + math.sqrt(3) / 4  # 93.3 %, the top duty at Vbus/2 today: the 3.35 us window that works
K_QUARTER = 0.25

RADII = {"Vbus/2": R_HALF, "Vbus/sqrt3": R_SQRT3}
READS = {"3 phases": False, "2 of 3": True}


# Placements, as v_zero of the three phase voltages per V_bus
def zero_midclamp(v): return (max(v) + min(v)) / 2 - 0.5
def zero_clamp(k): return lambda v: k * (max(v) - 1) + (1 - k) * min(v)
def zero_ceiling(ceiling): return lambda v: min(max(max(v) - ceiling, zero_midclamp(v)), min(v))


PLACEMENTS = {
    "midclamp": zero_midclamp,
    "ceiling sqrt3/2": zero_ceiling(D_SQRT3_DIV_2),
    "ceiling 93.3 %": zero_ceiling(D_TODAY),
    "clamp k=1/4": zero_clamp(K_QUARTER),
    "bottom clamp": zero_clamp(0.0),
}


def duties(r, theta, zero):
    v = [r * math.cos(theta - p) for p in PHASES]
    z = zero(v)
    return [x - z for x in v]


def clarke(a, b, c):
    return (2 * a - b - c) / 3, (b - c) / math.sqrt(3)


# Integer output path: [FOC_ProcInvClarkePark] -> svpwm_norm_vbus_inv -> [svpwm_midclamp] / [svpwm_clamp] / [svpwm_midclamp_ceiling]
def fract16_mul(a, b):
    return (a * b) >> 15


def fract16_sat(x):
    return max(-FRACT16_MAX, min(FRACT16_MAX, x))


def div_trunc(n, d):  # C integer division
    q = abs(n) // abs(d)
    return q if (n >= 0) == (d >= 0) else -q


def phase_voltages(vd, vq, theta, v_bus):
    """Phase voltages per V_bus in fract16, for the dq command at theta"""
    sin = fract16_sat(round(math.sin(theta) * FRACT16_SCALE))
    cos = fract16_sat(round(math.cos(theta) * FRACT16_SCALE))
    alpha = fract16_mul(vd, cos) - fract16_mul(vq, sin)
    beta = fract16_mul(vd, sin) + fract16_mul(vq, cos)
    va = fract16_sat(alpha)
    vb = fract16_sat(-fract16_mul(alpha, FRACT16_1_DIV_2) + fract16_mul(beta, FRACT16_SQRT3_DIV_2))
    vc = fract16_sat(-fract16_mul(alpha, FRACT16_1_DIV_2) - fract16_mul(beta, FRACT16_SQRT3_DIV_2))
    v_bus_inv = (FRACT16_SCALE << 15) // v_bus
    return [div_trunc(v * v_bus_inv, FRACT16_SCALE) for v in (va, vb, vc)]


def svpwm_midclamp(va, vb, vc):
    v_zero = div_trunc(max(va, vb, vc) + min(va, vb, vc), 2) - FRACT16_1_DIV_2
    return [min(max(x - v_zero, 0), FRACT16_MAX) for x in (va, vb, vc)]


def svpwm_clamp(va, vb, vc, k):
    v_max, v_min = max(va, vb, vc), min(va, vb, vc)
    v_zero = fract16_mul(k, v_max - FRACT16_SCALE) + fract16_mul(FRACT16_SCALE - k, v_min)
    return [min(max(x - v_zero, 0), FRACT16_MAX) for x in (va, vb, vc)]


def svpwm_midclamp_ceiling(va, vb, vc, ceiling):
    v_max, v_min = max(va, vb, vc), min(va, vb, vc)
    v_zero = min(max(v_max - ceiling, div_trunc(v_max + v_min, 2) - FRACT16_1_DIV_2), v_min)  # math_clamp(value, lower, upper)
    return [min(max(x - v_zero, 0), FRACT16_MAX) for x in (va, vb, vc)]


def svpwm_midclamp_duty_peak(radius):
    return FRACT16_1_DIV_2 + fract16_mul(radius, FRACT16_SQRT3_DIV_2)


def svpwm_midclamp_ceiling_radius(duty_peak, ceiling):
    return fract16_mul((2 * duty_peak - FRACT16_SCALE) if duty_peak < ceiling else duty_peak, FRACT16_1_DIV_SQRT3)


# The radius [FOC_V_CIRCLE_PER_V_BUS] at which the ceiling keeps midclamp's window at Vbus/2, reading 3 phases
R_CEILING_TODAY = svpwm_midclamp_ceiling_radius(svpwm_midclamp_duty_peak(FRACT16_1_DIV_2), FRACT16_SQRT3_DIV_2) / FRACT16_SCALE

INTEGER_PLACEMENTS = {
    "midclamp": svpwm_midclamp,
    "ceiling sqrt3/2": lambda va, vb, vc: svpwm_midclamp_ceiling(va, vb, vc, FRACT16_SQRT3_DIV_2),
    "clamp k=1/4": lambda va, vb, vc: svpwm_clamp(va, vb, vc, round(K_QUARTER * FRACT16_SCALE)),
    "bottom clamp": lambda va, vb, vc: svpwm_clamp(va, vb, vc, 0),
}
INTEGER_RADII = {"Vbus/2": FRACT16_1_DIV_2, "Vbus/sqrt3": FRACT16_1_DIV_SQRT3}


def integer_duties(svpwm, r, angles):
    """[n, 3] duties per unit through the integer path, the voltage vector of radius r per V_bus on d"""
    vd = round(r * V_BUS)
    return np.array([svpwm(*phase_voltages(vd, 0, th, V_BUS)) for th in angles]) / FRACT16_SCALE


def simulate(r, f_e, tau, two_of_three, zero, settle_cycles=5):
    """Current-vector error |e| / I_peak over a whole number of electrical cycles after settling, as low_side_hold.py"""
    dtheta = 2 * math.pi * f_e / F_PWM
    n = F_PWM // math.gcd(F_PWM, f_e)  # periods spanning whole cycles
    settle = settle_cycles * (F_PWM // f_e + 1)
    lag = math.radians(LAG_DEG)
    held, excl = [0.0] * 3, 0
    e_abs = []
    for k in range(settle + n):
        theta = k * dtheta
        d = duties(r, theta, zero)
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
    on = (np.abs(t[None, :, None] - 0.5) < d[:, None, :] / 2).astype(float)  # high about the period's center
    v_n = on - on.mean(axis=2, keepdims=True)  # phase to neutral
    i = np.cumsum(v_n - (d - d.mean(axis=1, keepdims=True))[:, None, :], axis=1) / n_t
    i -= i.mean(axis=1, keepdims=True)
    alpha, beta = clarke(i[..., 0], i[..., 1], i[..., 2])
    return math.sqrt(np.mean(alpha ** 2 + beta ** 2))


# Reach: 3 phases read, the top duty held to D. The bottom clamp gives out where the spread, sqrt3*r, exceeds D
print("3 phases read, top duty held to D: reaches r = D/sqrt3 (spread sqrt3*r is the most the offset can leave)")
for name, ceiling in (("ceiling 93.3 %", D_TODAY), ("ceiling sqrt3/2", D_SQRT3_DIV_2)):
    print(f"  {name:16s} D {ceiling:.3f}  r {ceiling / math.sqrt(3):.4f} Vbus ({ceiling / math.sqrt(3) / R_HALF - 1:+.1%} over Vbus/2)")
print(f"  ceiling sqrt3/2 is midclamp below r = {(2 * D_SQRT3_DIV_2 - 1) / math.sqrt(3):.4f}, the bottom clamp at Vbus/sqrt3")
print(f"  midclamp at Vbus/2 peaks at {svpwm_midclamp_duty_peak(FRACT16_1_DIV_2)} ({svpwm_midclamp_duty_peak(FRACT16_1_DIV_2) / FRACT16_SCALE:.4f});"
      f" ceiling sqrt3/2 reaches it at r = {R_CEILING_TODAY:.5f} [svpwm_midclamp_ceiling_radius] ({R_CEILING_TODAY / R_HALF - 1:+.1%})")

# Windows over the turn, integer path
angles = np.linspace(0.0, 2 * math.pi, 3600, endpoint=False)
degrees = np.degrees(angles)
duty = {}  # (placement, radius): [n, 3] per unit
for placement in ("midclamp", "ceiling sqrt3/2"):
    for radius, k in INTEGER_RADII.items():
        duty[(placement, radius)] = np.array([INTEGER_PLACEMENTS[placement](*phase_voltages(fract16_mul(V_BUS, k), 0, th, V_BUS))
                                              for th in angles]) / FRACT16_SCALE
r_reach = fract16_mul(V_BUS, round(D_TODAY / math.sqrt(3) * FRACT16_SCALE))  # console only: the reach of the 93.3 % ceiling
duty[("ceiling 93.3 %", "0.539")] = np.array([svpwm_midclamp_ceiling(*phase_voltages(r_reach, 0, th, V_BUS), round(D_TODAY * FRACT16_SCALE))
                                              for th in angles]) / FRACT16_SCALE
r_today = fract16_mul(V_BUS, round(R_CEILING_TODAY * FRACT16_SCALE))  # [_FOC_VCircle] at the board's FOC_V_CIRCLE_PER_V_BUS
duty[("ceiling sqrt3/2", "0.539")] = np.array([INTEGER_PLACEMENTS["ceiling sqrt3/2"](*phase_voltages(r_today, 0, th, V_BUS))
                                               for th in angles]) / FRACT16_SCALE
window = {key: (1 - np.sort(d, axis=1)[:, ::-1]) * T_US for key, d in duty.items()}  # columns: top, middle, bottom duty's window

print("\nshortest low-side window the read needs, us (integer output path); degrees per turn at or under 3.35 us")
print(f"  {'placement, radius':28s} {'3 phases: top':>14s} {'deg':>6s}   {'2 of 3: middle':>14s} {'deg':>6s}")
for (placement, radius), w in window.items():
    print(f"  {placement + ', ' + radius:28s} {w[:, 0].min():14.2f} {360 * np.mean(w[:, 0] <= 3.35 + 1e-9):6.1f}"
          f"   {w[:, 1].min():14.2f} {360 * np.mean(w[:, 1] <= 3.35 + 1e-9):6.1f}")

# Shortest window over the turn against the radius, integer path
reach_radii = np.linspace(0.30, R_SQRT3, 56)
reach_angles = np.linspace(0.0, 2 * math.pi, 720, endpoint=False)
reach = {}  # placement: ([top], [middle]) shortest window per radius, us
for placement, svpwm in INTEGER_PLACEMENTS.items():
    sorted_duty = [np.sort(integer_duties(svpwm, r, reach_angles), axis=1) for r in reach_radii]
    reach[placement] = (np.array([(1 - d[:, 2].max()) * T_US for d in sorted_duty]), np.array([(1 - d[:, 1].max()) * T_US for d in sorted_duty]))

# Track-and-hold error
HOLD = {}  # label: (radius, two of three, placement)
for read, two in READS.items():
    for placement in ("midclamp", "ceiling sqrt3/2"):
        for radius, r in RADII.items():
            HOLD[f"{read}, {placement}, {radius}"] = (r, two, placement)
HOLD["3 phases, ceiling 93.3 %, 0.539"] = (D_TODAY / math.sqrt(3), False, "ceiling 93.3 %")
HOLD["3 phases, ceiling sqrt3/2, 0.539"] = (R_CEILING_TODAY, False, "ceiling sqrt3/2")

print(f"\ncurrent-vector error / I_peak, max / rms; current lags V by {LAG_DEG:.0f} deg, {EDGE_US} us lost per window")
print(f"  {'read, placement, radius':40s}" + "".join(f"{f'{f_e} Hz {tau:.0f}us':>15s}" for f_e, tau in CASES))
errors = {}
for label, (r, two, placement) in HOLD.items():
    row = []
    for f_e, tau in CASES:
        e = simulate(r, f_e, tau, two, PLACEMENTS[placement])
        errors[(label, f_e, tau)] = (e.max(), math.sqrt(np.mean(e ** 2)))
        row.append("{:.3f}/{:.3f}".format(*errors[(label, f_e, tau)]))
    print(f"  {label:40s}" + "".join(f"{c:>15s}" for c in row))

# Ripple
ripple_radii = np.linspace(0.05, R_SQRT3, 48)
ripple = {name: np.array([ripple_rms(r, zero) for r in ripple_radii]) for name, zero in PLACEMENTS.items()}
ratio = {name: ripple[name] / ripple["midclamp"] for name in PLACEMENTS}
print("\nPWM ripple current RMS / midclamp's, same radius and switching frequency")
table_radii = (0.2, 0.35, (2 * D_SQRT3_DIV_2 - 1) / math.sqrt(3), R_HALF, D_TODAY / math.sqrt(3), R_SQRT3)
ratio_at = {name: {r: ripple_rms(r, zero) / ripple_rms(r, zero_midclamp) for r in table_radii} for name, zero in PLACEMENTS.items()}
print(f"  {'radius':>8s}" + "".join(f"{name:>17s}" for name in PLACEMENTS if name != "midclamp"))
for r in table_radii:
    print(f"  {r:8.4f}" + "".join(f"{ratio_at[name][r]:17.3f}" for name in PLACEMENTS if name != "midclamp"))

# The general clamp at a fixed k, against the ceiling, which picks k per vector
print("\ngeneral clamp at fixed k, against the ceiling sqrt3/2")
print("  shortest window us: 3 phases at Vbus/2, 2 of 3 at Vbus/sqrt3 (integer path);  ripple / midclamp at r 0.2 / 0.35 / 0.5;")
print("  error max/rms at 300 Hz tau 10 us: 3 phases at Vbus/2, 2 of 3 at Vbus/sqrt3")
fixed = {f"k = {k}": (round(float(k_value) * FRACT16_SCALE), zero_clamp(float(k_value)))
         for k, k_value in (("1/2", 0.5), ("3/8", 0.375), ("1/4", 0.25), ("1/8", 0.125), ("0", 0.0))}
fixed_rows = {name: (lambda va, vb, vc, k=k: svpwm_clamp(va, vb, vc, k), zero) for name, (k, zero) in fixed.items()}
fixed_rows["ceiling sqrt3/2"] = (INTEGER_PLACEMENTS["ceiling sqrt3/2"], PLACEMENTS["ceiling sqrt3/2"])
for name, (svpwm, zero) in fixed_rows.items():
    top = (1 - np.sort(integer_duties(svpwm, R_HALF, reach_angles), axis=1)[:, 2].max()) * T_US
    mid = (1 - np.sort(integer_duties(svpwm, R_SQRT3, reach_angles), axis=1)[:, 1].max()) * T_US
    rip = [ripple_rms(r, zero) / ripple_rms(r, zero_midclamp) for r in (0.2, 0.35, R_HALF)]
    e3, e2 = simulate(R_HALF, 300, 10.0, False, zero), simulate(R_SQRT3, 300, 10.0, True, zero)
    print(f"  {name:16s} {top:5.2f} {mid:5.2f}   " + " / ".join(f"{x:.2f}" for x in rip)
          + f"   {e3.max():.3f}/{math.sqrt(np.mean(e3 ** 2)):.3f}  {e2.max():.3f}/{math.sqrt(np.mean(e2 ** 2)):.3f}")

# Plot. The general clamp is one ordered family, light to dark as k goes 1/2 -> 0; the ceiling is blue.
# Solid is Vbus/sqrt3, dashed and light is Vbus/2; the columns are the reads
COLORS = {"midclamp": "#eb6834", "clamp k=1/4": "#b54a1d", "bottom clamp": "#7a2f0f", "ceiling sqrt3/2": "#2a78d6"}
LIGHT = 0.5
INK, GRID = "#6b6b6b", "#e4e4e4"
text = {"ceiling sqrt3/2": "ceiling √3/2", "midclamp": "midclamp", "clamp k=1/4": "clamp, k = ¼", "bottom clamp": "bottom clamp",
        "Vbus/2": "Vbus/2", "Vbus/sqrt3": "Vbus/√3"}
family = {"midclamp": "midclamp, k = ½", "clamp k=1/4": "clamp, k = ¼", "bottom clamp": "bottom clamp, k = 0", "ceiling sqrt3/2": "ceiling √3/2"}


def line_style(radius):
    return dict(linestyle="-", alpha=1.0) if radius == "Vbus/sqrt3" else dict(linestyle="--", alpha=LIGHT)


def finish(ax, title, xlabel, ylabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def mark_radii(ax):
    for r, name in ((R_HALF, "Vbus/2"), (R_SQRT3, "Vbus/√3")):
        ax.axvline(r, color=INK, linestyle=":", linewidth=1)
        ax.annotate(name, (r, 1.0), xycoords=("data", "axes fraction"), xytext=(-4, -4), textcoords="offset points",
                    ha="right", va="top", fontsize=8, color=INK)


def plot_family(ax, x, series):
    """The ceiling as a wide light band under the general clamp's lines: it lies on midclamp below r = 0.42 and on the bottom clamp at Vbus/sqrt3"""
    for placement in ("midclamp", "clamp k=1/4", "bottom clamp"):
        ax.plot(x, series[placement], color=COLORS[placement], linewidth=2, zorder=3, label=family[placement])
    ax.plot(x, series["ceiling sqrt3/2"], color=COLORS["ceiling sqrt3/2"], linewidth=6, alpha=0.4, zorder=2, solid_capstyle="butt",
            label=family["ceiling sqrt3/2"])


fig, axes = plt.subplots(4, 2, figsize=(14, 18))
(ax_duty, ax_ripple), (ax_reach3, ax_reach2), (ax_win3, ax_win2), (ax_err3, ax_err2) = axes

for placement in ("midclamp", "ceiling sqrt3/2"):
    for radius in INTEGER_RADII:
        ax_duty.plot(degrees, duty[(placement, radius)][:, 0], color=COLORS[placement], linewidth=2,
                     label=f"{text[placement]}, {text[radius]}", **line_style(radius))
ax_duty.axhline(D_SQRT3_DIV_2, color=INK, linestyle=":", linewidth=1, label="√3/2 ceiling, 86.6 %")
ax_duty.set_xticks(range(0, 361, 60))
ax_duty.set_ylim(-0.02, 1.02)
ax_duty.legend(loc="upper center", fontsize=8)
finish(ax_duty, "Phase A duty over one electrical turn", "Voltage vector angle (electrical degrees)", "Duty")

plot_family(ax_ripple, ripple_radii, ratio)
mark_radii(ax_ripple)
for name in ("bottom clamp", "clamp k=1/4"):
    ax_ripple.annotate(f"{ratio[name][0]:.2f}x", (ripple_radii[0], ratio[name][0]), xytext=(2, 6), textcoords="offset points", fontsize=8)
hump = int(np.argmax(ratio["ceiling sqrt3/2"]))
ax_ripple.annotate(f"{ratio['ceiling sqrt3/2'][hump]:.2f}x", (ripple_radii[hump], ratio["ceiling sqrt3/2"][hump]), xytext=(0, 7),
                   textcoords="offset points", ha="center", fontsize=8)
ax_ripple.annotate(f"ceiling √3/2 on midclamp below r = {(2 * D_SQRT3_DIV_2 - 1) / math.sqrt(3):.2f}", (0.20, 1.0), xytext=(0, 9),
                   textcoords="offset points", fontsize=8, color=INK)
ax_ripple.set_ylim(0.95, 2.05)
ax_ripple.legend(loc="center left", fontsize=8)
finish(ax_ripple, "PWM ripple current, RMS relative to midclamp", "Circle radius r (per Vbus)", "Ripple / midclamp's")

for ax, column, read in ((ax_reach3, 0, "3 phases read: the highest duty's"), (ax_reach2, 1, "2 of 3 read: the middle duty's")):
    plot_family(ax, reach_radii, {placement: windows[column] for placement, windows in reach.items()})
    ax.axhline(3.35, color=INK, linestyle=":", linewidth=1, label="3.35 µs, the Vbus/2 worst case today")
    ax.axhspan(0, EDGE_US, color="black", alpha=0.07, linewidth=0, label=f"< {EDGE_US:.0f} µs, lost to the switching edge")
    mark_radii(ax)
    if column == 0:  # the answer to FOC_V_CIRCLE_PER_V_BUS: the radius at which the ceiling's window falls to today's
        for r, name in ((R_HALF, "midclamp"), (R_CEILING_TODAY, "ceiling sqrt3/2")):
            ax.plot(r, 3.35, marker="o", markersize=7, color=COLORS[name], markeredgecolor="white", markeredgewidth=2, zorder=4)
        ax.annotate(f"3.35 µs: midclamp at {R_HALF}, ceiling √3/2 at {R_CEILING_TODAY:.3f}", (R_CEILING_TODAY, 3.35), xytext=(0.303, 1.6),
                    va="bottom", fontsize=8, arrowprops=dict(arrowstyle="-", color=INK, linewidth=0.8))
    ax.set_ylim(0, 30)
    ax.legend(loc="upper right", bbox_to_anchor=(1.0, 0.94), fontsize=8)
    finish(ax, f"{read} shortest window over a turn, {1e3 / T_US:.0f} kHz", "Circle radius r (per Vbus)", "Low-side window (µs)")

for ax, column, read, where in ((ax_win3, 0, "3 phases read: the highest duty's window", dict(loc="upper right")),
                                (ax_win2, 1, "2 of 3 read: the middle duty's window", dict(loc="lower center", ncol=2))):
    for placement in ("midclamp", "ceiling sqrt3/2"):
        for radius in INTEGER_RADII:
            ax.plot(degrees, window[(placement, radius)][:, column], color=COLORS[placement], linewidth=2,
                    label=f"{text[placement]}, {text[radius]}" + (" (today)" if (placement, radius, column) == ("midclamp", "Vbus/2", 0) else ""),
                    **line_style(radius))
    ax.axhline(3.35, color=INK, linestyle=":", linewidth=1, label="3.35 µs, the Vbus/2 worst case today")
    ax.axhspan(0, EDGE_US, color="black", alpha=0.07, linewidth=0, label=f"< {EDGE_US:.0f} µs, lost to the switching edge")
    ax.set_xticks(range(0, 361, 60))
    ax.set_ylim(0, 12)
    ax.legend(fontsize=8, **where)
    finish(ax, f"{read}, {1e3 / T_US:.0f} kHz", "Voltage vector angle (electrical degrees)", "Low-side window (µs)")

width = 0.19
x = np.arange(len(CASES))
for ax, read in ((ax_err3, "3 phases"), (ax_err2, "2 of 3")):
    for j, (placement, radius) in enumerate((("midclamp", "Vbus/2"), ("ceiling sqrt3/2", "Vbus/2"), ("midclamp", "Vbus/sqrt3"), ("ceiling sqrt3/2", "Vbus/sqrt3"))):
        label = f"{read}, {placement}, {radius}"
        alpha = 1.0 if radius == "Vbus/sqrt3" else LIGHT
        rms = [errors[(label, f_e, tau)][1] for f_e, tau in CASES]
        peak = [errors[(label, f_e, tau)][0] for f_e, tau in CASES]
        today = " (today)" if (read, placement, radius) == ("3 phases", "midclamp", "Vbus/2") else ""
        ax.bar(x + (j - 1.5) * 0.2, rms, width, color=COLORS[placement], alpha=alpha, label=f"{text[placement]}, {text[radius]}{today}")
        ax.plot(x + (j - 1.5) * 0.2, peak, linestyle="none", marker="_", markersize=11, markeredgewidth=2, color=COLORS[placement], alpha=alpha)
    ax.set_xticks(x, [f"{f_e} Hz\nτ {tau:.0f} µs" for f_e, tau in CASES])
    ax.set_ylim(0, 0.62)
    ax.legend(loc="upper left", fontsize=8)
    finish(ax, f"{read} read: track-and-hold error, RMS (bars) and peak (ticks)", "Electrical frequency, hold time constant", "Error / I_peak")

fig.tight_layout()
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "LowSideZeroSequence.png"), dpi=100)
plt.show()
