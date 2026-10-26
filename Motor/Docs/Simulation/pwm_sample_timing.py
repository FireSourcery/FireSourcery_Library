"""
One PWM perKE: FTM2 counter, phase PWM, conducting switch, sample switch, and what each ADC start reads.

Hardware chain, per phase:
    FTM2 CPWM, high-true (ELSnB:ELSnA = 1:0): output high for d*T, centered on CNTIN (counter bottom)
    PWMx high -> IR2104 IN high -> HO: high side conducts; sample switch open: holds
    PWMx low  -> LO, after the turn-on delay: low side conducts; sample switch closed: tracks -i*R_DS(on)
    TOF at MOD -> MOD-1 (counter top): the center of every low-side window, the O000 zero vector
    Init trigger and the duty load point at CNTIN (counter bottom): the center of every high pulse, O111

ADC starts:
    top      today: software start in the TOF ISR [BOARD_PWM_ISR]; VSOURCE, IA, IB, IC in channel order
    bottom   the option: the FTM2 init trigger at CNTIN, SIM_SOPT0[ADHWT] = 010, DELAY = 0
Each conversion samples in its first 3.5 ADCK. ADCK 10 MHz (Board.c), first conversion 23 ADCK + 5 bus cycles,
later ones 20 ADCK. The ISR's entry code before the start is not measured and not included.

The read is the hold's state: the current at the sample if the switch is closed, else at the trailing edge of the
phase's last low-side window. The hold's own time constant is left out. Ripple is the PWM ripple current, in units
of V_bus*T/L, against the period average; by symmetry it crosses the average at both counter extremes.
The plot runs from the CNTIN before the top through the conversions of the CNTIN start after it, so nothing wraps.
Each read sits on its ADC sample line; a hold draws a dotted line back to the trailing edge it froze at.
Writes PwmSampleTiming.png beside this file.
"""
import math
import os

import matplotlib.pyplot as plt
import numpy as np

T_US = 50.0  # 20 kHz
MOD = 1250  # BOARD_PWM_PERIOD_TICKS, the counter's top
EDGE_US = 0.68  # IR2104 turn-on, typ, incl. 520 ns dead time
ADCK_MHZ, BUS_MHZ = 10.0, 25.0
FIRST_US, NEXT_US = 23 / ADCK_MHZ + 5 / BUS_MHZ, 20 / ADCK_MHZ
ORDER = ("VSOURCE", "IA", "IB", "IC")
PHASES = (0.0, 2 * math.pi / 3, 4 * math.pi / 3)


def midclamp(r, theta):
    v = [r * math.cos(theta - p) for p in PHASES]
    z = (max(v) + min(v)) / 2
    return [min(max(0.5 + x - z, 0.0), 1.0) for x in v]


def start_times(t0):
    return {name: t0 + (0.0 if k == 0 else FIRST_US + (k - 1) * NEXT_US) for k, name in enumerate(ORDER)}


def half_window(duty):
    return (1.0 - duty) * T_US / 2  # low-side window about each counter top, t = 0, T, ...


def local(at):
    """Time from the nearest counter top."""
    return (np.asarray(at) + T_US / 2) % T_US - T_US / 2


# Example: Vbus/2, at a line-to-line peak, the shortest window today
d = midclamp(0.5, math.radians(30))
versions = {"top (today)": start_times(0.0), "CNTIN (option)": start_times(T_US / 2)}

# One period about the top, t0 in [-T/2, T/2): PWM state and ripple current
t0 = np.linspace(-T_US / 2, T_US / 2, 20000, endpoint=False)
high0 = np.array([np.abs(t0) >= half_window(x) for x in d], dtype=float)  # 1: PWMx high, the high side
v_n = high0 - high0.mean(axis=0)
ripple0 = np.cumsum(v_n - v_n.mean(axis=1, keepdims=True), axis=1) * (t0[1] - t0[0]) / T_US
ripple0 -= ripple0.mean(axis=1, keepdims=True)


def ripple_at(x, at):
    return np.interp(local(at), t0, ripple0[x], period=T_US)


def read(x, at):
    """(state, captured at, value): tracking inside a window, else the hold of the last trailing edge."""
    lo = half_window(d[x])
    if abs(local(at)) <= lo:
        return "track", at, float(ripple_at(x, at))
    edge = lo + T_US * math.floor((at - lo) / T_US)
    return "hold", edge, float(ripple_at(x, edge))


print(f"Vbus/2 at a line-to-line peak: duties {', '.join(f'{x:.3f}' for x in d)}; ADC: first {FIRST_US:.2f} us, then {NEXT_US:.2f} us each")
print(f"{'phase':5s} {'duty':>5s} {'window':>7s}  " + "   ".join(f"{v:>40s}" for v in versions))
for x, label in enumerate("ABC"):
    cells = []
    for starts in versions.values():
        at = starts["I" + label]
        state, when, value = read(x, at)
        cells.append(f"sample {at:5.2f} us: {state:5s} from {when:6.2f} us, {value:+.4f}")
    print(f"{label:5s} {d[x]:5.3f} {2 * half_window(d[x]):5.2f}us  " + "   ".join(f"{c:>40s}" for c in cells)
          + f"   ripple p-p {float(np.ptp(ripple0[x])):.4f}")
print("value: the read's ripple against the period average, V_bus*T/L")

# Plot, from the CNTIN before the top through the CNTIN start's last conversion
t = np.linspace(-T_US / 2, T_US / 2 + 10.0, 24001)
high = np.array([np.abs(local(t)) >= half_window(x) for x in d])
colors = ("red", "green", "blue")
fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True, gridspec_kw={"height_ratios": [1.3, 1, 1, 1.6]})
ax_cnt, ax_pwm, ax_sw, ax_rip = axes

ax_cnt.plot(t, MOD * (1 - np.abs(local(t)) / (T_US / 2)), color="black")
for x, (label, color) in enumerate(zip("ABC", colors)):
    ax_cnt.axhline(MOD * d[x], color=color, linestyle="--", linewidth=0.8)  # CnV = MOD * d [_PWM_DutyTicksOf]
    ax_cnt.text(-T_US / 2 + 0.3, MOD * d[x] + 15, f"CnV {label} = MOD·{d[x]:.3f}: PWM{label} high while the counter is below", color=color, fontsize=8)
ax_cnt.annotate("TOF, counter top: PWM ISR", (0, MOD), (3, MOD * 1.05), fontsize=8, arrowprops={"arrowstyle": "->"})
for edge in (-T_US / 2, T_US / 2):
    ax_cnt.annotate("CNTIN: init trigger,\nduty load point", (edge, 0), (edge - 9 if edge > 0 else edge + 1, MOD * 0.35), fontsize=8,
                    arrowprops={"arrowstyle": "->"})
ax_cnt.set_ylabel("Counter")
ax_cnt.set_ylim(-60, MOD * 1.2)
ax_cnt.set_title("KE: one PWM period about the counter top, Vbus/2 at a line-to-line peak")

for x, (label, color) in enumerate(zip("ABC", colors)):
    base = (2 - x) * 1.5
    ax_pwm.step(t, base + high[x], where="mid", color=color)
    ax_pwm.text(-T_US / 2 + 0.3, base + 1.1, f"PWM{label}: high = high side, low = low side; shaded, the low side's turn-on delay", color=color, fontsize=8)
    for fall in t[1:][(high[x][:-1] & ~high[x][1:])]:
        ax_pwm.fill_between([fall, fall + EDGE_US], base, base + 1, color=color, alpha=0.25)
ax_pwm.set_yticks([])
ax_pwm.set_ylim(-0.2, 4.4)
ax_pwm.set_ylabel("Gate")

for x, (label, color) in enumerate(zip("ABC", colors)):
    y = 2 - x
    ax_sw.fill_between(t, y - 0.25, y + 0.25, where=~high[x], color=color, alpha=0.6, step="mid")
    ax_sw.fill_between(t, y - 0.25, y + 0.25, where=high[x], color="lightgray", step="mid")
    lo = half_window(d[x])
    ax_sw.text(-lo + 0.3, y - 0.1, f"{label}: track", fontsize=8, color="white")
    ax_sw.text(lo + 0.3, y - 0.1, "hold", fontsize=8)
ax_sw.set_yticks([])
ax_sw.set_ylabel("Sample switch")

for x, (label, color) in enumerate(zip("ABC", colors)):
    ax_rip.plot(t, ripple_at(x, t), color=color, label=f"Ripple {label}")
lines = {"top (today)": "-", "CNTIN (option)": "--"}
markers = {"top (today)": dict(marker="o", markersize=8), "CNTIN (option)": dict(marker="s", markersize=9, markerfacecolor="none")}
for v, starts in versions.items():
    for name, at in starts.items():
        ax_rip.axvline(at, color="gray", linestyle=lines[v], linewidth=0.8)
        ax_rip.text(at + 0.15, 0.97, name, rotation=90, fontsize=7, color="gray", va="top", transform=ax_rip.get_xaxis_transform())
    for x, (label, color) in enumerate(zip("ABC", colors)):
        at = starts["I" + label]
        state, when, value = read(x, at)
        if state == "hold":
            ax_rip.plot([when, at], [value, value], color=color, linestyle=":", linewidth=1.2)
            ax_rip.plot(when, value, marker="|", color=color, markersize=10)
        ax_rip.plot(at, value, color=color, linestyle="none", **markers[v])
ax_rip.axhline(0, color="black", linewidth=0.6)
ax_rip.plot([], [], color="gray", linestyle="-", marker="o", label="ADC sample and read, started at the top (today)")
ax_rip.plot([], [], color="gray", linestyle="--", marker="s", markerfacecolor="none", label="ADC sample and read, started at CNTIN (option)")
ax_rip.plot([], [], color="gray", linestyle=":", marker="|", markersize=10, label="Hold: the read is the value at the window's trailing edge")
ax_rip.set_ylabel("Ripple (V_bus·T/L)")
ax_rip.set_xlabel("Time from the counter top (µs)")
ax_rip.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=8)
for ax in axes:
    ax.grid(alpha=0.4)
ax_rip.set_xlim(t[0], t[-1])

fig.tight_layout()
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "PwmSampleTiming.png"), dpi=100, bbox_inches="tight")
plt.show()
