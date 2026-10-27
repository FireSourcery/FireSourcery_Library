import numpy as np
from svpwm_ceiling import INK
from svpwm_blend import (INFLECTIONS, RADIUS_START, RADIUS_END, FW_RADIUS, CASES, k_of_magnitude, k_of_magnitude_squared, plot_text,
                         simulate, VZERO, VZERO_MIDCLAMP, K, DUTY)

T_US = 50.0  # 20 kHz PWM period, for the low-side window (1 - duty) * T

def mark_inflections(ax, labelY=None, color=INK):
    """ Where the blend changes: the ramp's ends on |v| """
    for r, formula, _ in INFLECTIONS:
        ax.axvline(r, color=color, linewidth=0.8, linestyle=":")
        if labelY is not None:
            ax.annotate(plot_text(formula), (r, labelY), xytext=(-3, 0), textcoords="offset points", rotation=90, ha="right", va="center", fontsize=8, color=INK)

# Over a cycle, against the vector magnitude. The inflections are in the grid, so the corners are exact
radii = np.unique(np.concatenate([np.linspace(0.0, 1 / np.sqrt(3), 401), [r for r, _, _ in INFLECTIONS], [FW_RADIUS]]))
cycle = np.linspace(0, 1, 360, endpoint=False)  # whole degrees: each phase's peak and each line-to-line peak included
def sweep(k_of):
    """ Per radius, over one cycle: the highest duty's min and max, the middle duty's max, the lowest duty's min and max, k's min and max,
        vZero above midclamp's, min and max """
    rows = []
    for r in radii:
        run = simulate(r, k_of, cycle)
        duties = np.sort(run[:, DUTY], axis=1)
        shift = run[:, VZERO] - run[:, VZERO_MIDCLAMP]
        rows.append((duties[:, 2].min(), duties[:, 2].max(), duties[:, 1].max(), duties[:, 0].min(), duties[:, 0].max(),
                     run[:, K].min(), run[:, K].max(), shift.min(), shift.max()))
    return np.array(rows)
blend = sweep(k_of_magnitude)
magnitudeSquared = sweep(k_of_magnitude_squared)
midclamp = sweep(lambda vA, vB, vC: 0.5)
HIGHEST_MIN, HIGHEST_MAX, MIDDLE_MAX, LOWEST_MIN, LOWEST_MAX, K_MIN, K_MAX, SHIFT_MIN, SHIFT_MAX = range(9)

if __name__ == "__main__":
    print("Inflections:")
    for r, formula, what in INFLECTIONS:
        print(f"  r = {formula:13s} = {r:.4f} VBus   {what}")

    inside = radii <= RADIUS_END + 1e-12
    windowToday = (1 - midclamp[np.argmin(np.abs(radii - RADIUS_START)), HIGHEST_MAX]) * T_US  # the window midclamp leaves at VBus/2, 3.35 us
    def keeps_window(run):
        return np.all((1 - run[inside, HIGHEST_MAX]) * T_US >= windowToday - 1e-9)
    print("  checks:"
          f"\n    one k over each cycle: {np.allclose(blend[:, K_MIN], blend[:, K_MAX])}"
          f"\n    |v|^2 form, no sqrt: max |k - the |v| blend's| {np.abs(magnitudeSquared[:, K_MIN] - blend[:, K_MIN]).max():.4f}"
          f"\n    the 3 phases read window at or over today's {windowToday:.2f} us up to {RADIUS_END:.3f}: |v| blend {keeps_window(blend)}; |v|^2 form {keeps_window(magnitudeSquared)}"
          f"\n    the middle duty's peak = the highest duty's lowest, every placement: {all(np.allclose(s[:, MIDDLE_MAX], s[:, HIGHEST_MIN]) for s in (blend, magnitudeSquared, midclamp))}")

    print(f"  {'radius':8s}{'placement':11s}{'k':7s}highest duty  middle duty   windows us, 3 phases / 2 of 3")
    for r in CASES:
        i = int(np.argmin(np.abs(radii - r)))
        for name, run in (("midclamp", midclamp), ("|v| blend", blend)):
            print(f"  {radii[i]:.4f}  {name:11s}{run[i, K_MIN]:.3f}  {run[i, HIGHEST_MAX]:.3f}         {run[i, MIDDLE_MAX]:.3f}"
                  f"         {(1 - run[i, HIGHEST_MAX]) * T_US:4.2f} / {(1 - run[i, MIDDLE_MAX]) * T_US:4.2f}")
