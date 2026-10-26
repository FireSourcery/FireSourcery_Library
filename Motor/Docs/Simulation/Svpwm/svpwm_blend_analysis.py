import numpy as np
from svpwm_3phase_ceiling import INK
from svpwm_3phase_ceiling_analysis import share_beyond
from svpwm_3phase_blend import BLENDS, INFLECTIONS, SPREAD_START, FW_RADIUS, simulate, regime_of, VZERO, VZERO_MIDCLAMP, K, PHASE_V, DUTY

T_US = 50.0  # 20 kHz PWM period, for the low-side window (1 - duty) * T

def mark_inflections(ax, labelY=None, color=INK):
    """ VBus/2 and VBus/sqrt3: both blends change there, the duty-max blend's radii are the same two """
    for r, formula, _ in INFLECTIONS["|v| blend"]:
        ax.axvline(r, color=color, linewidth=0.8, linestyle=":")
        if labelY is not None:
            ax.annotate(formula.replace("sqrt3", "√3"), (r, labelY), xytext=(-3, 0), textcoords="offset points", rotation=90, ha="right", va="center", fontsize=8, color=INK)

# Over a cycle, against the vector magnitude. The inflections are in the grid, so the corners are exact
radii = np.unique(np.concatenate([np.linspace(0.0, 1 / np.sqrt(3), 401), [r for r, _, _ in INFLECTIONS["|v| blend"]], [FW_RADIUS]]))
cycle = np.linspace(0, 1, 360, endpoint=False)  # whole degrees: each phase's peak and each line-to-line peak included
def sweep(k_of):
    """ Per radius, over one cycle: the highest duty's min and max, the middle duty's max, the lowest duty's min and max, k's min and max,
        vZero above midclamp's, min and max, and the share of each regime """
    rows = []
    for r in radii:
        run = simulate(r, k_of, cycle)
        duties = np.sort(run[:, DUTY], axis=1)
        shift = run[:, VZERO] - run[:, VZERO_MIDCLAMP]
        share = np.bincount(regime_of(run), minlength=3) / len(run)
        rows.append((duties[:, 2].min(), duties[:, 2].max(), duties[:, 1].max(), duties[:, 0].min(), duties[:, 0].max(),
                     run[:, K].min(), run[:, K].max(), shift.min(), shift.max(), *share))
    return np.array(rows)
magnitude = sweep(BLENDS["|v| blend"])
dutyMax = sweep(BLENDS["duty-max blend"])
midclamp = sweep(lambda vA, vB, vC: 0.5)
HIGHEST_MIN, HIGHEST_MAX, MIDDLE_MAX, LOWEST_MIN, LOWEST_MAX, K_MIN, K_MAX, SHIFT_MIN, SHIFT_MAX, SHARE = range(10)

# The duty-max blend leaves midclamp over the part of the cycle whose spread exceeds sqrt3/2. k reaches 0 only at the line-to-line peaks at VBus/sqrt3
offMidclampShare = share_beyond(SPREAD_START, radii)

if __name__ == "__main__":
    print("Inflections, the same two radii for both blends:")
    for name, inflections in INFLECTIONS.items():
        for r, formula, what in inflections:
            print(f"  {name:15s} r = {formula:8s} = {r:.4f} VBus   {what}")

    # Per vector, the duty-max blend's lowest duty is min(n/2, (2 + sqrt3)*n^2), n = 1 - spread the null time
    closedForm = 0.0
    for r in radii[::20]:
        run = simulate(r, BLENDS["duty-max blend"], cycle)
        n = 1 - np.ptp(run[:, PHASE_V], axis=1)
        closedForm = max(closedForm, np.abs(np.sort(run[:, DUTY], axis=1)[:, 0] - np.minimum(n / 2, (2 + np.sqrt(3)) * n * n)).max())

    print("  checks:"
          f" at each line-to-line peak the duty-max blend's k is the |v| blend's: {np.allclose(dutyMax[:, K_MIN], magnitude[:, K_MIN])};"
          f" so the highest duty's peak, the 3 phases read window, is the same: {np.allclose(dutyMax[:, HIGHEST_MAX], magnitude[:, HIGHEST_MAX])};"
          f"\n          the duty-max blend's middle duty's peak, the 2 of 3 read window, is midclamp's: {np.allclose(dutyMax[:, MIDDLE_MAX], midclamp[:, MIDDLE_MAX])};"
          f" the middle duty's peak = the highest duty's lowest, all three: {all(np.allclose(s[:, MIDDLE_MAX], s[:, HIGHEST_MIN]) for s in (magnitude, dutyMax, midclamp))};"
          f"\n          its lowest duty = min(n/2, (2 + sqrt3)*n^2): max |diff| {closedForm:.1e};"
          f" its share off midclamp, exact vs sampled: max |diff| {np.abs(offMidclampShare - (1 - dutyMax[:, SHARE])).max():.4f},"
          f" within a sample per arc, 6 arcs a cycle: {np.abs(offMidclampShare - (1 - dutyMax[:, SHARE])).max() <= 6 / len(cycle) + 1e-12}")

    print("  radius    k: |v|  duty max     highest duty: midclamp |v| duty max   middle duty: midclamp |v| duty max   windows us, 3 phases / 2 of 3: |v|  duty max")
    for r in (0.45, 0.5, 0.52, FW_RADIUS, 0.56, 1 / np.sqrt(3)):
        i = int(np.argmin(np.abs(radii - r)))
        print(f"  {radii[i]:.4f}     {magnitude[i, K_MIN]:.3f}  {dutyMax[i, K_MIN]:.3f}-{dutyMax[i, K_MAX]:.3f}"
              f"                {midclamp[i, HIGHEST_MAX]:.3f} {magnitude[i, HIGHEST_MAX]:.3f}    {dutyMax[i, HIGHEST_MAX]:.3f}"
              f"                {midclamp[i, MIDDLE_MAX]:.3f} {magnitude[i, MIDDLE_MAX]:.3f}    {dutyMax[i, MIDDLE_MAX]:.3f}"
              f"                     {(1 - magnitude[i, HIGHEST_MAX]) * T_US:4.2f} / {(1 - magnitude[i, MIDDLE_MAX]) * T_US:4.2f}"
              f"  {(1 - dutyMax[i, HIGHEST_MAX]) * T_US:4.2f} / {(1 - dutyMax[i, MIDDLE_MAX]) * T_US:4.2f}")
