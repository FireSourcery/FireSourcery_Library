import numpy as np
from svpwm_3phase_ceiling import DUTY_CEILING, simulate, regime_of, SPREAD_CEILING, SPREAD_BOTTOM, INFLECTIONS, REGIME_COLORS, INK

T_US = 50.0                                     # 20 kHz PWM period, for the low-side window (1 - duty) * T
MIDCLAMP, CEILING = REGIME_COLORS[0], "#6250d6" # by algorithm: svpwm_midclamp, the midclamp branch's; svpwm_midclamp_ceiling

def share_beyond(spread, radius):
    """ The share of a cycle whose spread exceeds spread. The spread is sqrt3*r*cos(phi), phi within 30 degrees of a line-to-line peak """
    return np.clip(np.arccos(np.clip(spread / (np.sqrt(3) * np.maximum(radius, 1e-12)), -1, 1)) / (np.pi / 6), 0, 1)

def mark_inflections(ax, labelY=None, color=INK):
    for r, formula, name in INFLECTIONS:
        ax.axvline(r, color=color, linewidth=0.8, linestyle=":")
        if labelY is not None:
            ax.annotate(f"r = {r:.3f}: {name}", (r, labelY), xytext=(-3, 0), textcoords="offset points",
                        rotation=90, ha="right", va="center", fontsize=8, color=INK)

# Over a cycle, against the vector magnitude. The inflections are in the grid, so the corners are exact
radii = np.unique(np.concatenate([np.linspace(0.0, 1 / np.sqrt(3), 401), [r for r, _, _ in INFLECTIONS]]))
cycle = np.linspace(0, 1, 360, endpoint=False)  # whole degrees, the line-to-line peaks included
def sweep(dutyCeiling):
    """ Per radius, over one cycle: the highest duty's min and max, the middle duty's max, the lowest duty's min and max, the spread's min and max, the share of each branch """
    rows = []
    for r in radii:
        run = simulate(r, dutyCeiling, cycle)
        duties = np.sort(run[:, 5:8], axis=1)
        spread = run[:, 2:5].max(axis=1) - run[:, 2:5].min(axis=1)
        share = np.bincount(regime_of(run, dutyCeiling), minlength=3) / len(run)
        rows.append((duties[:, 2].min(), duties[:, 2].max(), duties[:, 1].max(), duties[:, 0].min(), duties[:, 0].max(), spread.min(), spread.max(), *share))
    return np.array(rows)
ceiling = sweep(DUTY_CEILING)
midclamp = sweep(1.0)  # A ceiling of 1 never holds inside the linear range: midclamp
HIGHEST_MIN, HIGHEST_MAX, MIDDLE_MAX, LOWEST_MIN, LOWEST_MAX, SPREAD_MIN, SPREAD_MAX, SHARE = range(8)

beyondCeiling, beyondBottom = share_beyond(SPREAD_CEILING, radii), share_beyond(SPREAD_BOTTOM, radii)
share = np.column_stack((1 - beyondCeiling, beyondCeiling - beyondBottom, beyondBottom))  # midclamp, the ceiling, the bottom clamp

if __name__ == "__main__":
    print(f"Duty ceiling D = {DUTY_CEILING:.4f}. The spread vMax - vMin over a cycle runs from 1.5*r to sqrt3*r; the branch changes at:")
    for r, formula, name in INFLECTIONS:
        print(f"  r = {formula:15s} = {r:.4f} VBus   {name}")

    print("  checks: spread over a cycle = [1.5*r, sqrt3*r]: "
          f"{np.allclose(ceiling[:, SPREAD_MIN], 1.5 * radii) and np.allclose(ceiling[:, SPREAD_MAX], np.sqrt(3) * radii)};"
          f" the middle duty's peak = the highest duty's lowest, both placements: {np.allclose(ceiling[:, MIDDLE_MAX], ceiling[:, HIGHEST_MIN]) and np.allclose(midclamp[:, MIDDLE_MAX], midclamp[:, HIGHEST_MIN])};"
          f" branch shares, exact vs sampled: max |diff| {np.abs(share - ceiling[:, SHARE:]).max():.4f},"
          f" within a sample per arc, 6 arcs a cycle: {np.abs(share - ceiling[:, SHARE:]).max() <= 6 / len(cycle) + 1e-12}")

    print("  radius   highest duty: midclamp ceiling   middle duty: midclamp ceiling   lowest duty: midclamp ceiling")
    for r in (0.3, *(r for r, _, _ in INFLECTIONS)):
        i = int(np.argmin(np.abs(radii - r)))
        print(f"  {radii[i]:.4f}                 {midclamp[i, HIGHEST_MAX]:.3f}   {ceiling[i, HIGHEST_MAX]:.3f}"
              f"                  {midclamp[i, MIDDLE_MAX]:.3f}   {ceiling[i, MIDDLE_MAX]:.3f}"
              f"                  {midclamp[i, LOWEST_MIN]:.3f}   {ceiling[i, LOWEST_MIN]:.3f}")
