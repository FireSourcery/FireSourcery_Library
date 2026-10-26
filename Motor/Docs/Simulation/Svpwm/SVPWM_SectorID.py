import os
import numpy as np
import matplotlib.pyplot as plt

def line_to_line(alpha, beta):
    """
    The 3 basic vector magnitudes of svpwm_midclamp_sector; the other 3 of 6 are their inverses.
    In duty scaling they are also the line-to-line duties: X = B - C, Y = A - C, Z = B - A.
    alpha, beta - duty scaling, 1.0 <=> VBus/sqrt3, the linear limit.
    """
    X = beta
    Y = (beta + np.sqrt(3) * alpha) / 2
    Z = (beta - np.sqrt(3) * alpha) / 2
    return X, Y, Z

def sector(X, Y, Z):
    """The sign tree of svpwm_midclamp_sector."""
    return np.where(X >= 0, np.select([Z < 0, Y >= 0], [1, 2], 3), np.select([Z >= 0, Y < 0], [4, 5], 6))

def svpwm_midclamp_sector(alpha, beta):
    """
    Opposite sectors share one case, the same phase mid. The mid phase takes z0, centering max and min on 1/2;
    the other two follow from it by the line-to-line duties.
    """
    X, Y, Z = line_to_line(alpha, beta)
    s = sector(X, Y, Z)
    midB = (1 + X + Z) / 2
    midA = (1 + Y - Z) / 2
    midC = (1 - X - Y) / 2
    return np.clip(np.select([np.isin(s, (1, 4)), np.isin(s, (2, 5)), np.isin(s, (3, 6))], [
        (midB - Z, midB,     midB - X),
        (midA,     midA + Z, midA - Y),
        (midC + Y, midC + X, midC    ),
    ]), 0, 1)

def svpwm_midclamp(vA, vB, vC):
    """The max/min form, svpwm_math.h. vA, vB, vC normalized to VBus."""
    v = np.array([vA, vB, vC])
    return v - (v.max(axis=0) + v.min(axis=0)) / 2 + 0.5

deg = np.linspace(0, 360, 721)
theta = np.deg2rad(deg)
alpha, beta = np.cos(theta), np.sin(theta)  # radius 1.0, the linear limit

X, Y, Z = line_to_line(alpha, beta)
A, B, C = svpwm_midclamp_sector(alpha, beta)

midclamp = svpwm_midclamp(*np.cos([theta, theta - 2 * np.pi / 3, theta + 2 * np.pi / 3]) / np.sqrt(3))  # the same vector as phase voltages
print(f"max |svpwm_midclamp_sector - svpwm_midclamp| = {np.abs([A, B, C] - midclamp).max():.1e}")

fig, (axXYZ, axABC) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
axXYZ.plot(deg, X, label=r'X: $\beta$')
axXYZ.plot(deg, Y, label=r'Y: $\frac{1}{2}(\beta + \sqrt{3}\alpha)$')
axXYZ.plot(deg, Z, label=r'Z: $\frac{1}{2}(\beta - \sqrt{3}\alpha)$')
axXYZ.set_ylabel('Line-to-Line Duty')

axABC.plot(deg, A, label='A', color='red')
axABC.plot(deg, B, label='B', color='green')
axABC.plot(deg, C, label='C', color='blue')
axABC.plot(deg, midclamp.T, label=['svpwm_midclamp', '_', '_'], color='black', linestyle=':')
axABC.set_ylabel('Duty')
axABC.set_xlabel('Angle (deg)')
axABC.set_xticks(np.arange(0, 361, 60))
axABC.set_xlim(0, 360)

centers = np.deg2rad(np.arange(30, 360, 60))
sectorAxis = axXYZ.secondary_xaxis('top')
sectorAxis.set_xticks(np.rad2deg(centers), [f'Sector {s}' for s in sector(*line_to_line(np.cos(centers), np.sin(centers)))])
sectorAxis.tick_params(length=0)

for ax in (axXYZ, axABC):
    ax.grid(True)
    ax.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))

fig.suptitle('SVPWM by Sector: svpwm_midclamp_sector')
fig.tight_layout()
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'SVPWM_SectorID.png'), dpi=100)
plt.show()
