"""
The sensorless math as geometry: one PNG per component, for Motor/Docs/SensorlessMath.md.

    SensorlessVoltagePolygon.png    the stator equation as a closed polygon, seen from θ and from θ̂
    SensorlessFrames.png            Clarke as projection, Park as a turned frame, the hold angle as an arc's mean
    SensorlessSmo.png               sat and its balance point, the error map's cobwebs, the equivalent control
    SensorlessLpf.png               the LPF as a geometric series and as a circle
    SensorlessDetector.png          the rotor vector, the error as a parallelogram, the S-curve
    SensorlessPll.png               the loop as a pendulum on a washboard, its phase portrait, a speed step

Values are illustrative per unit unless named from EMF_OBSERVER_CONFIG_DEFAULT. Run: MPLBACKEND=Agg python sensorless_geometry.py
"""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, FancyArrowPatch, Polygon, Wedge

import sensorless_model as m

HERE = os.path.dirname(os.path.abspath(__file__))

# Reference palette, categorical slots 1-3 validated all-pairs; aqua below 3:1, so every mark is labeled
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#9b9a94"
RULE = "#d6d5cf"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
BLUE_LIGHT = "#cde2fb"
AQUA_LIGHT = "#c9eedd"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": RULE, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "font.size": 9.5, "axes.titlesize": 10.5, "axes.titleweight": "bold",
    "axes.titlelocation": "left", "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.6, "grid.color": RULE, "grid.linewidth": 0.6,
    "legend.frameon": False, "legend.fontsize": 8.5,
})

CONTROL_FREQ = 10000.0  # the example control cycle, 100 us

def save(fig, name):
    fig.savefig(os.path.join(HERE, name), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote", name)

# ---------------------------------------------------------------------------
# Drawing primitives
# ---------------------------------------------------------------------------
def arrow(ax, p0, p1, color, lw=1.8, style="-|>", ls="-", z=3):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=11, color=color, lw=lw, linestyle=ls, zorder=z, shrinkA=0, shrinkB=0))

def label(ax, p, text, color=INK, ha="center", va="center", size=9.5, **kw):
    ax.text(p[0], p[1], text, color=color, ha=ha, va=va, fontsize=size, zorder=6, **kw)

def frame(ax, angle, length, names, color=MUTED, ls=(0, (4, 3))):
    """Two perpendicular axes at [angle], named at their tips"""
    for k, name in enumerate(names):
        a = angle + k * math.pi / 2
        tip = (length * math.cos(a), length * math.sin(a))
        ax.plot([-0.12 * tip[0], tip[0]], [-0.12 * tip[1], tip[1]], color=color, lw=0.9, ls=ls, zorder=1)
        label(ax, (1.08 * tip[0], 1.08 * tip[1]), name, color=INK2)

def axes_dq(ax, x0, x1, y0, y1, names=("d", "q"), color=MUTED):
    ax.plot([x0, x1], [0, 0], color=color, lw=0.9, ls=(0, (4, 3)), zorder=1)
    ax.plot([0, 0], [y0, y1], color=color, lw=0.9, ls=(0, (4, 3)), zorder=1)
    label(ax, (x1 + 0.05, 0), names[0], color=INK2, ha="left")
    label(ax, (0, y1 + 0.05), names[1], color=INK2, va="bottom")

def angle_arc(ax, a0, a1, r, color=INK2, text=None, text_r=None):
    lo, hi = sorted((math.degrees(a0), math.degrees(a1)))
    ax.add_patch(Arc((0, 0), 2 * r, 2 * r, theta1=lo, theta2=hi, color=color, lw=1.0, zorder=2))
    if text:
        mid = (a0 + a1) / 2
        tr = text_r or r * 1.18
        label(ax, (tr * math.cos(mid), tr * math.sin(mid)), text, color=color, size=8.5)

def right_angle(ax, corner, u, v, s=0.045, color=INK2):
    """A square at [corner] between unit directions u and v"""
    p1 = (corner[0] + s * u[0], corner[1] + s * u[1])
    p2 = (p1[0] + s * v[0], p1[1] + s * v[1])
    p3 = (corner[0] + s * v[0], corner[1] + s * v[1])
    ax.plot([p1[0], p2[0], p3[0]], [p1[1], p2[1], p3[1]], color=color, lw=0.9, zorder=2)

def unit(a):
    return (math.cos(a), math.sin(a))

def add(p, q):
    return (p[0] + q[0], p[1] + q[1])

def scale(k, p):
    return (k * p[0], k * p[1])

def norm(p):
    l = math.hypot(*p)
    return (p[0] / l, p[1] / l)

def square(ax, lim):
    ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_aspect("equal"); ax.axis("off")

def caption(ax, p, text):
    label(ax, p, text, color=INK2, size=8.5)

# ---------------------------------------------------------------------------
# 1. The stator equation as a closed polygon
# ---------------------------------------------------------------------------
POLY = dict(R=0.3, X=0.75, E=0.9, i=(-0.12, 0.75))

def polygon_points():
    R, X, E, i = POLY["R"], POLY["X"], POLY["E"], POLY["i"]
    p1 = scale(R, i)
    p2 = add(p1, scale(X, m.J(i)))
    v = add(p2, (0.0, E))
    return i, p1, p2, v

def draw_polygon(ax, e_name):
    i, p1, p2, v = polygon_points()
    axes_dq(ax, -0.95, 0.5, -0.1, 1.2)
    arrow(ax, (0, 0), i, MUTED, lw=1.2, ls=(0, (3, 2)))
    label(ax, add(i, (-0.07, 0.0)), "i", color=INK2)
    arrow(ax, (0, 0), p1, BLUE)
    label(ax, (0.05, 0.12), "R·i", color=BLUE, ha="left")
    arrow(ax, p1, p2, ORANGE)
    label(ax, (-0.32, 0.05), "X·J·i,  X = ω·L", color=ORANGE)
    arrow(ax, p2, v, AQUA)
    label(ax, (p2[0] - 0.05, 0.6), e_name, color=AQUA, ha="right")
    right_angle(ax, p1, norm(scale(-1, p1)), norm((p2[0] - p1[0], p2[1] - p1[1])))
    return i, p1, p2, v

def fig_voltage_polygon():
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.6))

    # (a) the rotor frame, steady state
    ax = axs[0]
    i, p1, p2, v = draw_polygon(ax, "e = ω·ψ, on q")
    arrow(ax, (0, 0), v, INK, lw=2.2)
    label(ax, (-0.22, 0.6), "v", color=INK, size=11)
    ax.set_title("(a) v = R·i + X·J·i + e: a closed polygon")
    caption(ax, (-0.22, -0.35), "Steady state, L·di/dt = 0. J is the quarter turn: X·J·i ⊥ i.\nThe EEMF model drives each axis with v − X·J·i,\nthe frame's turn on the measured i.")
    square(ax, ((-1.05, 0.62), (-0.55, 1.32)))

    # (b) the same EMF from the estimated frame θ̂ = θ − Δ
    ax = axs[1]
    E = POLY["E"]
    D = math.radians(25)
    frame(ax, 0.0, 1.05, ["d", "q"], color=RULE)
    frame(ax, -D, 1.05, ["d̂", "q̂"])
    e = (0.0, E)
    arrow(ax, (0, 0), e, AQUA)
    label(ax, (-0.05, 0.6), "e", color=AQUA, ha="right")
    dh, qh = unit(-D), unit(-D + math.pi / 2)
    foot = scale(-E * math.sin(D), dh)
    footq = scale(E * math.cos(D), qh)
    ax.plot([e[0], foot[0]], [e[1], foot[1]], color=MUTED, lw=0.9, ls=(0, (2, 2)))
    ax.plot([e[0], footq[0]], [e[1], footq[1]], color=MUTED, lw=0.9, ls=(0, (2, 2)))
    arrow(ax, (0, 0), foot, ORANGE)
    label(ax, add(foot, (-0.04, 0.12)), "ê_d = −E·sin(θ − θ̂)", color=ORANGE, ha="right")
    ax.plot([0, footq[0]], [0, footq[1]], color=INK2, lw=2.4, alpha=0.35, solid_capstyle="round")
    label(ax, add(footq, (0.06, -0.06)), "ê_q = E·cos(θ − θ̂)", color=INK2, ha="left")
    arrow(ax, (0, 0), (E, 0.0), BLUE)
    label(ax, (0.62, 0.08), "r = −j·ê", color=BLUE)
    angle_arc(ax, -D, 0.0, 0.3, text="θ − θ̂", text_r=0.47)
    ax.set_title("(b) seen from θ̂: the EMF tilts by θ − θ̂")
    caption(ax, (0.18, -0.68), "The d̂ part is the angle error.\nA quarter turn back, r = −j·ê points along the rotor.")
    square(ax, ((-0.95, 1.4), (-0.85, 1.3)))

    # (c) the polygon that does not close: the gap is L·Δi, the model's step
    ax = axs[2]
    i, p1, p2, v = draw_polygon(ax, "z, in place of e")
    gap = (0.15, 0.10)
    vt = add(v, gap)
    arrow(ax, (0, 0), vt, INK, lw=2.2)
    label(ax, (-0.27, 0.5), "v", color=INK, size=11)
    arrow(ax, v, vt, INK2, lw=1.4, ls=(0, (3, 2)))
    label(ax, add(v, (-0.02, 0.2)), "gap = v − R·i − X·J·i − z = L·Δi", color=INK2, size=8.5)
    G = 0.9
    ni = add(i, scale(G, gap))
    arrow(ax, i, ni, BLUE, lw=1.6)
    label(ax, add(ni, (0.04, 0.05)), "î[k+1] = î[k] + G·gap", color=BLUE, ha="left", size=8.5)
    ax.set_title("(c) when it does not close, the current steps")
    caption(ax, (-0.22, -0.35), "stator_i_next moves î along the gap, G = 1/L per control\ncycle (drawn large). z grows until î follows i:\nthe polygon closes, and z = e.")
    square(ax, ((-1.05, 0.62), (-0.55, 1.32)))
    save(fig, "SensorlessVoltagePolygon.png")

# ---------------------------------------------------------------------------
# 2. Frames
# ---------------------------------------------------------------------------
def fig_frames():
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.2))
    phi = math.radians(35)
    s = unit(phi)

    # (a) Clarke: each phase reads the projection on its axis
    ax = axs[0]
    ax.add_patch(Circle((0, 0), 1.0, fill=False, color=RULE, lw=0.8))
    for k, name in enumerate(("a, α", "b", "c")):
        a = k * 2 * math.pi / 3
        u = unit(a)
        ax.plot([-1.1 * u[0], 1.1 * u[0]], [-1.1 * u[1], 1.1 * u[1]], color=MUTED, lw=0.9, ls=(0, (4, 3)))
        label(ax, scale(1.22, u), name, color=INK2)
        proj = math.cos(phi - a)
        foot = scale(proj, u)
        ax.plot([s[0], foot[0]], [s[1], foot[1]], color=MUTED, lw=0.8, ls=(0, (2, 2)))
        ax.plot([0, foot[0]], [0, foot[1]], color=ORANGE, lw=3.0, solid_capstyle="round", zorder=2)
        out = add(foot, add(scale(0.2 * (1 if proj >= 0 else -1), u), scale(0.1, unit(a + math.pi / 2))))
        label(ax, out, "%s %+.2f" % (name[0], proj), color=ORANGE, size=8.5)
    ax.plot([0, 0], [0, 1.1], color=RULE, lw=0.9)
    label(ax, (0, 1.2), "β", color=INK2)
    arrow(ax, (0, 0), s, BLUE, lw=2.0)
    label(ax, add(s, (0.08, 0.06)), "i", color=BLUE)
    ax.set_title("(a) Clarke: three projections of one vector")
    caption(ax, (0, -1.45), "a, b, c are i's shadows on axes 120° apart: a + b + c = 0.\nfoc_clarke reads the vector back from them.")
    square(ax, ((-1.35, 1.35), (-1.65, 1.35)))

    # (b) Park: the same vector from a frame turned by θ̂
    ax = axs[1]
    th = math.radians(20)
    ax.add_patch(Circle((0, 0), 1.0, fill=False, color=RULE, lw=0.8))
    frame(ax, 0.0, 1.15, ["α", "β"], color=RULE)
    frame(ax, th, 1.15, ["d̂", "q̂"])
    arrow(ax, (0, 0), s, BLUE, lw=2.0)
    label(ax, add(s, (0.08, 0.06)), "i", color=BLUE)
    fd = scale(math.cos(phi - th), unit(th))
    fq = scale(math.sin(phi - th), unit(th + math.pi / 2))
    ax.plot([s[0], fd[0]], [s[1], fd[1]], color=MUTED, lw=0.8, ls=(0, (2, 2)))
    ax.plot([s[0], fq[0]], [s[1], fq[1]], color=MUTED, lw=0.8, ls=(0, (2, 2)))
    ax.plot([0, fd[0]], [0, fd[1]], color=ORANGE, lw=3.0, solid_capstyle="round")
    ax.plot([0, fq[0]], [0, fq[1]], color=AQUA, lw=3.0, solid_capstyle="round")
    label(ax, add(fd, (0.04, -0.13)), "i_d", color=ORANGE)
    label(ax, add(fq, (-0.12, 0.03)), "i_q", color=AQUA)
    angle_arc(ax, 0.0, th, 0.35, text="θ̂", text_r=0.47)
    ax.set_title("(b) Park: the same vector, the frame turned by θ̂")
    caption(ax, (0, -1.45), "foc_park turns by −θ̂. At θ̂ = θ the EMF\nsits on q̂, and every dq quantity is DC.")
    square(ax, ((-1.35, 1.35), (-1.65, 1.35)))

    # (c) the hold angle: a fixed command seen from a turning frame sweeps an arc
    ax = axs[2]
    D = math.radians(50)
    end = math.radians(70)
    start = end + D
    ax.add_patch(Circle((0, 0), 1.0, fill=False, color=RULE, lw=0.8))
    frame(ax, 0.0, 1.15, ["d̂ at θ̂", "q̂"], color=RULE)
    ax.add_patch(Arc((0, 0), 2, 2, theta1=math.degrees(end), theta2=math.degrees(start), color=BLUE, lw=3.0))
    arrow(ax, (0, 0), unit(start), MUTED, lw=1.2)
    arrow(ax, (0, 0), unit(end), MUTED, lw=1.2)
    label(ax, add(scale(1.08, unit(start)), (-0.2, 0.08)), "the period's start", color=INK2, size=8.5)
    label(ax, add(scale(1.08, unit(end)), (0.3, 0.06)), "the sample", color=INK2, size=8.5)
    mid = end + D / 2
    k = math.sin(D / 2) / (D / 2)
    arrow(ax, (0, 0), scale(k, unit(mid)), ORANGE, lw=2.0)
    ax.plot([math.cos(start), math.cos(end)], [math.sin(start), math.sin(end)], color=MUTED, lw=0.8, ls=(0, (2, 2)))
    label(ax, (-0.42, 0.52), "the mean,\nat θ̂ − Δθ̂/2", color=ORANGE, size=8.5)
    angle_arc(ax, end, start, 0.3, text="Δθ̂", text_r=0.44)
    ax.set_title("(c) the hold angle: the mean of an arc")
    caption(ax, (0, -0.78), "The command holds in αβ while the frame turns Δθ̂ (drawn 50°).\nSeen from the frame it sweeps the arc. Its mean lies on the\nbisector, |mean| = sinc(Δθ̂/2) ≈ 1: Park the held v at θ̂ − Δθ̂/2.")
    square(ax, ((-1.35, 1.35), (-1.05, 1.45)))
    save(fig, "SensorlessFrames.png")

# ---------------------------------------------------------------------------
# 3. SMO
# ---------------------------------------------------------------------------
SMO = dict(r=0.6, k=0.115, thr=0.048, e=0.05)

def fig_smo():
    r, k, thr, e = SMO["r"], SMO["k"], SMO["thr"], SMO["e"]
    fig = plt.figure(figsize=(14, 8.8))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.0], hspace=0.4, wspace=0.28)

    # (a) the switch and its balance point
    ax = fig.add_subplot(gs[0, 0:2])
    xs = [x / 1000.0 for x in range(-160, 161)]
    ax.axvspan(-thr, thr, color=BLUE_LIGHT, alpha=0.55, lw=0)
    label(ax, (0.0, -0.135), "boundary layer, |σ| < SmoSat", color=INK2, size=8.5)
    ax.plot(xs, [m.smo_z(k, thr, x) for x in xs], color=BLUE, lw=2.0)
    label(ax, (0.155, k - 0.014), "z = K_smo·sat(σ / SmoSat)", color=BLUE, ha="right")
    ax.plot(xs, [e - r * x for x in xs], color=AQUA, lw=1.6)
    label(ax, (-0.152, 0.098), "e − R·σ: the EMF to balance", color=AQUA, ha="left")
    s_star = m.smo_fixed_point(r, k, thr, e)
    z_star = m.smo_z(k, thr, s_star)
    ax.plot([s_star], [z_star], "o", color=ORANGE, ms=7, zorder=5)
    ax.annotate("σ* = e / (R + K_smo/SmoSat)\nthe model settles off the current,\nî − i = σ*, z = e − R·σ*", (s_star, z_star), (0.06, 0.015),
                color=ORANGE, fontsize=8.5, arrowprops=dict(arrowstyle="-", color=ORANGE, lw=0.8))
    ax.axhline(0.165, color=MUTED, lw=1.0, ls=(0, (4, 3)))
    label(ax, (-0.152, 0.176), "an EMF above K_smo: no balance point, the model loses the current", color=INK2, ha="left", size=8.5)
    ax.axhline(0, color=RULE, lw=0.8); ax.axvline(0, color=RULE, lw=0.8)
    ax.set_xlabel("σ = î − i"); ax.set_ylabel("z")
    ax.set_xlim(-0.16, 0.16); ax.set_ylim(-0.16, 0.19)
    ax.set_title("(a) the switch: z balances the EMF where the lines cross")

    # (c) the equivalent control: the chatter's mean
    ax = fig.add_subplot(gs[0, 2])
    g = 0.80
    a = m.lpf_coef(CONTROL_FREQ)
    sig, emf, zs, es, ss = 0.12, 0.0, [], [], []
    for _ in range(140):
        z = m.smo_z(k, thr, sig)
        emf = m.lpf_step(a, emf, z)
        zs.append(z); es.append(emf); ss.append(sig)
        sig = m.smo_error_map(sig, g, r, k, thr, e)
    tail = slice(60, None)
    z_mean = sum(zs[tail]) / len(zs[tail])
    s_mean = sum(ss[tail]) / len(ss[tail])
    t = list(range(len(zs)))
    ax.step(t, zs, where="post", color=ORANGE, lw=0.8, alpha=0.7, label="z, the switch")
    ax.plot(t, es, color=AQUA, lw=2.2, label="ê = LPF(z)")
    ax.axhline(e, color=INK, lw=1.0, ls=(0, (4, 3)), label="e")
    ax.axhline(z_mean, color=AQUA, lw=0.9, ls=(0, (2, 2)), label="mean z = e − R·σ̄")
    ax.legend(loc="lower center", ncol=2)
    ax.set_xlabel("control cycles"); ax.set_ylim(-0.17, 0.14)
    ax.set_title("(c) G %.2f: the chatter's mean is the balance" % g)

    # (b) the error map, three step sizes
    for col, g in enumerate((0.09, 0.45, 0.80)):
        ax = fig.add_subplot(gs[1, col])
        p = m.smo_pole(g, r, k, thr)
        orbit = [0.12]
        for _ in range(60):
            orbit.append(m.smo_error_map(orbit[-1], g, r, k, thr, e))
        lo, hi = min(orbit + [-0.02]) - 0.02, max(orbit) + 0.02
        xs = [lo + (hi - lo) * n / 300.0 for n in range(301)]
        ax.axvspan(-thr, thr, color=BLUE_LIGHT, alpha=0.45, lw=0)
        ax.plot(xs, xs, color=MUTED, lw=1.0)
        ax.plot(xs, [m.smo_error_map(x, g, r, k, thr, e) for x in xs], color=BLUE, lw=2.0)
        px, py = [orbit[0]], [orbit[0]]
        for n in range(len(orbit) - 1):
            px += [orbit[n], orbit[n + 1]]
            py += [orbit[n + 1], orbit[n + 1]]
        ax.plot(px, py, color=ORANGE, lw=0.8)
        ax.plot([orbit[0]], [orbit[0]], "o", color=ORANGE, ms=4)
        kind = "a staircase: settles" if p > 0 else ("a zigzag: settles" if p > -1 else "an orbit across the layer: chatter")
        ax.set_title("(b%d) G = %.2f, pole %+.2f" % (col + 1, g, p))
        label(ax, (lo + 0.04 * (hi - lo), hi - 0.07 * (hi - lo)), kind, color=INK2, ha="left", size=8.5)
        ax.set_xlim(lo, hi); ax.set_ylim(lo, hi); ax.set_aspect("equal")
        ax.set_xlabel("σ[k]"); ax.set_ylabel("σ[k+1]")
    fig.text(0.07, 0.03, "The error map σ[k+1] = (1 − G·R)·σ − G·z(σ) + G·e (blue) against σ[k+1] = σ[k] (gray). The cobweb (orange) is the orbit; "
             "its slope in the layer is the pole 1 − G·(R + K_smo/SmoSat).\nOutside the layer the slope is 1 − G·R: z is pinned at ±K_smo, the reaching phase. "
             "Illustrative R %.2f, K_smo %.3f, SmoSat %.3f, e %.2f per unit; G = 1/L per control cycle." % (r, k, thr, e), color=INK2, fontsize=8.5)
    save(fig, "SensorlessSmo.png")
    return z_mean, s_mean

# ---------------------------------------------------------------------------
# 4. LPF
# ---------------------------------------------------------------------------
def fig_lpf():
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.2))

    # (a) the geometric series
    ax = axs[0]
    a = 0.25
    y, ys = 0.0, [0.0]
    for _ in range(12):
        y = m.lpf_step(a, y, 1.0); ys.append(y)
    ax.axhline(1.0, color=INK, lw=1.0, ls=(0, (4, 3)))
    label(ax, (12, 1.04), "the input, z", color=INK, ha="right", size=8.5)
    ax.step(range(len(ys)), ys, where="post", color=BLUE, lw=2.0)
    label(ax, (8.5, 0.82), "ê", color=BLUE)
    for n in range(4):
        ax.annotate("", (n + 1, ys[n + 1]), (n + 1, ys[n]), arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=1.0))
        ax.plot([n + 1, n + 1], [ys[n + 1], 1.0], color=MUTED, lw=0.8, ls=(0, (2, 2)))
        label(ax, (n + 1.12, (ys[n] + ys[n + 1]) / 2), "a·gap", color=ORANGE, ha="left", size=8)
    label(ax, (6.2, 0.55), "the gap after k steps: (1 − a)^k,\na geometric series", color=INK2, ha="left", size=8.5)
    ax.set_xlabel("steps"); ax.set_ylim(-0.03, 1.1)
    ax.set_title("(a) each step closes a fixed share of the gap (a = %.2f, drawn large)" % a)

    # (b) the frequency response is a circle
    ax = axs[1]
    a = m.lpf_coef(CONTROL_FREQ)
    T = 1.0 / CONTROL_FREQ
    def H(f):
        w = 2 * math.pi * f * T
        z = complex(math.cos(-w), math.sin(-w))
        return a / (1 - (1 - a) * z)
    hs = [H(f) for f in range(0, 5001, 5)]
    ax.add_patch(Circle((0.5, 0), 0.5, fill=False, color=RULE, lw=1.0, ls=(0, (4, 3))))
    ax.plot([h.real for h in hs], [h.imag for h in hs], color=BLUE, lw=2.0)
    marks = ((0, "0 Hz: dq, the EMF is DC,\ngain 1, no lag", (0.0, 0.1), "center"),
             (100, "100 Hz, the cutoff: −44°, 0.70", (0.04, -0.05), "left"),
             (200, "200 Hz", (0.05, -0.04), "left"),
             (400, "400 Hz", (0.04, -0.03), "left"))
    for f, txt, off, ha in marks:
        h = H(f)
        ax.plot([h.real], [h.imag], "o", color=ORANGE if f else AQUA, ms=6, zorder=5)
        label(ax, (h.real + off[0], h.imag + off[1]), txt, color=INK2, ha=ha, size=8.5)
    h = H(200)
    ax.plot([0, h.real], [0, h.imag], color=ORANGE, lw=1.4)
    lag = math.degrees(math.atan2(h.imag, h.real))
    ax.add_patch(Arc((0, 0), 0.36, 0.36, theta1=lag, theta2=0, color=ORANGE, lw=1.0))
    label(ax, (0.23, -0.08), "%.0f°" % lag, color=ORANGE, size=8.5)
    ax.axhline(0, color=RULE, lw=0.8); ax.axvline(0, color=RULE, lw=0.8)
    ax.set_aspect("equal"); ax.set_xlim(-0.1, 1.15); ax.set_ylim(-0.62, 0.32)
    ax.set_xlabel("real"); ax.set_ylabel("imaginary")
    ax.set_title("(b) the response lies on a circle (LpfCoef %d at %d kHz)" % (round(a * 32768), CONTROL_FREQ / 1000))
    label(ax, (0.55, 0.25), "In αβ the EMF turns at ω: ê shrinks and lags, to the circle's point.\nIn the frame θ̂ it stands still: ê sits at 1.", color=INK2, size=8.5)
    save(fig, "SensorlessLpf.png")

# ---------------------------------------------------------------------------
# 5. The rotor vector and the detector
# ---------------------------------------------------------------------------
def fig_detector():
    fig, axs = plt.subplots(1, 3, figsize=(16, 5.4), gridspec_kw=dict(width_ratios=[1.0, 1.25, 1.2]))

    # (a) the EMF a quarter turn ahead of the rotor, either way
    ax = axs[0]
    th = math.radians(30)
    ax.add_patch(Circle((0, 0), 1.0, fill=False, color=RULE, lw=0.8))
    arrow(ax, (0, 0), unit(th), BLUE, lw=2.2)
    label(ax, scale(1.22, unit(th)), "r, the rotor flux, at θ", color=BLUE, size=8.5)
    arrow(ax, (0, 0), unit(th + math.pi / 2), AQUA)
    label(ax, scale(1.18, unit(th + math.pi / 2)), "e, ω > 0", color=AQUA, size=8.5)
    arrow(ax, (0, 0), unit(th - math.pi / 2), AQUA, ls=(0, (3, 2)))
    label(ax, scale(1.18, unit(th - math.pi / 2)), "e, ω < 0", color=AQUA, size=8.5)
    ax.add_patch(FancyArrowPatch(scale(0.62, unit(th + math.pi / 2)), scale(0.62, unit(th + 0.12)), connectionstyle="arc3,rad=-0.4", arrowstyle="-|>", mutation_scale=10, color=ORANGE, lw=1.2))
    ax.add_patch(FancyArrowPatch(scale(0.62, unit(th - math.pi / 2)), scale(0.62, unit(th - 0.12)), connectionstyle="arc3,rad=0.4", arrowstyle="-|>", mutation_scale=10, color=ORANGE, lw=1.2, linestyle=(0, (3, 2))))
    label(ax, (-0.5, 0.15), "−j", color=ORANGE)
    label(ax, (0.62, -0.55), "+j", color=ORANGE)
    ax.set_title("(a) r = −j·sign(ω̂)·e")
    caption(ax, (0, -1.42), "e = jω·ψ: a quarter turn ahead of the flux, or behind\nin reverse. Turned back by the sign of ω̂, both give\nthe same rotor vector.")
    square(ax, ((-1.4, 1.4), (-1.75, 1.4)))

    # (b) the error is the parallelogram's area
    ax = axs[1]
    D = math.radians(35)
    floor = 0.18
    tol = 0.05
    ax.add_patch(Circle((0, 0), 1.0, fill=False, color=RULE, lw=0.8))
    ax.plot([-0.15, 1.95], [0, 0], color=MUTED, lw=0.9, ls=(0, (4, 3)), zorder=1)
    ax.plot([0, 0], [-0.15, 1.15], color=MUTED, lw=0.9, ls=(0, (4, 3)), zorder=1)
    label(ax, (2.0, 0), "d̂ at θ̂", color=INK2, ha="left")
    label(ax, (0, 1.22), "q̂", color=INK2)
    ax.add_patch(Circle((0, 0), floor, color=MUTED, alpha=0.25, lw=0))
    label(ax, (-0.5, -0.3), "MagFloor:\nno direction, error 0", color=INK2, size=8)
    half = math.degrees(math.asin(tol))
    ax.add_patch(Wedge((0, 0), 1.0, -half, half, width=1.0 - floor, color=AQUA, alpha=0.8, lw=0))
    ax.add_patch(Wedge((0, 0), 1.0, 180 - half, 180 + half, width=1.0 - floor, color=MUTED, alpha=0.5, lw=0))
    label(ax, (0.62, -0.13), "lock, ±%.1f°" % half, color=AQUA, size=8.5)
    label(ax, (-0.95, 0.14), "±%.1f° at π, repelled" % half, color=INK2, size=8, ha="left")
    r = unit(D)
    ax.add_patch(Polygon([(0, 0), (1, 0), add((1, 0), r), r], closed=True, color=BLUE_LIGHT, alpha=0.85, lw=0, zorder=1))
    arrow(ax, (0, 0), (1, 0), INK2, lw=1.4)
    arrow(ax, (0, 0), r, BLUE, lw=2.2)
    label(ax, add(r, (-0.08, 0.09)), "r / ‖r‖", color=BLUE, size=9)
    ax.plot([1 + r[0], 1 + r[0]], [0, r[1]], color=ORANGE, lw=2.4)
    label(ax, (1 + r[0], r[1] + 0.1), "height: sin(θ − θ̂)", color=ORANGE, size=8.5)
    angle_arc(ax, 0.0, D, 0.32, text="θ − θ̂", text_r=0.5)
    ax.set_title("(b) the error: the parallelogram's area")
    caption(ax, (0.45, -1.42), "angle_pll_error = x·sin θ̂ − y·cos θ̂ = ‖r‖·sin(θ̂ − θ): the cross product of r and d̂.\nNormalized by ‖r‖; the PI takes 0 minus it. The dq front end reads it\nin the frame θ̂: −y / ‖r‖.")
    square(ax, ((-1.3, 2.45), (-1.75, 1.35)))

    # (c) the S-curve: where the loop rests and where it cannot
    ax = axs[2]
    ds = [d / 2.0 for d in range(-360, 361)]
    ax.axhspan(-tol, tol, color=AQUA_LIGHT, lw=0)
    ax.plot(ds, [math.sin(math.radians(d)) for d in ds], color=BLUE, lw=2.0)
    label(ax, (-175, 0.85), "sin(θ − θ̂): θ̂ speeds up\nwhile it is positive", color=BLUE, ha="left", size=8.5)
    ax.plot([0], [0], "o", color=INK, ms=7, zorder=5)
    for d in (-180, 180):
        ax.plot([d], [0], "o", mfc=SURFACE, mec=INK, ms=7, zorder=5)
    label(ax, (0, -0.22), "stable:\nlock", color=INK, size=8.5)
    label(ax, (-180, -0.2), "unstable", color=INK, size=8.5)
    label(ax, (180, -0.2), "unstable", color=INK, size=8.5)
    for d in (-125, -65):
        arrow(ax, (d - 18, 0), (d + 18, 0), ORANGE, lw=1.6)
    for d in (65, 125):
        arrow(ax, (d + 18, 0), (d - 18, 0), ORANGE, lw=1.6)
    label(ax, (-95, 0.17), "θ − θ̂ shrinks", color=ORANGE, size=8.5)
    label(ax, (95, 0.17), "θ − θ̂ shrinks", color=ORANGE, size=8.5)
    label(ax, (95, -0.11), "|error| < LockErrTol", color=AQUA, size=8)
    ax.axhline(0, color=RULE, lw=0.8)
    ax.set_xlim(-200, 200); ax.set_ylim(-1.15, 1.15)
    ax.set_xticks([-180, -90, 0, 90, 180]); ax.set_xlabel("θ − θ̂ [°]"); ax.set_ylabel("the PI's input")
    ax.set_title("(c) the S-curve: one valley, one hilltop")
    save(fig, "SensorlessDetector.png")

# ---------------------------------------------------------------------------
# 6. The loop: a pendulum on a washboard
# ---------------------------------------------------------------------------
def simulate(freq, delta0, slip0_hz, w_hz=100.0, steps=1500, w_step_hz=0.0):
    """Δ = θ − θ̂ [rad], slip = ω − ω̂ [Hz], one entry per control cycle"""
    w = 2 * math.pi * w_hz / freq
    pll = m.AnglePll(freq, theta=-delta0, omega=w - 2 * math.pi * slip0_hz / freq)
    theta = 0.0
    ds, ss = [], []
    for n in range(steps):
        if n == 1:
            w += 2 * math.pi * w_step_hz / freq
        theta += w
        pll.predict()
        pll.correct(math.sin(theta - pll.theta))
        ds.append(theta - pll.theta)
        ss.append((w - pll.omega) * freq / (2 * math.pi))
    return ds, ss

def fig_pll():
    kp_s, ki_s, wn, zeta = m.pll_continuous(CONTROL_FREQ)
    fig, axs = plt.subplots(1, 3, figsize=(15.5, 5.0), gridspec_kw=dict(width_ratios=[1.1, 1.2, 1.0]))

    # (a) the washboard
    ax = axs[0]
    xs = [x / 100.0 for x in range(-820, 821)]
    for tilt, color, name in ((0.0, BLUE, "α = 0: level"), (0.3, ORANGE, "α = 0.3·Ki': tilted")):
        ax.plot(xs, [(1 - math.cos(x)) - tilt * x for x in xs], color=color, lw=2.0, label=name)
        rest = math.asin(tilt)
        for n in (-1, 0, 1):
            x0 = rest + 2 * n * math.pi
            ax.plot([x0], [(1 - math.cos(x0)) - tilt * x0 + 0.16], "o", color=color, ms=8, zorder=5)
    ax.legend(loc="upper right")
    ax.set_xticks([-2 * math.pi, -math.pi, 0, math.pi, 2 * math.pi]); ax.set_xticklabels(["−2π", "−π", "0", "π", "2π"])
    ax.set_xlabel("θ − θ̂"); ax.set_ylabel("U / Ki'")
    ax.set_ylim(-2.4, 5.0)
    ax.set_title("(a) a ball on a washboard")

    # (b) the phase portrait
    ax = axs[1]
    for d0 in (-160, -90, 0, 90, 160):
        for s0 in (-30, 0, 30):
            if d0 == 0 and s0 == 0:
                continue
            ds, ss = simulate(CONTROL_FREQ, math.radians(d0), s0)
            ax.plot([math.degrees(d) for d in ds], ss, color=BLUE, lw=0.9, alpha=0.85)
            ax.plot([d0], [ss[0]], "o", color=BLUE, ms=3)
    for s0 in (-90, 90, -140, 140):
        ds, ss = simulate(CONTROL_FREQ, 0.0, s0, steps=2500)
        ax.plot([math.degrees(d) for d in ds], ss, color=ORANGE, lw=1.0)
        ax.plot([0], [ss[0]], "o", color=ORANGE, ms=3)
    for d in (-360, 0, 360):
        ax.plot([d], [0], "o", color=INK, ms=7, zorder=5)
    for d in (-180, 180):
        ax.plot([d], [0], "o", mfc=SURFACE, mec=INK, ms=7, zorder=5)
    label(ax, (300, 135), "large slips: a cycle\nslips, then the next valley", color=ORANGE, size=8.5)
    label(ax, (-330, 135), "small: a spiral\ninto the nearest valley", color=BLUE, size=8.5)
    ax.axhline(0, color=RULE, lw=0.8)
    ax.set_xlim(-560, 560); ax.set_xticks([-540, -360, -180, 0, 180, 360, 540])
    ax.set_xlabel("θ − θ̂ [°]"); ax.set_ylabel("ω − ω̂ [Hz]")
    ax.set_title("(b) the phase portrait, %d kHz, the default gains" % (CONTROL_FREQ / 1000))

    # (c) a speed step
    ax = axs[2]
    ds, ss = simulate(CONTROL_FREQ, 0.0, 0.0, steps=1000, w_step_hz=20.0)
    t = [n / CONTROL_FREQ * 1000.0 for n in range(len(ds))]
    ax.plot(t, [math.degrees(d) for d in ds], color=BLUE, lw=2.0)
    ax.axhline(0, color=RULE, lw=0.8)
    ax.set_xlabel("ms"); ax.set_ylabel("θ − θ̂ [°]")
    ax.set_title("(c) a 20 Hz speed step: back to 0")
    label(ax, (98, 11), "type 2: the integrator learns the new ω,\nso the error returns to 0.\nω_n %.0f Hz, ζ %.2f at %d kHz" % (wn / (2 * math.pi), zeta, CONTROL_FREQ / 1000), color=INK2, ha="right", size=8.5)
    save(fig, "SensorlessPll.png")

def print_numbers(z_mean, s_mean):
    for f in (10000.0, 20000.0):
        kp_s, ki_s, wn, zeta = m.pll_continuous(f)
        print("Freq %5.0f Hz: Kp' %.0f /s, Ki' %.0f /s^2 (%.0f Hz/s), w_n %.1f Hz, zeta %.2f" % (f, kp_s, ki_s, ki_s / (2 * math.pi), wn / (2 * math.pi), zeta))
    r, k, thr, e = SMO["r"], SMO["k"], SMO["thr"], SMO["e"]
    for g in (0.09, 0.45, 0.80):
        print("SMO G %.2f: pole %+.3f" % (g, m.smo_pole(g, r, k, thr)))
    print("SMO sigma* %.4f; chatter at G 0.80: mean z %.4f, mean sigma %.4f, e - R*mean sigma %.4f" % (m.smo_fixed_point(r, k, thr, e), z_mean, s_mean, e - r * s_mean))
    print("LpfCoef at 10 kHz %d, at 20 kHz %d" % (round(m.lpf_coef(10000.0) * 32768), round(m.lpf_coef(20000.0) * 32768)))

if __name__ == "__main__":
    fig_voltage_polygon()
    fig_frames()
    z_mean, s_mean = fig_smo()
    fig_lpf()
    fig_detector()
    fig_pll()
    print_numbers(z_mean, s_mean)
