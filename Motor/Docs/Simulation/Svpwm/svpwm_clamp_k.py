"""
The general clamp's k, animated. [svpwm_clamp] places the zero sequence: every phase duty moves with it, and the
line-to-line voltages, all the motor sees, stay. The vector fixes the null time, 1 - spread. k puts k of it at O111,
all phases high, and 1 - k at O000, all phases low. Reading 3 phases, a low-side current sense has O000 to read in,
the top phase's low-side window (1 - k)*(1 - spread)*T.

    left    the duties over one turn at |v| = Vbus/2: O000 above the highest duty, O111 below the lowest
    middle  the period at the line-to-line peak, 330 deg, where the window is shortest: O000, v_ac, v_cb, O111
    right   that period on FTM2, CPWM high-true: pulses centred on CNTIN, O000 about MOD and its TOF

k holds at midclamp, the bottom clamp and the top clamp in turn. Duty is linear in k, so frames keyed at the holds
interpolate exactly. SMIL only, no script, so it animates as an <img> in markdown.
Writes SVPWM_Clamp_K.svg beside this file.
"""
import math
import os

RADIUS = 0.5                     # |v| per Vbus, FOC_V_CIRCLE_PER_V_BUS default
T_US = 50.0                      # 20 kHz
THETA_DEG = 330                  # a line-to-line peak
DUTY_LIMIT = 1.0 - 3.35 / T_US   # 93.3 %, the shortest window that works today

TIMELINE = ((0.0, 0.5), (2.0, 0.5), (3.5, 0.0), (5.5, 0.0), (8.5, 1.0), (10.5, 1.0), (12.0, 0.5))  # (s, k)
PERIOD_S = 2.4                   # one PWM period on screen

# One vertical axis for the plot, the slice and the carrier: duty, the compare value per MOD
PLOT_X0, PLOT_X1 = 80.0, 560.0
SLICE_X, SLICE_W = 614.0, 18.0
CARRIER_X0, CARRIER_X1 = 700.0, 1150.0
DUTY_Y0, DUTY_Y1 = 475.0, 155.0
TRACE_Y = {"A": 522.0, "B": 556.0, "C": 590.0}
TRACE_SWING = 9.0

COLOR = {"A": "#ff6b6b", "B": "#3ddc97", "C": "#4ea8ff", "O000": "#ffbf47", "O111": "#a98bff", "active": "#34406e"}
INK, MUTED, FAINT, PAPER, SCREEN = "#f2f4ff", "#aab3dd", "#6f7bb0", "#0b1027", "#0f1738"


def phase_voltages(radius, theta):
    """A balanced three phase set of peak radius per Vbus, as svpwm_clamp takes it."""
    return tuple(radius * math.cos(theta - p) for p in (0.0, 2 * math.pi / 3, 4 * math.pi / 3))


def svpwm_clamp(vA, vB, vC, k):
    """svpwm_clamp (Motor/Motor/Math/svpwm_math.h), without the saturation the linear range never reaches."""
    vZero = k * (max(vA, vB, vC) - 1.0) + (1.0 - k) * min(vA, vB, vC)
    return vA - vZero, vB - vZero, vC - vZero


def duty(theta_deg, k):
    return dict(zip("ABC", svpwm_clamp(*phase_voltages(RADIUS, math.radians(theta_deg)), k)))


def x_of_theta(deg): return PLOT_X0 + (PLOT_X1 - PLOT_X0) * deg / 360.0
def y_of_duty(d): return DUTY_Y0 + (DUTY_Y1 - DUTY_Y0) * d
def x_match_up(d): return CARRIER_X0 + (CARRIER_X1 - CARRIER_X0) / 2 * d    # the counter climbs past CnV: low
def x_match_down(d): return CARRIER_X1 - (CARRIER_X1 - CARRIER_X0) / 2 * d  # and falls back past it: high


def num(v, places=1):
    s = f"{v:.{places}f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def path_of(points):
    """M, then l steps taken between rounded absolutes: every frame shares one command list and nothing drifts."""
    xy = [(round(x, 1), round(y, 1)) for x, y in points]
    return f"M{num(xy[0][0])},{num(xy[0][1])}" + "".join(f"l{num(x - px)},{num(y - py)}" for (px, py), (x, y) in zip(xy, xy[1:]))


THETAS = range(0, 361, 3)


def curve(phase, k): return path_of([(x_of_theta(t), y_of_duty(duty(t, k)[phase])) for t in THETAS])
def band_o000(k): return path_of([(x_of_theta(t), y_of_duty(max(duty(t, k).values()))) for t in THETAS]) + f"V{num(DUTY_Y1)}H{num(PLOT_X0)}Z"
def band_o111(k): return path_of([(x_of_theta(t), y_of_duty(min(duty(t, k).values()))) for t in THETAS]) + f"V{num(DUTY_Y0)}H{num(PLOT_X0)}Z"


LOOP_S = TIMELINE[-1][0]
LOOP = f'dur="{num(LOOP_S)}s" repeatCount="indefinite"'
PERIOD = f'dur="{PERIOD_S}s" repeatCount="indefinite"'
KEYED = (f'calcMode="spline" keyTimes="{";".join(num(t / LOOP_S, 4) for t, _ in TIMELINE)}" '
         f'keySplines="{";".join("0 0 1 1" if k0 == k1 else ".45 0 .55 1" for (_, k0), (_, k1) in zip(TIMELINE, TIMELINE[1:]))}"')
HOLDS = {k0: (t0, t1) for (t0, k0), (t1, k1) in zip(TIMELINE, TIMELINE[1:]) if k0 == k1}


def over_k(attribute, value_of_k):
    """<animate> through the k timeline, keyed at the holds."""
    return f'<animate attributeName="{attribute}" {LOOP} {KEYED} values="{";".join(value_of_k(k) for _, k in TIMELINE)}"/>'


def slide_over_k(dy_of_k):
    values = ";".join(f"0,{num(dy_of_k(k))}" for _, k in TIMELINE)
    return f'<animateTransform attributeName="transform" type="translate" {LOOP} {KEYED} values="{values}"/>'


def while_held(k, low=0.0, high=1.0, fade=0.3):
    """Opacity: high through the hold at k, low elsewhere, fading either side. Cyclic over the loop."""
    start, end = HOLDS[k]

    def level(t):
        since = (t - start) % LOOP_S
        if since <= end - start:
            return high
        return low + (high - low) * max(0.0, 1 - (since - (end - start)) / fade, 1 - (LOOP_S - since) / fade)

    times = sorted({0.0, LOOP_S} | {x % LOOP_S for x in (start - fade, start, end, end + fade)})
    return (f'<animate attributeName="opacity" {LOOP} keyTimes="{";".join(num(t / LOOP_S, 4) for t in times)}" '
            f'values="{";".join(num(level(t), 2) for t in times)}"/>')


def at(k): return duty(THETA_DEG, k)


REF = at(0.5)                                         # the static frame, midclamp, the default
TOP, MID, LOW = sorted(REF, key=REF.get, reverse=True)
CURSOR_X = x_of_theta(THETA_DEG)
APEX_X = (CARRIER_X0 + CARRIER_X1) / 2


def slide(k): return y_of_duty(at(k)[TOP]) - y_of_duty(REF[TOP])    # the same for every phase: a rigid move


def window_us(k): return (1.0 - at(k)[TOP]) * T_US


svg = []
add = svg.append

add(f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 700" width="1200" height="700" role="img" aria-labelledby="title desc">
<title id="title">k moves the null time, not the voltage</title>
<desc id="desc">svpwm_clamp at |v| = Vbus/2. As k moves between the bottom clamp, midclamp and the top clamp, all three phase duties move together and the line-to-line voltages stay the same. Only the split of the null time between O000, all low, and O111, all high, changes. Reading 3 phases, a low-side current sense has O000 to read in, so the window at the line-to-line peak is 6.70 us at k = 0, 3.35 us at k = 1/2 and none at k = 1.</desc>
<defs>
<style>
text {{ font-family: 'Cascadia Mono', Consolas, 'SF Mono', Menlo, 'DejaVu Sans Mono', monospace; font-size: 13px; fill: {MUTED}; }}
.title {{ font-family: 'Segoe UI', system-ui, -apple-system, 'Helvetica Neue', Arial, sans-serif; font-size: 30px; font-weight: 700; fill: {INK}; }}
.kicker {{ letter-spacing: 2.4px; fill: #9aa7ff; }}
.head {{ font-size: 12px; letter-spacing: 1.6px; fill: #8792c6; }}
.tick {{ font-size: 12px; fill: {FAINT}; }}
.ink {{ fill: {INK}; }}
.halo {{ paint-order: stroke; stroke: {PAPER}; stroke-width: 4px; stroke-linejoin: round; }}
.grid {{ stroke: #1c2752; stroke-width: 1; }}
</style>
<pattern id="dots" width="24" height="24" patternUnits="userSpaceOnUse"><circle cx="2" cy="2" r="1" fill="#18204a"/></pattern>
<linearGradient id="counter" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#8d9cff" stop-opacity=".16"/><stop offset="1" stop-color="#8d9cff" stop-opacity=".02"/></linearGradient>
<filter id="glow" x="-1" y="-1" width="3" height="3"><feGaussianBlur stdDeviation="2.5" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
</defs>
<rect width="1200" height="700" fill="{PAPER}"/>
<rect width="1200" height="700" fill="url(#dots)"/>''')

# Header and key
add(f'''<text x="60" y="44" class="kicker">ZERO-SEQUENCE PLACEMENT</text>
<text x="60" y="80" class="title">k moves the null time, not the voltage</text>
<text x="60" y="106">svpwm_clamp(vA, vB, vC, k) · |v| = Vbus/2 · 20 kHz · 3 phases read</text>''')
for i, p in enumerate("ABC"):
    add(f'<line x1="{820 + 46 * i}" y1="38" x2="{836 + 46 * i}" y2="38" stroke="{COLOR[p]}" stroke-width="3" stroke-linecap="round"/>'
        f'<text x="{842 + 46 * i}" y="42" class="ink">{p}</text>')
add(f'<text x="962" y="42">phase duty</text>')
for y, swatch, label in ((64, COLOR["O000"], "O000 · all low: the low-side window"),
                         (86, COLOR["active"], f"v_{TOP.lower()}{MID.lower()}, v_{MID.lower()}{LOW.lower()} · line-to-line, any k"),
                         (108, COLOR["O111"], "O111 · all high")):
    add(f'<rect x="820" y="{y - 10}" width="16" height="11" rx="2" fill="{swatch}"/><text x="844" y="{y}">{label}</text>')

# Column heads
add(f'''<text x="{PLOT_X0}" y="138" class="head">DUTY OVER ONE TURN</text>
<text x="{CARRIER_X0}" y="138" class="head">ONE PWM PERIOD AT {THETA_DEG}°</text>''')

# Duty over one turn
add(f'<rect x="{PLOT_X0}" y="{DUTY_Y1}" width="{PLOT_X1 - PLOT_X0}" height="{DUTY_Y0 - DUTY_Y1}" fill="{SCREEN}"/>')
for t in range(60, 360, 60):
    add(f'<line x1="{num(x_of_theta(t))}" y1="{DUTY_Y1}" x2="{num(x_of_theta(t))}" y2="{DUTY_Y0}" class="grid"/>')
add(f'<line x1="{PLOT_X0}" y1="{num(y_of_duty(.5))}" x2="{PLOT_X1}" y2="{num(y_of_duty(.5))}" class="grid" stroke-dasharray="3 5"/>')
add(f'<path d="{band_o111(.5)}" fill="{COLOR["O111"]}" fill-opacity=".32">{over_k("d", band_o111)}</path>')
add(f'<path d="{band_o000(.5)}" fill="{COLOR["O000"]}" fill-opacity=".36">{over_k("d", band_o000)}</path>')
for p in "ABC":
    add(f'<path d="{curve(p, .5)}" fill="none" stroke="{COLOR[p]}" stroke-width="2.2" stroke-linejoin="round">{over_k("d", lambda k, p=p: curve(p, k))}</path>')
for t in range(0, 361, 60):
    add(f'<text x="{num(x_of_theta(t))}" y="492" class="tick" text-anchor="middle">{t}°</text>')
for d, label in ((1, "1"), (.5, "0.5"), (0, "0")):
    add(f'<text x="72" y="{num(y_of_duty(d) + 4)}" class="tick" text-anchor="end">{label}</text>')
add(f'''<line x1="{num(CURSOR_X)}" y1="{DUTY_Y1}" x2="{num(CURSOR_X)}" y2="{DUTY_Y0}" stroke="{INK}" stroke-opacity=".45" stroke-dasharray="4 4"/>
<text x="{num(CURSOR_X)}" y="149" class="tick" text-anchor="middle">{THETA_DEG}°</text>''')

# One PWM period: the counter from CNTIN to MOD and back
apex = f"{num(APEX_X)},{num(DUTY_Y1)}"
add(f'''<rect x="{CARRIER_X0}" y="{DUTY_Y1}" width="{CARRIER_X1 - CARRIER_X0}" height="{DUTY_Y0 - DUTY_Y1}" fill="{SCREEN}"/>
<line x1="{num(APEX_X)}" y1="{DUTY_Y1}" x2="{num(APEX_X)}" y2="{DUTY_Y0}" class="grid" stroke-dasharray="3 5"/>
<polygon points="{num(CARRIER_X0)},{num(DUTY_Y0)} {apex} {num(CARRIER_X1)},{num(DUTY_Y0)}" fill="url(#counter)"/>''')


def cap(k):
    d = at(k)[TOP]
    return f"{num(x_match_up(d))},{num(y_of_duty(d))} {apex} {num(x_match_down(d))},{num(y_of_duty(d))}"


def foot_up(k):
    d = at(k)[LOW]
    return f"{num(CARRIER_X0)},{num(DUTY_Y0)} {num(x_match_up(d))},{num(y_of_duty(d))} {num(x_match_up(d))},{num(DUTY_Y0)}"


def foot_down(k):
    d = at(k)[LOW]
    return f"{num(CARRIER_X1)},{num(DUTY_Y0)} {num(x_match_down(d))},{num(y_of_duty(d))} {num(x_match_down(d))},{num(DUTY_Y0)}"


add(f'<polygon points="{cap(.5)}" fill="{COLOR["O000"]}" fill-opacity=".85">{over_k("points", cap)}</polygon>')
for foot in (foot_up, foot_down):
    add(f'<polygon points="{foot(.5)}" fill="{COLOR["O111"]}" fill-opacity=".85">{over_k("points", foot)}</polygon>')

# The compare values, the plot's slice at the cursor: they slide as one
add(f'<g>{slide_over_k(slide)}')
for p in "ABC":
    y = num(y_of_duty(REF[p]))
    add(f'<line x1="{num(CURSOR_X)}" y1="{y}" x2="{CARRIER_X1}" y2="{y}" stroke="{COLOR[p]}" stroke-width="1.4" stroke-opacity=".75"/>')
add('</g>')

# The slice: O000, the line-to-line voltages, O111
add(f'<rect x="{SLICE_X}" y="{DUTY_Y1}" width="{SLICE_W}" height="{num(y_of_duty(REF[TOP]) - DUTY_Y1)}" fill="{COLOR["O000"]}">'
    f'{over_k("height", lambda k: num(y_of_duty(at(k)[TOP]) - DUTY_Y1))}</rect>')
add(f'<rect x="{SLICE_X}" y="{num(y_of_duty(REF[LOW]))}" width="{SLICE_W}" height="{num(DUTY_Y0 - y_of_duty(REF[LOW]))}" fill="{COLOR["O111"]}">'
    f'{over_k("y", lambda k: num(y_of_duty(at(k)[LOW])))}{over_k("height", lambda k: num(DUTY_Y0 - y_of_duty(at(k)[LOW])))}</rect>')
add(f'<g>{slide_over_k(slide)}')
for upper, lower in ((TOP, MID), (MID, LOW)):
    y0, y1 = y_of_duty(REF[upper]), y_of_duty(REF[lower])
    add(f'<rect x="{SLICE_X}" y="{num(y0)}" width="{SLICE_W}" height="{num(y1 - y0)}" fill="{COLOR["active"]}" stroke="{PAPER}" stroke-width="1"/>'
        f'<text x="{SLICE_X + SLICE_W + 8}" y="{num((y0 + y1) / 2 + 4)}" class="ink halo">v_{upper.lower()}{lower.lower()}</text>')
for p in "ABC":
    add(f'<circle cx="{num(CURSOR_X)}" cy="{num(y_of_duty(REF[p]))}" r="4.5" fill="{COLOR[p]}" stroke="{PAPER}" stroke-width="2"/>')
add('</g>')
add(f'<text x="{SLICE_X + SLICE_W + 8}" y="{num((DUTY_Y1 + y_of_duty(REF[TOP])) / 2 + 4)}" class="halo" style="fill:{COLOR["O000"]}">O000'
    f'{over_k("y", lambda k: num((DUTY_Y1 + y_of_duty(at(k)[TOP])) / 2 + 4))}{over_k("opacity", lambda k: "0" if k == 1 else "1")}</text>')
add(f'<text x="{SLICE_X + SLICE_W + 8}" y="{num((DUTY_Y0 + y_of_duty(REF[LOW])) / 2 + 4)}" class="halo" style="fill:{COLOR["O111"]}">O111'
    f'{over_k("y", lambda k: num((DUTY_Y0 + y_of_duty(at(k)[LOW])) / 2 + 4))}{over_k("opacity", lambda k: "0" if k == 0 else "1")}</text>')

# The counter over the compare values, and where each phase switches
add(f'<polyline points="{num(CARRIER_X0)},{num(DUTY_Y0)} {apex} {num(CARRIER_X1)},{num(DUTY_Y0)}" fill="none" stroke="#d7ddff" stroke-width="2" stroke-linejoin="round"/>')
add(f'<line x1="{PLOT_X0}" y1="{num(y_of_duty(DUTY_LIMIT))}" x2="{CARRIER_X1}" y2="{num(y_of_duty(DUTY_LIMIT))}" stroke="{INK}" stroke-opacity=".4" stroke-dasharray="2 4"/>'
    f'<text x="72" y="{num(y_of_duty(DUTY_LIMIT) + 4)}" class="tick" text-anchor="end">93.3%</text>')
for p in "ABC":
    for x_match in (x_match_up, x_match_down):
        add(f'<path d="M{num(x_match(REF[p]))},{num(y_of_duty(REF[p]))}V{num(TRACE_Y[p])}" stroke="{COLOR[p]}" stroke-opacity=".4" stroke-dasharray="2 3">'
            f'{over_k("d", lambda k, p=p, x_match=x_match: f"M{num(x_match(at(k)[p]))},{num(y_of_duty(at(k)[p]))}V{num(TRACE_Y[p])}")}</path>')
add(f'<g>{slide_over_k(slide)}')
for p in "ABC":
    for x_match in (x_match_up, x_match_down):
        add(f'<circle cx="{num(x_match(REF[p]))}" cy="{num(y_of_duty(REF[p]))}" r="3.5" fill="{COLOR[p]}" stroke="{PAPER}" stroke-width="1.5">'
            f'{over_k("cx", lambda k, p=p, x_match=x_match: num(x_match(at(k)[p])))}</circle>')
    add(f'<text x="{CARRIER_X1 + 8}" y="{num(y_of_duty(REF[p]) + 4)}" class="halo" style="fill:{COLOR[p]}">{p}</text>')
add('</g>')
add(f'''<text x="{num(APEX_X)}" y="149" class="tick" text-anchor="middle">MOD · TOF</text>
<text x="{CARRIER_X0}" y="632" class="tick">CNTIN</text>
<text x="{CARRIER_X1}" y="632" class="tick" text-anchor="end">CNTIN</text>''')

# The phase outputs: high-true, centred on CNTIN
TRACE_TOP, TRACE_BOTTOM = TRACE_Y["A"] - 17, TRACE_Y["C"] + 17
add(f'<rect x="{CARRIER_X0}" y="{TRACE_TOP}" width="{CARRIER_X1 - CARRIER_X0}" height="{TRACE_BOTTOM - TRACE_TOP}" fill="{SCREEN}"/>')
add(f'<rect x="{num(x_match_up(REF[TOP]))}" y="{TRACE_TOP}" width="{num(x_match_down(REF[TOP]) - x_match_up(REF[TOP]))}" height="{TRACE_BOTTOM - TRACE_TOP}" fill="{COLOR["O000"]}" fill-opacity=".28">'
    f'{over_k("x", lambda k: num(x_match_up(at(k)[TOP])))}{over_k("width", lambda k: num(x_match_down(at(k)[TOP]) - x_match_up(at(k)[TOP])))}</rect>')
add(f'<rect x="{CARRIER_X0}" y="{TRACE_TOP}" width="{num(x_match_up(REF[LOW]) - CARRIER_X0)}" height="{TRACE_BOTTOM - TRACE_TOP}" fill="{COLOR["O111"]}" fill-opacity=".28">'
    f'{over_k("width", lambda k: num(x_match_up(at(k)[LOW]) - CARRIER_X0))}</rect>')
add(f'<rect x="{num(x_match_down(REF[LOW]))}" y="{TRACE_TOP}" width="{num(CARRIER_X1 - x_match_down(REF[LOW]))}" height="{TRACE_BOTTOM - TRACE_TOP}" fill="{COLOR["O111"]}" fill-opacity=".28">'
    f'{over_k("x", lambda k: num(x_match_down(at(k)[LOW])))}{over_k("width", lambda k: num(CARRIER_X1 - x_match_down(at(k)[LOW])))}</rect>')


def levels(phase, k):
    """The high and low runs as separate subpaths: a run that closes has length zero and draws nothing."""
    d, high, low = at(k)[phase], TRACE_Y[phase] - TRACE_SWING, TRACE_Y[phase] + TRACE_SWING
    up, down = num(x_match_up(d)), num(x_match_down(d))
    return f"M{num(CARRIER_X0)},{num(high)}H{up}M{up},{num(low)}H{down}M{down},{num(high)}H{num(CARRIER_X1)}"


def edges(phase, k):
    d, high, low = at(k)[phase], TRACE_Y[phase] - TRACE_SWING, TRACE_Y[phase] + TRACE_SWING
    return f"M{num(x_match_up(d))},{num(high)}V{num(low)}M{num(x_match_down(d))},{num(low)}V{num(high)}"


def hidden_while(is_gone):
    """Opacity 0 through the hold where is_gone(k): an edge or a bracket with no pulse to bound."""
    holds = [k for k in HOLDS if is_gone(k)]
    assert len(holds) <= 1
    return while_held(holds[0], low=1, high=0) if holds else ""


def on_rail(d): return min(abs(d), abs(1 - d)) < 1e-9


for p in "ABC":
    add(f'<path d="{levels(p, .5)}" fill="none" stroke="{COLOR[p]}" stroke-width="2">{over_k("d", lambda k, p=p: levels(p, k))}</path>'
        f'<path d="{edges(p, .5)}" fill="none" stroke="{COLOR[p]}" stroke-width="2" stroke-linecap="square">'
        f'{over_k("d", lambda k, p=p: edges(p, k))}{hidden_while(lambda k, p=p: on_rail(at(k)[p]))}</path>'
        f'<text x="{CARRIER_X0 - 10}" y="{num(TRACE_Y[p] + 4)}" text-anchor="end" style="fill:{COLOR[p]}">{p}</text>')


# The window, and what it reads
def bracket(k):
    x0, x1 = x_match_up(at(k)[TOP]), x_match_down(at(k)[TOP])
    return f"M{num(x0)},{TRACE_BOTTOM + 4}v12M{num(x0)},{TRACE_BOTTOM + 10}H{num(x1)}M{num(x1)},{TRACE_BOTTOM + 4}v12"


add(f'<path d="{bracket(.5)}" fill="none" stroke="{COLOR["O000"]}" stroke-width="1.6">'
    f'{over_k("d", bracket)}{hidden_while(lambda k: on_rail(at(k)[TOP]))}</path>')
add(f'<text x="{CARRIER_X0}" y="656">O000 = (1 − k)·(1 − spread)·T =</text>')
for k, label, color in ((0.5, f"{window_us(.5):.2f} µs", INK), (0.0, f"{window_us(0):.2f} µs", INK), (1.0, f"0 µs · {TOP} never low", "#ff5d73")):
    add(f'<text x="{CARRIER_X0 + 252}" y="656" style="fill:{color}" opacity="{1 if k == .5 else 0}">{label}{while_held(k)}</text>')

# The counter running
add(f'''<line x1="{CARRIER_X0}" y1="{DUTY_Y1}" x2="{CARRIER_X0}" y2="{TRACE_BOTTOM}" stroke="{INK}" stroke-opacity=".22">
<animateTransform attributeName="transform" type="translate" values="0,0;{num(CARRIER_X1 - CARRIER_X0)},0" {PERIOD}/></line>
<circle r="4.5" fill="{INK}" filter="url(#glow)"><animateMotion path="M{num(CARRIER_X0)},{num(DUTY_Y0)}L{apex}L{num(CARRIER_X1)},{num(DUTY_Y0)}" {PERIOD}/></circle>
<circle cx="{num(APEX_X)}" cy="{num(DUTY_Y1)}" r="6" fill="none" stroke="{COLOR["O000"]}" stroke-width="2" opacity="0">
<animate attributeName="r" values="6;6;26;26" keyTimes="0;.5;.8;1" {PERIOD}/>
<animate attributeName="opacity" values="0;0;.9;0;0" keyTimes="0;.499;.5;.8;1" {PERIOD}/></circle>''')

# k
KNOB_X0, KNOB_X1, KNOB_Y = PLOT_X0, PLOT_X1, 548


def knob_x(k): return num(KNOB_X0 + (KNOB_X1 - KNOB_X0) * k)


add(f'''<text x="{PLOT_X0}" y="524">k · the share of the null time at O111</text>
<line x1="{KNOB_X0}" y1="{KNOB_Y}" x2="{knob_x(.5)}" y2="{KNOB_Y}" stroke="{COLOR["O111"]}" stroke-width="6" stroke-linecap="round">{over_k("x2", knob_x)}</line>
<line x1="{knob_x(.5)}" y1="{KNOB_Y}" x2="{KNOB_X1}" y2="{KNOB_Y}" stroke="{COLOR["O000"]}" stroke-width="6" stroke-linecap="round">{over_k("x1", knob_x)}</line>''')
for k, anchor, name, note in ((0.0, "start", "0 · bottom clamp", "PHASE_SVPWM_CLAMP"), (0.5, "middle", "1/2 · midclamp", "svpwm_midclamp, default"), (1.0, "end", "1 · top clamp", "no O000")):
    add(f'<line x1="{knob_x(k)}" y1="{KNOB_Y + 10}" x2="{knob_x(k)}" y2="{KNOB_Y + 16}" class="grid" stroke="{FAINT}"/>'
        f'<g opacity="{1 if k == .5 else .4}">{while_held(k, .4, 1)}<text x="{knob_x(k)}" y="{KNOB_Y + 32}" text-anchor="{anchor}" class="ink">{name}</text>'
        f'<text x="{knob_x(k)}" y="{KNOB_Y + 48}" text-anchor="{anchor}" class="tick">{note}</text></g>')
add(f'<circle cx="{knob_x(.5)}" cy="{KNOB_Y}" r="9" fill="{INK}" stroke="{PAPER}" stroke-width="3">{over_k("cx", knob_x)}</circle>')

add(f'''<text x="{PLOT_X0}" y="634" class="ink">vZero = k·(vMax − 1) + (1 − k)·vMin</text>
<text x="{PLOT_X0}" y="656">duty{TOP} − duty{MID} = v{TOP} − v{MID}, for every k</text>
<text x="{PLOT_X0}" y="686" class="tick">Motor/Motor/Math/svpwm_math.h</text>
<text x="{CARRIER_X1}" y="686" class="tick" text-anchor="end">FTM2 CPWM, high-true · T = {num(T_US)} µs</text>
</svg>''')

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Clamp_K.svg")
with open(path, "w", encoding="utf-8", newline="\n") as f:
    f.write("\n".join(svg) + "\n")
print(f"{THETA_DEG} deg: duties at k = 0, 1/2, 1:",
      *(", ".join(f"{p} {at(k)[p]:.3f}" for p in "ABC") for k in (0.0, 0.5, 1.0)), sep="\n  ")
print(f"O000 window: {window_us(0):.2f}, {window_us(.5):.2f}, {window_us(1):.2f} us; {os.path.getsize(path) / 1024:.0f} KiB")
