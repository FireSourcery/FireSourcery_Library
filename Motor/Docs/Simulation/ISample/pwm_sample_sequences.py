"""
KE current-loop sequences: where the I sample is captured, Parked and run through the PI, and when its output loads.

One timeline per sequence (PwmSampleSequence_<n>_<name>.png), and a summary across a CPU speed factor (PwmSampleSequenceSummary.png).
CurrentLoopSequencing.md and PwmConvention.md describe the hardware and the code paths.

Model
    FTM2 center-aligned, T = 50 us. Counter top (TOF, PWM ISR) at t = 0, 50, ...; bottom at 25 + 50n.
    Both loading points are enabled: a duty written loads at the next top or bottom after the write.
    The current loop runs at 10 kHz: the analog ticks are t = 0, 100, ... (every 2nd period). Batch b is the sample started on tick b.
    ADC: 10 MHz ADCK, VSOURCE then IA, IB, IC; IB samples 4.5 us after the start, the batch completes at 8.5 us.
    NVIC: lower priority value preempts; equal priority does not, and a tie between pending IRQs goes to the lower number (ADC 15, FTM2 19).
    The sample's place in the period does not change its age through the sense filter (CurrentSenseFilter.md), so the comparison
    is sample -> load. Delay to actuation = age (~96 us at hold tau 10 us) + sample -> load + 50 us (half the 2-period hold).

Durations are estimates at k = 1: instruction counts of the linked KE06 image at ~32 ns each (50 MHz, CPI ~1.6). Not measured.
The summary sweeps k, a factor on every duration, because which load point each sequence makes depends on it.
ASCII console output. Writes the PNGs beside this file.
"""
import heapq
import itertools
import math
import os

import matplotlib.pyplot as plt
from matplotlib.patches import Patch

T = 50.0
LOAD = 25.0  # loading points: every top and bottom
DT = 0.05
PRIO_PWM, PRIO_HIGH = 2, 1
IRQ_ADC, IRQ_FTM2 = 15, 19

BASE = dict(
    entry=0.4,       # exception entry
    adc_start=1.6,   # mark VSOURCE, IA, IB, IC; activate the FIFO
    sensor=3.0,      # Hall angle capture: theta of this tick
    sm=2.0,          # state machine dispatch
    requests=7.2,    # torque ramp, field-weakening integrator (vector magnitude, sqrt)
    theta=1.6,       # FOC_SetTheta, sin/cos
    park=1.5,        # Clarke, Park
    pi=9.6,          # 2 PID, the circle limit's sqrt
    out=5.1,         # inverse Park, inverse Clarke, SVPWM, duty write
    exit=1.5,
    adc_entry=1.5,   # ADC ISR entry, FIFO read, walk continuation
    vsource=3.5,     # VBus capture and its divide
    icapture=2.0,    # the I set from the results buffer
    dispatch=0.3,    # ON_PHASE action dispatch
    adc_exit=0.5,
)
CONV = (("VSOURCE", 0.0, 2.5), ("IA", 2.5, 2.0), ("IB", 4.5, 2.0), ("IC", 6.5, 2.0))
CONV_DONE = 8.5
SAMPLE_OFFSET = 4.5  # IB, the middle of the set
AGE_US, HOLD_HALF_US = 96.0, 50.0  # CurrentSenseFilter.md, hold tau 10 us, 300 Hz; the 10 kHz loop's half hold
HW_LEAD = CONV_DONE + BASE["adc_entry"] + BASE["vsource"] + BASE["icapture"] + BASE["adc_exit"] + 0.5  # sequence 6: done before the top

KIND_COLOR = {"shell": "#d9d9d9", "theta": "#81c784", "req": "#b39ddb", "capture": "#64b5f6", "pi": "#e57373", "out": "#ffb74d", "spin": "#f5f5f5"}
KIND_LABEL = {"shell": "entry, dispatch, exit", "theta": "angle: sensor, SetTheta", "req": "requests: ramp, field weakening",
              "capture": "VSOURCE, I capture, Park", "pi": "PI", "out": "inverse Park, SVPWM, write", "spin": "spin, waiting for the ADC"}


class Seq:
    def __init__(self, key, title, file, adc_prio, adc_isr, park_in, pi_in, order, trigger, notes):
        self.key, self.title, self.file = key, title, file
        self.adc_prio, self.adc_isr = adc_prio, adc_isr      # the ADC ISR handles the I set
        self.park_in, self.pi_in = park_in, pi_in            # 'adc' / 'loop' ; 'loop' / 'adc' / 'poll'
        self.order, self.trigger = order, trigger            # 'default' / 'foc_first' ; 'sw' / 'hw'
        self.notes = notes


SEQS = [
    Seq("0", "LOOP captures, at the next tick's angle", None, PRIO_PWM, True, "loop", "loop", "default", "sw",
        ["Reference only. The ADC ISR captures, the next tick's LOOP Parks and runs the PI."]),
    Seq("1", "Run_Proc, ADC ISR at equal priority (MOTOR_I_CAPTURE_ON_PHASE)", "1_RunProcEqualPriority", PRIO_PWM, True, "adc", "loop", "default", "sw",
        ["The ADC ISR runs after the PWM ISR returns: Run_Proc on this tick always comes first,",
         "so the PI runs on the next tick. One duty writer per tick, the LOOP."]),
    Seq("2", "ON_PHASE in the ADC ISR, equal priority (MOTOR_I_FEEDBACK_ON_PHASE)", "2_OnPhaseEqualPriority", PRIO_PWM, True, "adc", "adc", "default", "sw",
        ["The ADC ISR runs after the PWM ISR returns, so the PI waits for the whole PWM ISR.",
         "The PWM ISR and the PI land in the same period: the analog period's load.",
         "The 1 ms SysTick thread (priority 2, exception 15) also runs first when it is pending."]),
    Seq("3", "ON_PHASE in the ADC ISR, ADC at higher priority", "3_OnPhaseHigherPriority", PRIO_HIGH, True, "adc", "adc", "default", "sw",
        ["The ADC ISR preempts the PWM ISR wherever it is. Hazards:",
         "- the Park takes the held angle: the previous tick's if the LOOP has not reached SetTheta,",
         "- a state transition in progress in the PWM ISR: ON_PHASE dispatches to a half-entered state,",
         "- the LOOP's write after it can overwrite its duty with the old Vdq."]),
    Seq("4", "Run_Proc, ADC at higher priority: capture preempts, the LOOP takes a fresh sample", "4_RunProcHigherPriority", PRIO_HIGH, True, "loop", "loop", "default", "sw",
        ["The ADC ISR only captures; the LOOP Parks at this tick's angle and runs the PI if the batch",
         "landed before its take, else on the next tick, as in 1. One duty writer.",
         "Needs the LOOP's take and Park atomic against the capture (mask the ADC IRQ for a few us)."]),
    Seq("5", "Run_Proc polls the ADC (suggestion)", "5_RunProcPoll", PRIO_PWM, False, "loop", "poll", "default", "sw",
        ["The PWM ISR waits for the batch at the take, captures and Parks it from the results buffer.",
         "No second ISR, no race. Spins only if the LOOP reaches the take before the batch lands.",
         "The ADC ISR keeps the other channels; its I set work moves into the PWM ISR."]),
    Seq("6", "Hardware trigger ahead of the top, FOC first in the PWM ISR (suggestion)", "6_TriggerAheadFocFirst", PRIO_PWM, True, "loop", "loop", "foc_first", "hw",
        [f"FTM2 starts the ADC {HW_LEAD:.1f} us before the top (init trigger + SIM_SOPT0[DELAY], or a match trigger),",
         "so the batch and its capture finish in idle time before the PWM ISR.",
         "The PWM ISR runs SetTheta, Park, PI and the write first, then the requests for the next tick",
         "(the PI takes requests one tick old). ADTRG is ADC-wide: the other channels need the trigger too."]),
]


class Sim:
    def __init__(self, seq, k, t0=-200.0, t1=200.0):
        self.seq, self.k = seq, k
        self.d = {name: value * k for name, value in BASE.items()}
        self.t, self.t1 = t0, t1
        self.jobs, self.running = [], None
        self.events, self.order = [], itertools.count()
        self.trace = []                          # [lane, label, kind, t_start, t_end, batch]
        self.token = None                        # the batch captured and not yet taken
        self.sensor_tick, self.held_tick = None, None
        self.vdq_batch = None
        self.batches = {}                        # tick -> start, done, park_tick, park_at, pi_end
        self.writes = []                         # (time, batch of the Vdq written)
        self.pwm_delay = {}                      # tick -> PWM ISR start - TOF
        for n in range(int(math.floor(t0 / T)), int(math.ceil(t1 / T)) + 1):
            tick = n * T
            analog = (n % 2 == 0)
            self.at(tick, lambda tick=tick, analog=analog: self.release("PWM ISR", PRIO_PWM, IRQ_FTM2, self.pwm_isr(tick, analog), tick))
            if analog and seq.trigger == "hw":
                self.at(tick - HW_LEAD, lambda tick=tick: self.start_adc(tick - HW_LEAD, tick))

    # Events and jobs
    def at(self, time, fn):
        heapq.heappush(self.events, (time, next(self.order), fn))

    def release(self, lane, prio, irq, gen, tag):
        self.jobs.append(dict(lane=lane, prio=prio, irq=irq, gen=gen, seg=None, release=self.t, tag=tag, started=None))

    def start_adc(self, time, tick):
        self.batches[tick] = dict(start=time, done=time + CONV_DONE)
        if self.seq.adc_isr:
            self.at(time + CONV_DONE, lambda tick=tick: self.release("ADC ISR", self.seq.adc_prio, IRQ_ADC, self.adc_isr(tick), tick))

    def landed(self, tick):
        return tick in self.batches and self.t >= self.batches[tick]["done"] - 1e-9

    # Effects
    def capture(self, tick):
        self.token = tick

    def park(self, tick):
        self.batches[tick].update(park_tick=self.held_tick, park_at=self.t, park_lane=self.running["lane"])

    def take(self):
        tick, self.token = self.token, None
        return tick

    def pi(self, tick):
        self.batches[tick]["pi_end"] = self.t
        self.vdq_batch = tick

    def write(self):
        self.writes.append((self.t, self.vdq_batch))

    # The ISRs, as segments: (label, kind, duration or a condition, effect)
    def seg(self, label, kind, dur, effect=None):
        return dict(label=label, kind=kind, left=dur, effect=effect, until=None)

    def until(self, label, kind, cond, effect=None):
        return dict(label=label, kind=kind, left=math.inf, effect=effect, until=cond)

    def loop_take(self, tick, analog):
        s, d = self.seq, self.d
        if s.pi_in == "poll" and analog:
            yield self.until("spin", "spin", lambda: self.landed(tick))
            yield self.seg("VSOURCE, I", "capture", d["vsource"] + d["icapture"], lambda: self.capture(tick))
        if s.pi_in in ("loop", "poll") and self.token is not None:
            b = self.take()
            if s.park_in == "loop":
                yield self.seg("Park", "capture", d["park"], lambda b=b: self.park(b))
            yield self.seg("PI", "pi", d["pi"], lambda b=b: self.pi(b))

    def pwm_isr(self, tick, analog):
        s, d = self.seq, self.d
        yield self.seg("entry", "shell", d["entry"])
        if analog and s.trigger == "sw":
            yield self.seg("ADC start", "shell", d["adc_start"], lambda: self.start_adc(self.t, tick))
        yield self.seg("sensor", "theta", d["sensor"], lambda: setattr(self, "sensor_tick", tick))
        yield self.seg("state machine", "shell", d["sm"])
        if s.order == "default":
            yield self.seg("requests", "req", d["requests"])
            yield self.seg("SetTheta", "theta", d["theta"], lambda: setattr(self, "held_tick", self.sensor_tick))
            yield from self.loop_take(tick, analog)
            yield self.seg("out", "out", d["out"], self.write)
        else:
            yield self.seg("SetTheta", "theta", d["theta"], lambda: setattr(self, "held_tick", self.sensor_tick))
            yield from self.loop_take(tick, analog)
            yield self.seg("out", "out", d["out"], self.write)
            yield self.seg("requests", "req", d["requests"])
        yield self.seg("exit", "shell", d["exit"])

    def adc_isr(self, tick):
        s, d = self.seq, self.d
        yield self.seg("entry", "shell", d["adc_entry"])
        yield self.seg("VSOURCE", "capture", d["vsource"])
        yield self.seg("I", "capture", d["icapture"], lambda: self.capture(tick))
        if s.park_in == "adc":
            yield self.seg("Park", "capture", d["park"], lambda: self.park(tick))
        if s.pi_in == "adc":
            yield self.seg("dispatch", "shell", d["dispatch"])
            if self.token is not None:
                b = self.take()
                yield self.seg("PI", "pi", d["pi"], lambda b=b: self.pi(b))
                yield self.seg("out", "out", d["out"], self.write)
        yield self.seg("exit", "shell", d["adc_exit"])

    # Scheduler
    def pick(self):
        live = [j for j in self.jobs if j["gen"] is not None]
        if not live:
            return None
        if self.running in live and not any(j["prio"] < self.running["prio"] for j in live):
            return self.running
        return min(live, key=lambda j: (j["prio"], j["irq"], j["release"]))

    def next_seg(self, job):
        job["seg"] = next(job["gen"], None)
        if job["seg"] is None:
            job["gen"] = None

    def run(self):
        while self.t < self.t1:
            while self.events and self.events[0][0] <= self.t + 1e-9:
                heapq.heappop(self.events)[2]()
            job = self.pick()
            self.running = job
            if job is not None:
                if job["started"] is None:
                    job["started"] = self.t
                    if job["lane"] == "PWM ISR":
                        self.pwm_delay[job["tag"]] = self.t - job["release"]
                    self.next_seg(job)
                while job["gen"] is not None and job["seg"]["until"] is not None and job["seg"]["until"]():
                    self.end_seg(job)
                if job["gen"] is not None:
                    sg = job["seg"]
                    last = self.trace[-1] if self.trace else None
                    if last and last[0] == job["lane"] and last[1] == sg["label"] and abs(last[4] - self.t) < 1e-9 and last[5] == job["tag"]:
                        last[4] = self.t + DT
                    else:
                        self.trace.append([job["lane"], sg["label"], sg["kind"], self.t, self.t + DT, job["tag"]])
                    sg["left"] -= DT
                    if sg["left"] <= 1e-9:
                        self.t += DT
                        self.end_seg(job)
                        continue
            self.t += DT
        return self

    def end_seg(self, job):
        if job["seg"]["effect"] is not None:
            job["seg"]["effect"]()
        self.next_seg(job)

    # Results
    def result(self, tick=0.0):
        b = self.batches.get(tick, {})
        sample = b.get("start", math.nan) + SAMPLE_OFFSET
        write = next((w for w, wb in self.writes if wb == tick and w >= b.get("pi_end", math.inf) - 1e-9), math.nan)
        load = (math.floor(write / LOAD + 1e-9) + 1) * LOAD if not math.isnan(write) else math.nan
        busy = [sum(max(0.0, min(r[4], lo + T) - max(r[3], lo)) for r in self.trace) for lo in (tick, tick + T)]
        park_tick = b.get("park_tick")
        return dict(sample=sample, park_at=b.get("park_at", math.nan), park_tick=park_tick, park_lane=b.get("park_lane"),
                    park_offset=(park_tick - sample) if park_tick is not None else math.nan,
                    pi_end=b.get("pi_end", math.nan), write=write, load=load, sample_to_load=load - sample,
                    busy_analog=busy[0], busy_off=busy[1],
                    pwm_delay=max(self.pwm_delay.get(tick + n * T, 0.0) for n in (0, 1, 2)))


def plot_sequence(seq, sim, path):
    r = sim.result()
    t_lo, t_hi = -30.0, 130.0
    fig, (ax_c, ax_l) = plt.subplots(2, 1, figsize=(13, 6.8), sharex=True, gridspec_kw={"height_ratios": [1, 2.2]})

    # Counter
    ts = [t_lo + i * 0.1 for i in range(int((t_hi - t_lo) / 0.1) + 1)]
    ax_c.plot(ts, [1 - abs(2 * (((t + 25) % 50) / 50) - 1) for t in ts], color="black", linewidth=1)
    for n in range(int(t_lo // LOAD), int(t_hi // LOAD) + 1):
        lp = n * LOAD
        if t_lo <= lp <= t_hi:
            top = (n % 2 == 0)
            ax_c.plot(lp, 1.0 if top else 0.0, marker="v" if top else "^", color="gray", markersize=6)
            ax_c.text(lp, 1.12 if top else -0.3, ("TOF" if top else "bottom"), ha="center", fontsize=7, color="gray")
    if not math.isnan(r["load"]):
        ax_c.plot(r["load"], 1.0 if r["load"] % T == 0 else 0.0, marker="*", color="green", markersize=16)
        ax_c.annotate("batch 0's PI output loads", (r["load"], 0.5), (r["load"] + 2, 0.55), fontsize=8, color="green")
    ax_c.set_ylim(-0.45, 1.3)
    ax_c.set_yticks([])
    ax_c.set_ylabel("FTM2\ncounter")
    ax_c.set_title(f"{seq.key}. {seq.title}", fontsize=11)

    # Lanes
    lanes = {"ADC": 3.0, "ADC ISR": 2.0, "PWM ISR": 1.0}
    for name, start, width in CONV:
        for tick, b in sim.batches.items():
            x = b["start"] + start
            if t_lo <= x <= t_hi:
                ax_l.broken_barh([(x, width)], (lanes["ADC"] - 0.3, 0.6), facecolors="#cfd8dc" if name == "VSOURCE" else "#90a4ae", edgecolor="white")
                ax_l.text(x + width / 2, lanes["ADC"], {"VSOURCE": "VS"}.get(name, name) if tick == 0 else "", ha="center", va="center", fontsize=6)
                if name == "IB":
                    ax_l.plot(x, lanes["ADC"] + 0.42, marker="v", color="black", markersize=5)
    for lane, label, kind, a, b, tag in sim.trace:
        if b < t_lo or a > t_hi:
            continue
        ax_l.broken_barh([(a, b - a)], (lanes[lane] - 0.3, 0.6), facecolors=KIND_COLOR[kind], edgecolor="#555555", linewidth=0.4,
                         hatch="///" if kind == "spin" else None)
        if b - a >= 2.4:
            ax_l.text((a + b) / 2, lanes[lane], label, ha="center", va="center", fontsize=6)
    for n in range(int(t_lo // T), int(t_hi // T) + 1):
        if t_lo <= n * T <= t_hi:
            ax_l.axvline(n * T, color="black", linewidth=0.6, linestyle=":")
            ax_l.axvline(n * T + LOAD, color="gray", linewidth=0.4, linestyle=":")
            ax_l.text(n * T + 0.5, 3.55, "analog tick" if n % 2 == 0 else "off tick", fontsize=7)

    # Batch 0: sample -> Park -> PI -> load
    y = 0.15
    if not math.isnan(r["load"]):
        ax_l.annotate("", (r["load"], y), (r["sample"], y), arrowprops=dict(arrowstyle="->", color="green", linewidth=1.5))
        ax_l.text((r["sample"] + r["load"]) / 2, y - 0.25, f"batch 0: sample -> load {r['sample_to_load']:.1f} us", ha="center", fontsize=8, color="green")
    if not math.isnan(r["park_at"]):
        y_park = lanes[r["park_lane"]] + 0.38
        ax_l.plot(r["park_at"], y_park, marker="D", color="#1565c0", markersize=5)
        ax_l.text(r["park_at"] + 0.8, y_park + 0.05, f"batch 0 Parked at the angle of t = {r['park_tick']:.0f}", fontsize=7, color="#1565c0")
    ax_l.set_yticks(list(lanes.values()))
    ax_l.set_yticklabels(list(lanes.keys()))
    ax_l.set_ylim(-0.3, 3.8)
    ax_l.set_xlim(t_lo, t_hi)
    ax_l.set_xlabel("Time from the analog tick's counter top (us)")

    total = AGE_US + r["sample_to_load"] + HOLD_HALF_US
    lines = [f"sample -> load {r['sample_to_load']:.1f} us; delay to actuation {total:.0f} us at hold tau 10 us",
             f"Park angle from t = {r['park_tick']:.0f}: {r['park_offset']:+.1f} us from the sample",
             f"CPU in the analog period {r['busy_analog']:.1f} of 50 us, the off period {r['busy_off']:.1f}; PWM ISR start delay up to {r['pwm_delay']:.1f} us"] + seq.notes
    ax_l.text(0.995, -0.2, "\n".join(lines), transform=ax_l.transAxes, ha="right", va="top", fontsize=7.5,
              bbox=dict(boxstyle="round", facecolor="white", edgecolor="#999999"))
    fig.legend(handles=[Patch(facecolor=c, edgecolor="#555555", hatch="///" if k == "spin" else None, label=KIND_LABEL[k]) for k, c in KIND_COLOR.items()],
               loc="lower left", bbox_to_anchor=(0.01, 0.035), fontsize=7, ncol=2)
    fig.text(0.01, 0.005, "Durations are estimates (k = 1), not measured. See PwmSampleSequenceSummary.png for the sweep.", fontsize=7, color="gray")
    fig.tight_layout(rect=(0, 0.2, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)


here = os.path.dirname(os.path.abspath(__file__))
nominal = {s.key: Sim(s, 1.0).run() for s in SEQS}

print("Nominal durations (k = 1), us: " + ", ".join(f"{n} {v}" for n, v in BASE.items()))
print(f"{'seq':3s} {'sample':>7s} {'Park from':>9s} {'offset':>7s} {'PI end':>7s} {'write':>7s} {'load':>6s} {'smp->load':>9s} {'actuation':>9s} {'CPU analog':>10s} {'off':>5s} {'PWM delay':>9s}")
for s in SEQS:
    r = nominal[s.key].result()
    print(f"{s.key:3s} {r['sample']:7.1f} {r['park_tick']:9.0f} {r['park_offset']:+7.1f} {r['pi_end']:7.1f} {r['write']:7.1f} {r['load']:6.0f}"
          f" {r['sample_to_load']:9.1f} {AGE_US + r['sample_to_load'] + HOLD_HALF_US:9.0f} {r['busy_analog']:10.1f} {r['busy_off']:5.1f} {r['pwm_delay']:9.1f}")

for s in SEQS:
    if s.file:
        plot_sequence(s, nominal[s.key], os.path.join(here, f"PwmSampleSequence_{s.file}.png"))

# Sweep the speed factor
ks = [0.4 + 0.02 * i for i in range(61)]
sweep = {s.key: [Sim(s, k).run().result() for k in ks] for s in SEQS}
print("\nsample -> load (us) by speed factor k")
print("k     " + " ".join(f"{s.key:>6s}" for s in SEQS))
for i, k in enumerate(ks):
    if i % 5 == 0:
        print(f"{k:4.2f}  " + " ".join(f"{sweep[s.key][i]['sample_to_load']:6.1f}" for s in SEQS))

fig, (ax_a, ax_b, ax_c) = plt.subplots(3, 1, figsize=(11, 12.5), sharex=True)
styles = {"0": ("black", ":"), "1": ("tab:blue", "-"), "2": ("tab:orange", "-"), "3": ("tab:red", "--"), "4": ("tab:purple", "--"), "5": ("tab:green", "-"), "6": ("tab:brown", "-")}
for s in SEQS:
    color, ls = styles[s.key]
    label = f"{s.key}. {s.title}"
    ax_a.step(ks, [r["sample_to_load"] for r in sweep[s.key]], where="mid", color=color, linestyle=ls, label=label)
    ax_b.plot(ks, [r["busy_analog"] for r in sweep[s.key]], color=color, linestyle=ls)
    ax_c.plot(ks, [r["park_offset"] for r in sweep[s.key]], color=color, linestyle=ls)
ax_a.axvline(1.0, color="gray", linewidth=0.8)
for load, name in ((25, "same period's bottom"), (50, "next top"), (75, "next bottom"), (100, "top after")):
    ax_a.text(1.605, load - SAMPLE_OFFSET, f"loads at the {name}", fontsize=7, va="center", color="gray")
ax_a.text(1.605, 25 + HW_LEAD - SAMPLE_OFFSET + 0.5, "6: its sample is ahead of the top", fontsize=7, va="bottom", color="tab:brown")
ax_a.set_ylabel("Sample -> load (us)")
ax_a.set_title("Current-loop sequences on KE, by a factor on every ISR duration (1 = the estimate)")
ax_a.grid()
fig.legend(*ax_a.get_legend_handles_labels(), loc="lower center", ncol=1, fontsize=8, bbox_to_anchor=(0.5, 0.0))
ax_b.axhline(T, color="red", linewidth=0.8)
ax_b.axvline(1.0, color="gray", linewidth=0.8)
ax_b.text(0.41, T + 0.5, "the period: past it the next PWM ISR starts late", fontsize=7, color="red")
ax_b.set_ylabel("CPU in the analog period (us)")
ax_b.grid()
ax_c.axvline(1.0, color="gray", linewidth=0.8)
ax_c.set_ylabel("Park angle time - sample time (us)")
ax_c.set_xlabel("Speed factor k on the estimated durations (lower is faster)")
ax_c.text(0.41, -48, "-50: the previous tick's angle, Parked before this tick's SetTheta", fontsize=7)
ax_c.grid()
fig.tight_layout(rect=(0, 0.13, 0.86, 1))
fig.savefig(os.path.join(here, "PwmSampleSequenceSummary.png"), dpi=100)
plt.show()
