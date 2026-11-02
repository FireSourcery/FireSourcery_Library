"""
KE current-loop sequences: where the I sample is captured, Parked and run through the PI, and when its output loads.

Three thread placements, each with its variants:
    1. The ADC ISR captures the I set, and the PWM ISR's LOOP takes it and runs the PI. 1a the take finds it, 1b it misses, 1c the LOOP waits.
    2. The ADC ISR also runs the PI, at the I set's completion. 2a at the PWM ISR's priority, 2b above it.
       The LOOP skips its out on the I Sample Cycle, a duplicate of the ADC ISR's. The code doesn't support that yet: its LOOP writes every cycle.
    3. Hardware Triggered: the trigger starts the I set to complete before the PWM ISR, whose LOOP takes it and runs the PI.
       Two cycle forms: 3A alternating, an I Feedback Cycle every 2nd PWM cycle and the requests on the off cycle; 3B the PI every cycle.
       Variations: -r the requests every cycle (3A-r is built), -w the take waits for the capture, -v VBus on the main thread
       [VBUS_CAPTURE_ON_MAIN]. The start is calibrated to the take, or for 3A-v to the bottom: the latest start whose write makes it.
Terms: the I Sample Cycle marks the set and starts or arms it; the I Feedback Cycle takes it and runs the PI; an off cycle is not an
I Feedback Cycle. Before 2026-11-02: analog tick, take tick, off tick; 3a, 3b, 3c for 3A, 3A-r, 3B.
One timeline per variant (PwmSampleSequence_<variant>_<name>.png), and a summary across a CPU speed factor (PwmSampleSequenceSummary.png).
CurrentLoopSequencing.md and CurrentSampling.md describe the hardware and the code paths.

Model
    FTM2 center-aligned, T = 50 us. Counter top (TOF, PWM ISR) at t = 0, 50, ...; bottom at 25 + 50n.
    Both loading points are enabled: a duty written loads at the next top or bottom after the write.
    The current loop runs at 10 kHz: the cycles at t = 0, 100, ... (every 2nd period) are scheduled to take a set; 3B every period.
    Batch b is the set the cycle at t = b is scheduled to take: started in it in software, or ahead of it by the trigger.
    ADC: 10 MHz ADCK, VSOURCE then IA, IB, IC; IB samples 4.5 us after the start, the batch completes at 8.5 us.
    NVIC: lower priority value preempts; equal priority does not, and a tie between pending IRQs goes to the lower number (ADC 15, FTM2 19).
    The sample's place in the period does not change its age through the sense filter (CurrentSenseFilter.md), so the comparison
    is sample -> load. Delay to actuation = age (~96 us at hold tau 10 us) + sample -> load + 50 us (half the 2-period hold).

Durations are estimates at k = 1: instruction counts of the linked KE06 image at ~32 ns each (50 MHz, CPI ~1.6). Not measured.
The summary sweeps k, a factor on every duration, because which load point each sequence makes depends on it.
ASCII console output. Writes the PNGs beside this file.
"""
import copy
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
HW_LEAD = CONV_DONE + BASE["adc_entry"] + BASE["vsource"] + BASE["icapture"] + BASE["adc_exit"] + 0.5  # path 3: done before the top

KIND_COLOR = {"shell": "#d9d9d9", "theta": "#81c784", "req": "#b39ddb", "capture": "#64b5f6", "pi": "#e57373", "out": "#ffb74d", "spin": "#f5f5f5"}
KIND_LABEL = {"shell": "entry, dispatch, exit", "theta": "angle: sensor, SetTheta", "req": "requests: ramp, field weakening",
              "capture": "VSOURCE, I capture, Park", "pi": "PI", "out": "inverse Park, SVPWM, write", "spin": "spin, waiting for the ADC"}


class Seq:
    def __init__(self, key, title, file, adc_prio, adc_isr, park_in, pi_in, order, trigger, notes, vbus="adc", lead=None, requests="every", divider=2):
        self.key, self.title, self.file = key, title, file
        self.adc_prio, self.adc_isr = adc_prio, adc_isr      # the ADC ISR handles the I set
        self.park_in, self.pi_in = park_in, pi_in            # 'adc' / 'loop' ; 'loop' / 'adc' / 'wait'
        self.order, self.trigger = order, trigger            # 'default' / 'foc_first' ; 'sw' / 'hw'
        self.notes = notes
        self.vbus = vbus                                     # 'adc': VBus captured in the ADC ISR; 'main': on the main thread [VBUS_CAPTURE_ON_MAIN]
        self.lead = HW_LEAD if lead is None else lead        # 'hw': the set's start before the top
        self.requests = requests                             # 'every': the ramp and field weakening every cycle; 'off': on the off cycle only
        self.divider = divider                               # PWM ticks per I set: 2 on KE (10 kHz), 1 for the PI every cycle


SEQS = [
    Seq("1a", "PI in the PWM ISR, the take finds the set. The ADC ISR preempts before the take", "1a_TakeHit", PRIO_HIGH, True, "loop", "loop", "default", "sw",
        ["The ADC ISR only captures. The LOOP's take Parks at this tick's angle and runs the PI",
         "if the set landed before the take, else on the next tick: 1b. One duty writer."]),
    Seq("1b", "PI in the PWM ISR, the take misses. The ADC ISR at the PWM ISR's priority", "1b_TakeMiss", PRIO_PWM, True, "loop", "loop", "default", "sw",
        ["1a's miss: the capture comes after this tick's take. At the PWM ISR's priority the ADC ISR",
         "runs after the PWM ISR returns, so it always misses. The next tick's LOOP takes the set,",
         "Parked at its own angle. One duty writer per tick, the LOOP."]),
    Seq("1c", "PI in the PWM ISR, the LOOP waits at the take. The ADC ISR preempts and captures", "1c_TakeWait", PRIO_HIGH, True, "loop", "wait", "default", "sw",
        ["The LOOP spins at its take until the ADC ISR, above it, has captured the set. It never misses.",
         "Spins only if the LOOP reaches the take before the set lands. One duty writer."]),
    Seq("2a", "PI in the ADC ISR, at the PWM ISR's priority (MOTOR_I_FEEDBACK_ON_PHASE)", "2a_OnPhaseEqualPriority", PRIO_PWM, True, "adc", "adc", "default", "sw",
        ["The ADC ISR runs after the PWM ISR returns, so the PI waits for the whole PWM ISR.",
         "The PWM ISR and the PI land in the same period: the I Sample Cycle's load.",
         "The 1 ms SysTick thread (priority 2, exception 15) also runs first when it is pending.",
         "The LOOP skips its out on the I Sample Cycle, a duplicate of the ADC ISR's: not supported in the code yet."]),
    Seq("2b", "PI in the ADC ISR, above the PWM ISR", "2b_OnPhasePreempt", PRIO_HIGH, True, "adc", "adc", "default", "sw",
        ["The ADC ISR preempts the PWM ISR wherever it is. Hazards:",
         "- the Park takes the held angle: the previous tick's if the LOOP has not reached SetTheta,",
         "- a state transition in progress in the PWM ISR: ON_PHASE dispatches to a half-entered state.",
         "The LOOP skips its out on the I Sample Cycle, so it can't overwrite the ADC ISR's duty: not supported in the code yet."]),
    Seq("3A", "Hardware Triggered, alternating: the requests on the off cycle only", "3A_Alternating", PRIO_HIGH, True, "loop", "loop", "default", "hw",
        [], requests="off"),  # lead and notes: calibrated below, once Sim is defined
    Seq("3A-r", "Hardware Triggered, alternating, the requests every cycle (built)", "3A_RequestsEveryCycle", PRIO_HIGH, True, "loop", "loop", "default", "hw",
        [f"FTM2 starts the ADC {HW_LEAD:.1f} us before the top (init trigger + SIM_SOPT0[DELAY], or a match trigger),",
         "so the batch and its capture finish in idle time before the PWM ISR, and the take never misses.",
         "The requests run before the take every cycle, so the write misses the bottom; FOC first made it (ORDERS below).",
         "ADTRG is ADC-wide: the other channels need the trigger too."]),
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
        self.take_at = {}                        # tick -> the time the LOOP reaches its take
        for n in range(int(math.floor(t0 / T)), int(math.ceil(t1 / T)) + 1):
            tick = n * T
            analog = (n % seq.divider == 0)
            self.at(tick, lambda tick=tick, analog=analog: self.release("PWM ISR", PRIO_PWM, IRQ_FTM2, self.pwm_isr(tick, analog), tick))
            if analog and seq.trigger == "hw":
                self.at(tick - seq.lead, lambda tick=tick: self.start_adc(tick - self.seq.lead, tick))

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
        self.batches[tick]["captured"] = self.t

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
        self.take_at.setdefault(tick, self.t)
        if s.pi_in == "wait" and analog:
            yield self.until("spin", "spin", lambda: self.token == tick)  # the ADC ISR, above, captures
        if s.pi_in in ("loop", "wait") and self.token is not None:
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
        requests = (s.requests == "every") or not analog
        out = not (s.pi_in == "adc" and analog)  # path 2: the ADC ISR writes the I Sample Cycle's duty, so the LOOP's would be a duplicate. Not in the code
        if s.order == "default":
            if requests:
                yield self.seg("requests", "req", d["requests"])
            yield self.seg("SetTheta", "theta", d["theta"], lambda: setattr(self, "held_tick", self.sensor_tick))
            yield from self.loop_take(tick, analog)
            if out:
                yield self.seg("out", "out", d["out"], self.write)
        else:
            yield self.seg("SetTheta", "theta", d["theta"], lambda: setattr(self, "held_tick", self.sensor_tick))
            yield from self.loop_take(tick, analog)
            if out:
                yield self.seg("out", "out", d["out"], self.write)
            if requests:
                yield self.seg("requests", "req", d["requests"])
        yield self.seg("exit", "shell", d["exit"])

    def adc_isr(self, tick):
        s, d = self.seq, self.d
        yield self.seg("entry", "shell", d["adc_entry"])
        if s.vbus == "adc":
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
        wait = max(0.0, b.get("captured", math.inf) - self.take_at.get(tick, math.inf)) if tick in self.take_at else math.nan
        return dict(wait=wait, hit=(park_tick == tick), sample=sample, park_at=b.get("park_at", math.nan), park_tick=park_tick, park_lane=b.get("park_lane"),
                    park_offset=(park_tick - sample) if park_tick is not None else math.nan,
                    pi_end=b.get("pi_end", math.nan), write=write, load=load, sample_to_load=load - sample,
                    busy_analog=busy[0], busy_off=busy[1],
                    pwm_delay=max(self.pwm_delay.get(tick + n * T, 0.0) for n in (0, 1, 2)))


def variant(seq, **changes):
    v = copy.copy(seq)
    for name, value in changes.items():
        setattr(v, name, value)
    return v


def at_start(seq, k, lead):
    """Tick 0's result with the set started [lead] before the top. Its wait: how long the take would have to wait for the capture. Theory only."""
    return Sim(variant(seq, lead=lead), k, t0=-100.0, t1=150.0).run().result()


def calibrated_lead(seq, k):
    """The latest start, as the lead before the top, whose set the take finds on its own tick at k, so no wait. 1 us scan, then 0.1 us."""
    coarse = [25.0 - i for i in range(41)]  # the init trigger's bottom, 25 us ahead, to 15 us after the top
    last = coarse[0]
    for lead in coarse:
        if not at_start(seq, k, lead)["hit"]:
            break
        last = lead
    for i in range(1, 10):
        lead = last - 0.1 * i
        if not at_start(seq, k, lead)["hit"]:
            break
        last = lead
    return round(last, 1)


def bottom_lead(seq, k):
    """The latest start, as the lead before the top, whose I Feedback Cycle writes before the bottom at k: the load at 25 us. 1 us scan, then 0.1 us."""
    def makes(lead):
        r = at_start(seq, k, lead)
        return r["hit"] and r["load"] == LOAD
    coarse = [25.0 - i for i in range(26)]  # the init trigger's bottom, 25 us ahead, to the top
    last = None
    for lead in coarse:
        if not makes(lead):
            break
        last = lead
    if last is None:
        return None
    for i in range(1, 10):
        lead = last - 0.1 * i
        if not makes(lead):
            break
        last = lead
    return round(last, 1)


def bottom_until(seq):
    """The largest k, by 0.01 from 1, at which the I Feedback Cycle's write still makes the bottom."""
    k = 1.0
    while Sim(seq, k + 0.01, t0=-100.0, t1=150.0).run().result()["load"] == LOAD:
        k += 0.01
    return round(k, 2)


def start_text(lead):
    return f"{abs(lead):.1f} us {'before' if lead >= 0 else 'after'} the top"


SEQ_3A = next(s for s in SEQS if s.key == "3A")
SEQ_3A.lead = calibrated_lead(SEQ_3A, 1.0)
SEQ_3A.notes = [f"FTM2 starts the ADC {start_text(SEQ_3A.lead)}: the latest start whose set the take finds without waiting, at k = 1.",
                "The I Feedback Cycle runs no requests. The off cycle steps the ramp and field weakening, at 10 kHz, then the inverse Park and the write.",
                "A faster cycle reaches the take first: the miss. The off cycle takes the set. No busy wait (3A-w waits)."]

# 3A-v: VBus on the main thread [VBUS_CAPTURE_ON_MAIN], the ADC ISR without VSOURCE's capture and divide, and the start earlier, so the
# I Feedback Cycle's write makes the bottom. 3A's own bottom start, VBus in the ADC ISR, for comparison
LEAD_3A_BOTTOM = bottom_lead(SEQ_3A, 1.0)
SEQ_3AV = variant(SEQ_3A, key="3A-v", title="Hardware Triggered, alternating, VBus on the main thread (VBUS_CAPTURE_ON_MAIN): the write makes the bottom",
                  file="3A_VBusOnMain", vbus="main")
SEQ_3AV.lead = bottom_lead(SEQ_3AV, 1.0)
SEQ_3AV.notes = [f"FTM2 starts the ADC {start_text(SEQ_3AV.lead)}: the latest start whose I Feedback Cycle writes before the bottom, at k = 1.",
                 f"VSOURCE still converts, but the main thread captures it and divides: the ADC ISR is {BASE['vsource']:.1f} us shorter.",
                 f"With VBus in the ADC ISR, 3A's latest start for the bottom is {start_text(LEAD_3A_BOTTOM)}.",
                 f"The I Feedback Cycle runs no requests: its PWM ISR writes {sum(BASE[n] for n in ('entry', 'sensor', 'sm', 'theta', 'park', 'pi', 'out')):.1f} us in,"
                 " and the ADC ISR can overrun the top by the rest of the 25.",
                 f"A slower cycle, from k {bottom_until(SEQ_3AV) + 0.01:.2f}, misses the bottom: the cliff to the next top. A faster one finds the set sooner."]

# 3B: the PI every cycle. S32's form; its durations are unmeasured, so it runs at half KE's (k 0.5), on KE's ADC timing
K_3B = 0.5
SEQ_3B = Seq("3B", "Hardware Triggered, the PI every cycle", "3B_PiEveryCycle", PRIO_HIGH, True, "loop", "loop", "default", "hw", [], divider=1)
SEQ_3B.lead = calibrated_lead(SEQ_3B, K_3B)
SEQ_3B.notes = [f"At half KE's durations (k = {K_3B}), a stand-in for S32's, unmeasured, on KE's ADC timing.",
                f"FTM2 starts the ADC {start_text(SEQ_3B.lead)}: the latest start whose set the take finds without waiting, at that k.",
                "Every PWM cycle is an I Feedback Cycle, and an I Sample Cycle for the next: the requests, the take, the PI,",
                "and the write before the bottom. No busy wait (3B-w waits)."]


def plot_sequence(seq, sim, path, k=1.0):
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
            ax_l.text(n * T + 0.5, 3.55, ("I Feedback Cycle" if seq.trigger == "hw" else "I Sample Cycle") if n % seq.divider == 0 else "off cycle", fontsize=7)

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
    ax_l.set_xlabel("Time from the counter top of the cycle scheduled to take batch 0 (us)")

    total = AGE_US + r["sample_to_load"] + HOLD_HALF_US
    lines = [f"sample -> load {r['sample_to_load']:.1f} us; delay to actuation {total:.0f} us at hold tau 10 us",
             f"Park angle from t = {r['park_tick']:.0f}: {r['park_offset']:+.1f} us from the sample",
             f"CPU in batch 0's cycle {r['busy_analog']:.1f} of 50 us, the next {r['busy_off']:.1f}; PWM ISR start delay up to {r['pwm_delay']:.1f} us"] + seq.notes
    ax_l.text(0.995, -0.2, "\n".join(lines), transform=ax_l.transAxes, ha="right", va="top", fontsize=7.5,
              bbox=dict(boxstyle="round", facecolor="white", edgecolor="#999999"))
    fig.legend(handles=[Patch(facecolor=c, edgecolor="#555555", hatch="///" if k == "spin" else None, label=KIND_LABEL[k]) for k, c in KIND_COLOR.items()],
               loc="lower left", bbox_to_anchor=(0.01, 0.035), fontsize=7, ncol=2)
    fig.text(0.01, 0.005, f"Durations are estimates (k = {k:g}), not measured. See PwmSampleSequenceSummary.png for the sweep.", fontsize=7, color="gray")
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
plot_sequence(SEQ_3AV, Sim(SEQ_3AV, 1.0).run(), os.path.join(here, f"PwmSampleSequence_{SEQ_3AV.file}.png"))
plot_sequence(SEQ_3B, Sim(SEQ_3B, K_3B).run(), os.path.join(here, f"PwmSampleSequence_{SEQ_3B.file}.png"), k=K_3B)

# Sweep the speed factor
ks = [0.4 + 0.02 * i for i in range(61)]
sweep = {s.key: [Sim(s, k).run().result() for k in ks] for s in SEQS}
print("\nsample -> load (us) by speed factor k")
print("k     " + " ".join(f"{s.key:>6s}" for s in SEQS))
for i, k in enumerate(ks):
    if i % 5 == 0:
        print(f"{k:4.2f}  " + " ".join(f"{sweep[s.key][i]['sample_to_load']:6.1f}" for s in SEQS))

# The LOOP's order: the requests before the take (default), or after the write (FOC first, the PI on requests 1 tick old)
ORDERS = [
    ("1a, FOC first", Seq("1a", "", None, PRIO_HIGH, True, "loop", "loop", "foc_first", "sw", [])),
    ("1c, FOC first", Seq("1c", "", None, PRIO_HIGH, True, "loop", "wait", "foc_first", "sw", [])),
    ("3A-r, FOC first", Seq("3A-r", "", None, PRIO_HIGH, True, "loop", "loop", "foc_first", "hw", [])),
]
# VBus on the main thread [VBUS_CAPTURE_ON_MAIN]: the ADC ISR loses the VBus capture, so the set can start that much later
HW_LEAD_VBUS_MAIN = HW_LEAD - BASE["vsource"]
VBUS_MAIN = [
    ("3A-r, VBus main", Seq("3A-r", "", None, PRIO_HIGH, True, "loop", "loop", "default", "hw", [], vbus="main")),
    ("3A-r, VBus main, later", Seq("3A-r", "", None, PRIO_HIGH, True, "loop", "loop", "default", "hw", [], vbus="main", lead=HW_LEAD_VBUS_MAIN)),
]
order_ks = (0.6, 0.8, 1.0, 1.2, 1.4)
print("\nsample -> load (us) by LOOP order; the write at k = 1")
print(f"{'':22s}" + " ".join(f"k {k:<4}" for k in order_ks) + "  write")
for name, s in [(f"{s.key}, default", s) for s in SEQS if s.key in ("1a", "1c", "3A", "3A-r")] + ORDERS + VBUS_MAIN:
    print(f"{name:22s}" + " ".join(f"{Sim(s, k).run().result()['sample_to_load']:6.1f}" for k in order_ks) + f"  {Sim(s, 1.0).run().result()['write']:5.1f}")


def plot_order_compare(pairs, path, title="Hardware Triggered, the set timed ahead of the top: the LOOP's order", t_hi=60.0):
    """Sequences on one time axis around the take's top, one panel each. A row is (label, seq) at k = 1, or (label, seq, k)."""
    t_lo = -25.0
    lanes = {"ADC": 3.0, "ADC ISR": 2.0, "PWM ISR": 1.0}
    fig, axes = plt.subplots(len(pairs) + 1, 1, figsize=(12 if t_hi <= 60.0 else 15, 1.6 + 2.6 * len(pairs)), sharex=True,
                             gridspec_kw={"height_ratios": [0.7] + [2.0] * len(pairs)})
    ax_c = axes[0]
    ts = [t_lo + i * 0.1 for i in range(int((t_hi - t_lo) / 0.1) + 1)]
    ax_c.plot(ts, [1 - abs(2 * (((t + 25) % 50) / 50) - 1) for t in ts], color="black", linewidth=1)
    for n in range(int(t_lo // LOAD), int(t_hi // LOAD) + 1):
        lp = n * LOAD
        top = (lp % T == 0)
        ax_c.plot(lp, 1.0 if top else 0.0, marker="v" if top else "^", color="gray", markersize=6)
        if lp > t_lo:
            ax_c.text(lp, 1.12 if top else -0.32, "top: loads" if top else "bottom: loads", ha="center", fontsize=7, color="gray")
    ax_c.set_ylim(-0.45, 1.3)
    ax_c.set_yticks([])
    ax_c.set_ylabel("FTM2\ncounter")
    ax_c.set_title(title, fontsize=11)
    for ax, row in zip(axes[1:], pairs):
        label, seq, k = (tuple(row) + (1.0,))[:3]
        sim = Sim(seq, k).run()
        r = sim.result()
        for tick, b in sim.batches.items():
            for name, start, width in CONV:
                x = b["start"] + start
                if t_lo <= x <= t_hi:
                    ax.broken_barh([(x, width)], (lanes["ADC"] - 0.3, 0.6), facecolors="#cfd8dc" if name == "VSOURCE" else "#90a4ae", edgecolor="white")
                    if tick == 0.0:
                        ax.text(x + width / 2, lanes["ADC"], {"VSOURCE": "VS"}.get(name, name), ha="center", va="center", fontsize=6)
        for lane, seg_label, kind, a, e, tag in sim.trace:
            if e < t_lo or a > t_hi:
                continue
            ax.broken_barh([(max(a, t_lo), min(e, t_hi) - max(a, t_lo))], (lanes[lane] - 0.3, 0.6), facecolors=KIND_COLOR[kind], edgecolor="#555555", linewidth=0.4,
                           hatch="///" if kind == "spin" else None)
            if min(e, t_hi) - max(a, t_lo) >= 2.4:
                ax.text((max(a, t_lo) + min(e, t_hi)) / 2, lanes[lane], seg_label, ha="center", va="center", fontsize=6)
        for n in range(-1, int(t_hi // T) + 1):
            ax.axvline(n * T, color="black", linewidth=0.6, linestyle=":")
            ax.axvline(n * T + LOAD, color="gray", linewidth=0.4, linestyle=":")
            if t_hi > 60.0 and 0 <= n * T < t_hi:
                ax.text(n * T + 0.5, 3.42, "I Feedback Cycle" if n % seq.divider == 0 else "off cycle", fontsize=7, color="gray")
        ax.plot([r["write"]] * 2, [lanes["PWM ISR"] - 0.45, lanes["PWM ISR"] + 0.45], color="black", linewidth=1.5)
        ax.text(r["write"] + 0.6, lanes["PWM ISR"] + 0.42, f"write {r['write']:.1f} us", fontsize=7)
        ax.annotate("", (r["load"], 0.25), (r["sample"], 0.25), arrowprops=dict(arrowstyle="->", color="green", linewidth=1.5))
        ax.plot(r["load"], 0.25, marker="*", color="green", markersize=13)
        ax.text((r["sample"] + r["load"]) / 2, -0.05, f"sample -> load {r['sample_to_load']:.1f} us: loads at the {'bottom' if r['load'] % T else 'next top'}",
                ha="center", fontsize=8, color="green")
        ax.set_yticks(list(lanes.values()))
        ax.set_yticklabels(list(lanes.keys()))
        ax.set_ylim(-0.3, 3.6)
        ax.set_title(label, fontsize=9, loc="left")
    axes[-1].set_xlim(t_lo, t_hi)
    axes[-1].set_xlabel("Time from the top whose LOOP takes the set (us)")
    fig.legend(handles=[Patch(facecolor=c, edgecolor="#555555", hatch="///" if k == "spin" else None, label=KIND_LABEL[k]) for k, c in KIND_COLOR.items()],
               loc="lower center", ncol=4, fontsize=7, bbox_to_anchor=(0.5, 0.0))
    fig.text(0.01, 0.005, "Durations are estimates, not measured. k is a factor on all of them, 1 the estimate.", fontsize=7, color="gray")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)


# Hardware Triggered's cycle forms and variations. 3B, the PI every cycle, is S32's: its durations are unmeasured, so it runs at half KE's (k 0.5), on KE's ADC timing
SEQ_3AR = next(s for s in SEQS if s.key == "3A-r")
LEAD_3AR = calibrated_lead(SEQ_3AR, 1.0)
print(f"\ncalibrated starts, the latest the take finds in its I Feedback Cycle: 3A {start_text(SEQ_3A.lead)} at k 1,"
      f" 3A-r {start_text(LEAD_3AR)} at k 1, 3B {start_text(SEQ_3B.lead)} at k {K_3B}")
print(f"the latest starts whose write makes the bottom at k 1: 3A {start_text(LEAD_3A_BOTTOM)} (to k {bottom_until(variant(SEQ_3A, lead=LEAD_3A_BOTTOM))}),"
      f" 3A-v {start_text(SEQ_3AV.lead)} (to k {bottom_until(SEQ_3AV)}); 3A-v 15.0 us before the top to k {bottom_until(variant(SEQ_3AV, lead=15.0))},"
      f" 3A 15.0 us before the top to k {bottom_until(variant(SEQ_3A, lead=15.0))}")

plot_order_compare([(f"3A, alternating: the requests on the off cycle only, started {start_text(SEQ_3A.lead)}, calibrated: the latest the take finds without waiting", SEQ_3A),
                    ("3A, the same start, a cycle 20 % faster (k 0.8): the take misses, and the off cycle takes the set", SEQ_3A, 0.8),
                    ("3A-w, the take waits: the same start and the same faster cycle, no miss", variant(SEQ_3A, pi_in="wait"), 0.8),
                    (f"3A-v, VBus on the main thread: started {start_text(SEQ_3AV.lead)}, the latest whose write makes the bottom", SEQ_3AV),
                    (f"3A-r as built: the requests every cycle, the set started {start_text(SEQ_3AR.lead)}", SEQ_3AR),
                    (f"3A-r calibrated: started {start_text(LEAD_3AR)}", variant(SEQ_3AR, lead=LEAD_3AR)),
                    (f"3B: the PI every cycle, at half KE's durations (S32's, unmeasured) on KE's ADC timing, started {start_text(SEQ_3B.lead)}", SEQ_3B, K_3B)],
                   os.path.join(here, "PwmSampleSequence_3_Options.png"), title="Hardware Triggered: the cycle forms and their variations, the start calibrated to the take", t_hi=110.0)


# Hardware Triggered: the cycle forms and their variations, sample -> load by k. * the take missed; a later cycle took the set
HT_KS = (0.8, 0.9, 1.0, 1.1, 1.2)
HT_ROWS = [
    (f"3A, {start_text(SEQ_3A.lead)}, calibrated", SEQ_3A, 1.0),
    ("3A-w, the take waits, the same start", variant(SEQ_3A, pi_in="wait"), 1.0),
    ("3A-w, started at the top", variant(SEQ_3A, pi_in="wait", lead=0.0), 1.0),
    ("3A, 15.0 us before the top", variant(SEQ_3A, lead=15.0), 1.0),
    (f"3A-v, {start_text(SEQ_3AV.lead)}, for the bottom", SEQ_3AV, 1.0),
    ("3A-v, 15.0 us before the top", variant(SEQ_3AV, lead=15.0), 1.0),
    (f"3A-r, {start_text(SEQ_3AR.lead)}, built", SEQ_3AR, 1.0),
    (f"3A-r, {start_text(LEAD_3AR)}, calibrated", variant(SEQ_3AR, lead=LEAD_3AR), 1.0),
    ("3A-rw, the take waits, the same start", variant(SEQ_3AR, lead=LEAD_3AR, pi_in="wait"), 1.0),
    (f"3B, {start_text(SEQ_3B.lead)}, calibrated", SEQ_3B, K_3B),
    ("3B-w, the take waits, the same start", variant(SEQ_3B, pi_in="wait"), K_3B),
]
print(f"\nHardware Triggered: sample -> load (us) by k; * the take missed. 3B runs at k x {K_3B}, a stand-in for S32")
print(f"{'':46s}" + " ".join(f"k {k:<5}" for k in HT_KS) + "  write  CPU, I Feedback / next cycle")
for name, s, kf in HT_ROWS:
    cells = []
    for k in HT_KS:
        r = Sim(s, k * kf).run().result()
        cells.append(f"{r['sample_to_load']:6.1f}{' ' if r['hit'] else '*'}")
    r1 = Sim(s, kf).run().result()
    print(f"{name:46s}" + " ".join(cells) + f"  {r1['write']:5.1f}  {r1['busy_analog']:4.1f} / {r1['busy_off']:4.1f}")


def plot_start_calibration(path):
    """The wait the take would need, and sample -> load without it, by the set's start. The code takes the latest start with no wait."""
    starts = [-25.0 + i for i in range(41)]  # from the init trigger at the bottom, delay 0, to 15 us after the top
    cases = [("3A, requests on the off cycle", SEQ_3A, "tab:blue"), ("3A-r, requests every cycle", SEQ_3AR, "tab:brown")]
    k_styles = ((0.8, ":"), (1.0, "-"), (1.2, "--"))
    fig, (ax_w, ax_l) = plt.subplots(2, 1, figsize=(11, 8.5), sharex=True)
    for name, seq, color in cases:
        for k, ls in k_styles:
            results = [at_start(seq, k, -x) for x in starts]
            waits = [r["wait"] for r in results]
            loads = [r["sample_to_load"] for r in results]
            ax_w.plot(starts, waits, color=color, linestyle=ls, label=f"{name}, k {k}")
            ax_l.step(starts, loads, where="mid", color=color, linestyle=ls, label=f"{name}, k {k}")
        cal = SEQ_3A.lead if seq is SEQ_3A else LEAD_3AR
        for ax in (ax_w, ax_l):
            ax.axvline(-cal, color=color, linewidth=1.2, alpha=0.6)
        ax_w.text(-cal + 0.3, 1.0 if seq is SEQ_3A else 2.0, f"calibrated at k 1: {-cal:+.1f} us", color=color, fontsize=8)
    for ax in (ax_w, ax_l):
        ax.axvline(0.0, color="black", linewidth=0.6, linestyle=":")
        ax.axvline(-HW_LEAD, color="gray", linewidth=0.8, linestyle="-.")
        ax.axvspan(-25.0, -25.0 + 10.2, color="#e8f5e9", zorder=0)   # SIM_SOPT0[DELAY], 255 ticks at the 25 MHz bus clock
        ax.axvspan(-25.0 + 10.2, -25.0 + 20.4, color="#fff8e1", zorder=0)  # BUSREF / 2
        ax.axvspan(-25.0 + 20.4, 15.0, color="#fbe9e7", zorder=0)    # BUSREF / 4 and beyond
        ax.grid(alpha=0.4)
    ax_w.text(-24.5, ax_w.get_ylim()[1] * 0.9, "the delay at the bus clock", fontsize=7, color="green")
    ax_w.text(-14.3, ax_w.get_ylim()[1] * 0.9, "BUSREF / 2", fontsize=7, color="#b8860b")
    ax_w.text(-4.1, ax_w.get_ylim()[1] * 0.9, "BUSREF / 4", fontsize=7, color="#bf360c")
    ax_w.text(-HW_LEAD + 0.3, ax_w.get_ylim()[1] * 0.75, f"built, {-HW_LEAD:.1f} us", fontsize=7, color="gray")
    ax_w.set_ylabel("Wait the take would need (us)\nthe -w variations spin it")
    ax_w.set_title("Hardware Triggered: calibrating the start. The latest start with no wait at the take", fontsize=11)
    ax_l.set_ylabel("Sample -> load without the wait (us)\na late set is taken on the next cycle")
    ax_l.set_xlabel("The set's start, from the top whose LOOP takes it (us). The init trigger is at the bottom, -25")
    for load, label in ((25, "the bottom"), (50, "the next top"), (75, "the next bottom"), (100, "the top after")):
        ax_l.axhline(load - SAMPLE_OFFSET, color="#cccccc", linewidth=0.5)
    ax_l.legend(fontsize=7, ncol=2, loc="upper left")
    fig.text(0.01, 0.005, "Durations are estimates, not measured. k is a factor on all of them. The delay's clock is an open point (CurrentLoopSequencing.md).",
             fontsize=7, color="gray")
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)


plot_start_calibration(os.path.join(here, "PwmSampleSequence_3_StartCalibration.png"))

plot_order_compare([("Before 2026-11-01: FOC first. The requests after the write, the PI on requests 1 tick old", ORDERS[2][1]),
                    ("Built now, 3A-r: the requests first, then the take", next(s for s in SEQS if s.key == "3A-r")),
                    ("VBUS_CAPTURE_ON_MAIN, the same trigger delay: the ADC ISR ends earlier", VBUS_MAIN[0][1]),
                    (f"VBUS_CAPTURE_ON_MAIN, the set started {BASE['vsource']:.1f} us later, {HW_LEAD_VBUS_MAIN:.1f} us before the top", VBUS_MAIN[1][1])],
                   os.path.join(here, "PwmSampleSequence_3_Order.png"))

fig, (ax_a, ax_b, ax_c) = plt.subplots(3, 1, figsize=(11, 12.5), sharex=True)
styles = {"1a": ("tab:purple", "-"), "1b": ("tab:purple", ":"), "1c": ("tab:green", "-"), "2a": ("tab:orange", "-"), "2b": ("tab:red", "--"), "3A": ("tab:blue", "-"), "3A-r": ("tab:brown", "-")}
for s in SEQS:
    color, ls = styles[s.key]
    label = f"{s.key}. {s.title}"
    ax_a.step(ks, [r["sample_to_load"] for r in sweep[s.key]], where="mid", color=color, linestyle=ls, label=label)
    ax_b.plot(ks, [r["busy_analog"] for r in sweep[s.key]], color=color, linestyle=ls)
    ax_c.plot(ks, [r["park_offset"] for r in sweep[s.key]], color=color, linestyle=ls)
ax_a.axvline(1.0, color="gray", linewidth=0.8)
for load, name in ((25, "same period's bottom"), (50, "next top"), (75, "next bottom"), (100, "top after")):
    ax_a.text(1.605, load - SAMPLE_OFFSET, f"loads at the {name}", fontsize=7, va="center", color="gray")
ax_a.text(1.605, 25 + HW_LEAD - SAMPLE_OFFSET + 0.5, "3A-r: its sample is ahead of the top", fontsize=7, va="bottom", color="tab:brown")
ax_a.set_ylabel("Sample -> load (us)")
ax_a.set_title("Current-loop sequences on KE, by a factor on every ISR duration (1 = the estimate)")
ax_a.grid()
fig.legend(*ax_a.get_legend_handles_labels(), loc="lower center", ncol=1, fontsize=8, bbox_to_anchor=(0.5, 0.0))
ax_b.axhline(T, color="red", linewidth=0.8)
ax_b.axvline(1.0, color="gray", linewidth=0.8)
ax_b.text(0.41, T + 0.5, "the period: past it the next PWM ISR starts late", fontsize=7, color="red")
ax_b.set_ylabel("CPU in batch 0's cycle (us)")
ax_b.grid()
ax_c.axvline(1.0, color="gray", linewidth=0.8)
ax_c.set_ylabel("Park angle time - sample time (us)")
ax_c.set_xlabel("Speed factor k on the estimated durations (lower is faster)")
ax_c.text(0.41, -48, "-50: the previous tick's angle, Parked before this tick's SetTheta", fontsize=7)
ax_c.grid()
fig.tight_layout(rect=(0, 0.13, 0.86, 1))
fig.savefig(os.path.join(here, "PwmSampleSequenceSummary.png"), dpi=100)
plt.show()
