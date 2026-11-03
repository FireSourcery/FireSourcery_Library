"""
The model: KE06's PWM period, ADC timing and ISR durations, a sequence's description [Seq], and the two-level preemptive
scheduler that runs one [Sim]. The sequences themselves are in isample_sequences.py, the figures in isample_plots.py, and
pwm_sample_sequences.py runs them. The vocabulary: CurrentLoopSequencing.exp.md (Kelly_KMC/Docs), Derivation.

Model
    FTM2 center-aligned, T = 50 us. Counter top (TOF, PWM ISR) at t = 0, 50, ...; bottom at 25 + 50n.
    Both loading points are enabled: a duty written loads at the next top or bottom after the write.
    The control cycle is every 2nd PWM period (divider 2: KE at the 20 kHz PWM) or every period (divider 1): the cycles at t = 0, 100, ...
    are scheduled to take a set. The PWM ISR between, the off cycle, either marks and arms the next set and runs no LOOP [between 'arm',
    the built form], or runs the LOOP and writes the held V at its own angle [between 'loop', the alternating forms].
    Batch b is the set the cycle at t = b is scheduled to take: started in it in software, or ahead of it by the trigger.
    ADC: 10 MHz ADCK, VSOURCE then IA, IB, IC; IB samples 4.5 us after the start, the batch completes at 8.5 us.
    NVIC: lower priority value preempts; equal priority does not, and a tie between pending IRQs goes to the lower number (ADC 15, FTM2 19).
    The sample's place in the period does not change its age through the sense filter (CurrentSenseFilter.md), so the comparison
    is sample -> load. Delay to actuation = age (~96 us at hold tau 10 us) + sample -> load + 50 us (half the 2-period hold).

Durations are estimates at k = 1: instruction counts of the linked KE06 image at ~32 ns each (50 MHz, CPI ~1.6). Measured on the
board 2026-11-03 at about 1.2 times the estimate for the code, 1.4 for the ADC ISR (CurrentLoopSequencing.exp.md, Time per call).
k is a factor on every duration, because which load point each sequence makes depends on it.
"""
import copy
import heapq
import itertools
import math

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
    vbus_store=0.5,  # VBus_Pu alone, its divide in the off cycle: -v
    vbus_divide=3.0, # VBusInv_Pu, the divide, in the off cycle's PWM ISR: -v
    icapture=2.0,    # the I set from the results buffer
    dispatch=0.3,    # ON_PHASE action dispatch
    adc_exit=0.5,
    fifo_read=0.3,   # 1 more fifo result in the ADC ISR's capture, past the set's 4
    arm=1.6,         # ADC_ArmMarked: the fifo from the marks, the trigger on. As the software start's estimate
    arm_idle=0.3,    # ADC_ArmMarked, nothing marked
)
CONV = (("VSOURCE", 0.0, 2.5), ("IA", 2.5, 2.0), ("IB", 4.5, 2.0), ("IC", 6.5, 2.0))
CONV_DONE = 8.5
SAMPLE_OFFSET = 4.5  # IB, the middle of the set
AGE_US, HOLD_HALF_US = 96.0, 50.0  # CurrentSenseFilter.md, hold tau 10 us, 300 Hz; the 10 kHz loop's half hold
HW_LEAD = CONV_DONE + BASE["adc_entry"] + BASE["vsource"] + BASE["icapture"] + BASE["adc_exit"] + 0.5  # triggered: done before the top


def walk_conv(n):
    """A FIFO of n channels, by software or the trigger: the first conversion 2.5 us, each later one 2 us"""
    return 2.5 + 2.0 * (n - 1)


class Seq:
    """
    One sequence. [key] its id (1a, 2b, before, after+V, bottom, ...), [title] its description, [file] the timeline's file suffix or None.
        adc_prio    the ADC ISR's NVIC priority: PRIO_HIGH above the PWM ISR, PRIO_PWM equal to it
        adc_isr     the ADC ISR handles the I set
        park_in     'adc' / 'loop': where the set is Parked
        pi_in       'loop' / 'adc' / 'wait': where the PI runs; 'wait' spins the LOOP's take until the capture
        order       'default' the requests before the take; 'foc_first' after the write (removed 2026-11-01)
        trigger     'sw' the PWM ISR starts the set at its top; 'hw' the trigger starts it [lead] us before the top (negative: after it)
        vbus        'adc': VBus captured and divided in the ADC ISR; 'off': VBus_Pu stored there, divided in the off cycle
        requests    'every': the ramp and field weakening every cycle; 'off': on the off cycle only (loads at the bottom)
        divider     PWM periods per control cycle: 2 KE at the 20 kHz PWM, 1 every period
        between     the PWM ISR between control cycles: 'loop' runs the LOOP and writes the held V at its angle, then arms if [sets];
                    'arm' marks and arms the next set, no LOOP (the built form)
        walk        main-loop channels marked with batch 0's set: its completion walks them by software (until 2026-11-03)
        sets        alternating sets: each PWM ISR arms the next trigger's set after its write, nothing walks
        others      alternating sets: main-loop channels on the trigger after batch 0's, the off cycle's set
    """

    def __init__(self, key, title, file, adc_prio, adc_isr, park_in, pi_in, order, trigger, notes, vbus="adc", lead=None, requests="every", divider=2,
                 between="loop", walk=0, sets=False, others=0):
        self.key, self.title, self.file = key, title, file
        self.adc_prio, self.adc_isr = adc_prio, adc_isr
        self.park_in, self.pi_in = park_in, pi_in
        self.order, self.trigger = order, trigger
        self.notes = notes
        self.vbus = vbus
        self.lead = HW_LEAD if lead is None else lead
        self.requests = requests
        self.divider = divider
        self.between = between
        self.walk = walk
        self.sets = sets
        self.others = others


class Sim:
    def __init__(self, seq, k, t0=-200.0, t1=200.0):
        self.seq, self.k = seq, k
        self.d = {name: value * k for name, value in BASE.items()}
        self.conv, self.conv_done, self.sample_offset = CONV, CONV_DONE, SAMPLE_OFFSET
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
        self.walks = []                          # (start, channels, name): the software walk after a set's completion, or the off cycle's set on the trigger
        self.arm_end = {}                        # tick -> the time its PWM ISR armed the next trigger's set [sets]
        for n in range(int(math.floor(t0 / T)), int(math.ceil(t1 / T)) + 1):
            tick = n * T
            analog = (n % seq.divider == 0)
            self.at(tick, lambda tick=tick, analog=analog: self.release("PWM ISR", PRIO_PWM, IRQ_FTM2, self.pwm_isr(tick, analog), tick))
            if analog and seq.trigger == "hw":
                self.at(tick - seq.lead, lambda tick=tick: self.start_adc(tick - self.seq.lead, tick))
            if seq.sets and seq.others and tick == T:
                self.at(tick - seq.lead, lambda: self.start_others(self.seq.others))

    # Events and jobs
    def at(self, time, fn):
        heapq.heappush(self.events, (time, next(self.order), fn))

    def release(self, lane, prio, irq, gen, tag):
        self.jobs.append(dict(lane=lane, prio=prio, irq=irq, gen=gen, seg=None, release=self.t, tag=tag, started=None))

    def start_adc(self, time, tick):
        self.batches[tick] = dict(start=time, done=time + self.conv_done, walk=(self.seq.walk if tick == 0.0 else 0))
        if self.seq.adc_isr:
            self.at(time + self.conv_done, lambda tick=tick: self.release("ADC ISR", self.seq.adc_prio, IRQ_ADC, self.adc_isr(tick), tick))

    def start_walk(self, tick):
        """ADC_OnComplete_ISR continues the walk after reading the set's FIFO: the marked main-loop channels, one software-started FIFO"""
        n = self.batches[tick].get("walk", 0)
        if n:
            self.batches[tick]["walk"] = 0
            self.walks.append((self.t, n, "walk"))
            self.at(self.t + walk_conv(n), lambda tick=tick: self.release("ADC ISR", self.seq.adc_prio, IRQ_ADC, self.walk_isr(), tick))

    def start_others(self, n):
        """The alternating sets: the main-loop channels armed by batch 0's taking cycle, on the next trigger. Captured, nothing continues"""
        self.walks.append((self.t, n, "others"))
        self.at(self.t + walk_conv(n), lambda: self.release("ADC ISR", self.seq.adc_prio, IRQ_ADC, self.walk_isr(n), T))

    def walk_isr(self, n=0):
        yield self.seg("others", "walk", self.d["adc_entry"] + max(0, n - 4) * self.d["fifo_read"] + self.d["adc_exit"])

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
        if not analog and s.between == "arm":
            # The PWM ISR between control cycles, the built form: the marks and the arm, no LOOP
            yield self.seg("marks, arm", "arm", d["adc_start"] + d["arm"], lambda: self.arm_end.__setitem__(tick, self.t))
            yield self.seg("exit", "shell", d["exit"])
            return
        if analog and s.trigger == "sw":
            yield self.seg("ADC start", "shell", d["adc_start"], lambda: self.start_adc(self.t, tick))
        yield self.seg("sensor", "theta", d["sensor"], lambda: setattr(self, "sensor_tick", tick))
        yield self.seg("state machine", "shell", d["sm"])
        if s.vbus == "off" and not analog:
            yield self.seg("VBus divide", "req", d["vbus_divide"])
        requests = (s.requests == "every") or not analog
        out = not (s.pi_in == "adc" and analog)  # PI at the completion: the ADC ISR writes the sampling cycle's duty, so the LOOP's would be a duplicate. Not in the code
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
        if s.sets:
            # The alternating sets: the off cycle arms the I set, the taking cycle the main-loop channels, after the write
            marked = (not analog) or (tick == 0.0 and s.others > 0)
            yield self.seg("arm", "arm", d["arm"] if marked else d["arm_idle"], lambda: self.arm_end.__setitem__(tick, self.t))
        yield self.seg("exit", "shell", d["exit"])

    def adc_isr(self, tick):
        s, d = self.seq, self.d
        yield self.seg("entry", "shell", d["adc_entry"], lambda: self.start_walk(tick))
        yield self.seg("VSOURCE", "capture", d["vsource"] if s.vbus == "adc" else d["vbus_store"])
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
        sample = b.get("start", math.nan) + self.sample_offset
        write = next((w for w, wb in self.writes if wb == tick and w >= b.get("pi_end", math.inf) - 1e-9), math.nan)
        load = (math.floor(write / LOAD + 1e-9) + 1) * LOAD if not math.isnan(write) else math.nan
        busy = [sum(max(0.0, min(r[4], lo + T) - max(r[3], lo)) for r in self.trace) for lo in (tick, tick + T)]
        park_tick = b.get("park_tick")
        wait = max(0.0, b.get("captured", math.inf) - self.take_at.get(tick, math.inf)) if tick in self.take_at else math.nan
        # Alternating sets: each arm before the trigger it arms for. The off cycle's, before the set's; batch 0's, before the others'
        arm_set = (tick - self.seq.lead) - self.arm_end.get(tick - T, math.nan)
        arm_others = (tick + T - self.seq.lead) - self.arm_end.get(tick, math.nan)
        return dict(wait=wait, hit=(park_tick == tick), sample=sample, park_at=b.get("park_at", math.nan), park_tick=park_tick, park_lane=b.get("park_lane"),
                    park_offset=(park_tick - sample) if park_tick is not None else math.nan,
                    pi_end=b.get("pi_end", math.nan), write=write, load=load, sample_to_load=load - sample,
                    busy_analog=busy[0], busy_off=busy[1], arm_set=arm_set, arm_others=arm_others,
                    pwm_delay=max(self.pwm_delay.get(tick + n * T, 0.0) for n in (0, 1, 2)))


def variant(seq, **changes):
    v = copy.copy(seq)
    for name, value in changes.items():
        setattr(v, name, value)
    return v


def at_start(seq, k, lead):
    """Tick 0's result with the set started [lead] before the top. Its wait: how long the take would have to wait for the capture. Theory only."""
    return Sim(variant(seq, lead=lead), k, t0=-100.0, t1=150.0).run().result()


def latest_lead(seq, k, load):
    """The latest start, as the lead before the top (negative: after it), whose taking cycle takes its set and writes in time for
    [load] at k: the bottom, 25 us, or the next top, 50. Scanned from the bottom, 25 us ahead, to 25 us after the top: 1 us, then 0.1 us."""
    def makes(lead):
        r = at_start(seq, k, lead)
        return r["hit"] and r["load"] <= load + 1e-9
    last = None
    for i in range(51):
        lead = 25.0 - i
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


def k_range(seq, kf=1.0):
    """The k, by 0.01 within 0.5 to 1.5, over which [seq] keeps its load point at k 1 and its take finds the set. [kf] scales k: S32's 0.5"""
    def keeps(k, load):
        r = Sim(seq, k * kf, t0=-100.0, t1=150.0).run().result()
        return r["hit"] and abs(r["load"] - load) < 1e-6
    load = Sim(seq, kf, t0=-100.0, t1=150.0).run().result()["load"]
    lo = hi = 1.0
    while lo > 0.5 and keeps(round(lo - 0.01, 2), load):
        lo = round(lo - 0.01, 2)
    while hi < 1.5 and keeps(round(hi + 0.01, 2), load):
        hi = round(hi + 0.01, 2)
    return lo, hi


def delay_clock(lead):
    """SIM_SOPT0[BUSREF]: the least divider of the 25 MHz bus clock whose 8-bit DELAY, counted from the bottom, reaches the start"""
    for div in (1, 2, 4, 8, 16):
        if (25.0 - lead) * 25.0 / div <= 255.0 + 1e-9:
            return "bus" if div == 1 else f"bus / {div}"
    return "past bus / 16"


def start_text(lead):
    return f"{abs(lead):.1f} us {'before' if lead >= 0 else 'after'} the top"
