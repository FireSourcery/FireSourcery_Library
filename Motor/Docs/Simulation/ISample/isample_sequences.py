"""
The sequences the current loop was chosen from, by the ids the docs use. Until 2026-11-04 the record numbered them; the old ids
in brackets, mapped in CurrentLoopSequencing.exp.md (Kelly_KMC/Docs), Derivation:

    Software start          the PWM ISR starts the set at its top; it lands during the LOOP
        1a                  the take finds the set: the ADC ISR, above, preempts before it
        1b                  the take misses: the next cycle's LOOP takes the set
        1c                  the take waits for the capture
    PI at the completion    the ADC ISR runs the PI through ON_PHASE [MOTOR_I_FEEDBACK_ON_PHASE]
        2a                  at the PWM ISR's priority
        2b                  above it
    Triggered               the trigger lands the set ahead of the PWM ISR whose LOOP takes it. KE at the 20 kHz PWM: the control
                            cycle on every 2nd PWM ISR. The PWM ISR between, the off cycle, either marks and arms the next set and
                            runs no LOOP, or runs the LOOP and writes the held V at its own angle: +V
        before              the set 16.5 us before the control cycle's top, the between ISR marks and arms. The built form since
                            2026-11-03: loads at the next top. The taking cycle of before+V, without the off cycle's LOOP
        after               the set after the top, as late as the take finds it: fresher; faster code misses. Not built
        before+V            the off cycle's LOOP writes the held V at its angle. Built 2026-10-25 to 11-02 [3A-r at 16.5 us]
        after+V             the same, the set after the top [3A-r at its latest start]. after+V, wait: the take waits [3A-rw]
        bottom              the requests on the off cycle, so the taking cycle writes before the bottom; the off cycle writes the
                            held V. Built 2026-11-02 to 11-04 [3A]. bottom, VBus off: VBus's divide in the off cycle [3A-v]
        every               one set per PWM period: S32's form, at half KE's durations [3B]. every, wait [3B-w]

Each triggered variant runs at the start its load point allows [latest_lead]: the latest start whose taking cycle's write makes
that load point. The bottom: nothing before the take but the angle (bottom); the ADC ISR's end limits the start. The next top: the
requests before the take (before, after); the take limits the start. An earlier start buys margin on slower code at the bottom, on
faster code at the take. The +V forms are modeled without the off cycle's arm, as the builds they record. Before 2026-11-02 the
ids were 3a, 3b, 3c for 3A, 3A-r, 3B; before 11-01, sequences 1-6.
"""
from isample_model import (BASE, HW_LEAD, LOAD, PRIO_HIGH, PRIO_PWM, T, Seq, Sim, delay_clock, k_range, latest_lead, start_text, variant)

LEAD_BUILT = 16.5                    # BOARD_ADC_TRIGGER_DELAY: 213 ticks of the 25 MHz bus clock past the bottom, 8.5 us after it
LEAD_BOTTOM_KE = 15.0                # the bottom form as built: 250 ticks
LEAD_REACH = 25.0 - 255 / 25.0       # the latest the 8-bit delay reaches at the bus clock, 14.8 us before the top

NAMES = {
    "1a": "software start, the take finds the set", "1b": "software start, the take misses", "1c": "software start, the take waits",
    "2a": "PI at the completion, at the PWM ISR's priority", "2b": "PI at the completion, above the PWM ISR",
    "before": "triggered, the set before the top, the between ISR marks and arms", "after": "triggered, the set after the top, the between ISR marks and arms",
    "before+V": "triggered, the set before the top, the off cycle's LOOP writes the held V", "after+V": "triggered, the set after the top, the off cycle's LOOP writes the held V",
    "after+V, wait": "after+V, the take waits", "bottom": "triggered, the requests on the off cycle: loads at the bottom",
    "bottom, VBus off": "bottom, VBus's divide in the off cycle", "every": "triggered, one set per PWM period", "every, wait": "every, the take waits",
}

SEQS = [
    Seq("1a", "Software start, the take finds the set: the ADC ISR preempts before the take", "1a_TakeHit", PRIO_HIGH, True, "loop", "loop", "default", "sw",
        ["The ADC ISR only captures. The LOOP's take Parks at this tick's angle and runs the PI",
         "if the set landed before the take, else on the next tick: 1b. One duty writer."]),
    Seq("1b", "Software start, the take misses: the ADC ISR at the PWM ISR's priority", "1b_TakeMiss", PRIO_PWM, True, "loop", "loop", "default", "sw",
        ["1a's miss: the capture comes after this tick's take. At the PWM ISR's priority the ADC ISR",
         "runs after the PWM ISR returns, so it always misses. The next tick's LOOP takes the set,",
         "Parked at its own angle. One duty writer per tick, the LOOP."]),
    Seq("1c", "Software start, the LOOP waits at the take: the ADC ISR preempts and captures", "1c_TakeWait", PRIO_HIGH, True, "loop", "wait", "default", "sw",
        ["The LOOP spins at its take until the ADC ISR, above it, has captured the set. It never misses.",
         "Spins only if the LOOP reaches the take before the set lands. One duty writer."]),
    Seq("2a", "PI at the completion, at the PWM ISR's priority (MOTOR_I_FEEDBACK_ON_PHASE)", "2a_AdcCallback", PRIO_PWM, True, "adc", "adc", "default", "sw",
        ["The ADC ISR runs after the PWM ISR returns, so the PI waits for the whole PWM ISR.",
         "The PWM ISR and the PI land in the same period: the sampling cycle's load.",
         "The 1 ms SysTick thread (priority 2, exception 15) also runs first when it is pending.",
         "The LOOP skips its out on the sampling cycle, a duplicate of the ADC ISR's: not supported in the code."]),
    Seq("2b", "PI at the completion, above the PWM ISR", "2b_Preempt", PRIO_HIGH, True, "adc", "adc", "default", "sw",
        ["The ADC ISR preempts the PWM ISR wherever it is. Hazards:",
         "- the Park takes the held angle: the previous tick's if the LOOP has not reached SetTheta,",
         "- a state transition in progress in the PWM ISR: ON_PHASE dispatches to a half-entered state.",
         "The LOOP skips its out on the sampling cycle, so it can't overwrite the ADC ISR's duty: not supported in the code."]),
    # Triggered, loads at the next top. The built form, and its start moved after the top. Notes below
    Seq("before", "Triggered, the set before the top, the PWM ISR between control cycles marks and arms it: the built form at the 20 kHz PWM. Loads at the next top",
        "LoadTop", PRIO_HIGH, True, "loop", "loop", "default", "hw", [], between="arm", lead=LEAD_BUILT),
    Seq("after", "Triggered, the set after the top, as late as the take finds it; the between ISR marks and arms. Loads at the next top", None,
        PRIO_HIGH, True, "loop", "loop", "default", "hw", [], between="arm"),
    # The same, with the off cycle's LOOP writing the held V at its angle: the forms built 2026-10-25 to 11-02
    Seq("before+V", "Triggered, the set before the top, the off cycle's LOOP writes the held V at its angle: loads at the next top. Built 2026-10-25 to 11-02",
        "LoadTopSplit", PRIO_HIGH, True, "loop", "loop", "default", "hw", [], lead=LEAD_BUILT),
    Seq("after+V", "Triggered, the set after the top, the latest the take finds; the off cycle's LOOP writes the held V. Loads at the next top", None,
        PRIO_HIGH, True, "loop", "loop", "default", "hw", []),
    # Loads at the bottom: the requests on the off cycle, started 15 us before the top, the sets alternating, as built on KE06 2026-11-02 to 11-04
    Seq("bottom", "Triggered, the requests on the off cycle, the taking cycle writes before the bottom: started 15.0 us before the top, the sets alternating. Built on KE06 2026-11-02 to 11-04",
        "LoadBottom", PRIO_HIGH, True, "loop", "loop", "default", "hw", [], requests="off", lead=LEAD_BOTTOM_KE, sets=True, others=2),
]


def by(key):
    return next(s for s in SEQS if s.key == key)


SEQ_BEFORE, SEQ_AFTER, SEQ_BEFORE_V, SEQ_AFTER_V, SEQ_BOTTOM = (by(k) for k in ("before", "after", "before+V", "after+V", "bottom"))

# Loads at the next top: the take sets the latest start
SEQ_AFTER.lead = latest_lead(SEQ_AFTER, 1.0, T)            # the latest the take finds at k 1: after the top
SEQ_AFTER_V.lead = latest_lead(SEQ_AFTER_V, 1.0, T)        # the same taking cycle, the same start
LEAD_AFTER_FAST = latest_lead(SEQ_AFTER, 0.8, T)           # the take finds the set on a cycle 20 % faster
SEQ_AFTER_VW = variant(SEQ_AFTER_V, key="after+V, wait", pi_in="wait")
LEAD_AFTER_VW = latest_lead(SEQ_AFTER_VW, 1.0, T)
R_BEFORE = Sim(SEQ_BEFORE, 1.0).run().result()
R_AFTER = Sim(SEQ_AFTER, 1.0).run().result()
R_BEFORE_V = Sim(SEQ_BEFORE_V, 1.0).run().result()
R_AFTER_MISS = Sim(SEQ_AFTER, 0.8).run().result()          # the miss: the next control cycle takes the set
R_AFTER_V_MISS = Sim(SEQ_AFTER_V, 0.8).run().result()      # the miss with the off cycle's LOOP: it takes the set

SEQ_BEFORE.notes = [f"FTM2 starts the ADC {start_text(LEAD_BUILT)}: the init trigger at the bottom, delayed 213 ticks of the 25 MHz bus clock (BOARD_ADC_TRIGGER_DELAY).",
                    f"The PWM ISR between control cycles marks the set and arms it, {R_BEFORE['arm_set']:.1f} us ahead of its trigger, and runs no LOOP: {R_BEFORE['busy_off']:.1f} us.",
                    f"The set and its ADC ISR are done before the top. The LOOP takes it after the requests, at its own angle, and writes {R_BEFORE['write']:.1f} us in:",
                    f"past the bottom, so it loads at the next top. Holds to k {k_range(SEQ_BEFORE)[1]:.2f}. Measured 2026-11-03: the write at 30.6 us, 62.0 us; 66.0 by the ADC ISR's entry.",
                    "S32's form, where PDB1 and the DMA do the setup. Main-loop channels ride behind the set in the one FIFO, 2 us each on the completion.",
                    "A set that lands after the take waits for the next control cycle: the between ISR takes nothing. The taking cycle of before+V."]
SEQ_BEFORE_V.notes = [f"The taking cycle of 'before'. The off cycle's LOOP runs the angle and the requests and writes the held Vd, Vq at its own angle: {R_BEFORE_V['busy_off']:.1f} us.",
                      "Each write is applied for one PWM period instead of two: the angle sawtooth over the hold halves, +-5.4 to +-2.7 deg at 300 Hz electrical.",
                      "The PI and |V| still run at 10 kHz, so the current loop's delay is unchanged. The mean angle lag, absorbed by the PI's integrators, drops 25 us.",
                      "On a miss the off cycle's LOOP takes the set: 50 us late instead of 100. Its arm is not drawn.",
                      "Built 2026-10-25 to 11-02 with the I-loop divider (MOTOR_I_LOOP_FREQ), removed 11-04. Not adopted: the sawtooth is the whole benefit."]

# Loads at the bottom: the ADC ISR's end sets the latest start
LEAD_BOTTOM_LATEST = latest_lead(SEQ_BOTTOM, 1.0, LOAD)    # the latest whose write makes the bottom, 265 ticks: past the bus clock's 255
LEAD_BOTTOM_CLEAR = LEAD_BUILT                             # the ADC ISR ends before the top
SEQ_BOTTOM_WALK = variant(SEQ_BOTTOM, sets=False, others=0)  # until 2026-11-03: the set alone on the trigger, its completion walks the others
R_BOTTOM = Sim(SEQ_BOTTOM, 1.0).run().result()
R_BOTTOM_WALK = Sim(variant(SEQ_BOTTOM_WALK, walk=2), 1.0).run().result()
SEQ_BOTTOM.notes = [f"FTM2 starts the ADC {start_text(LEAD_BOTTOM_KE)}: the init trigger at the bottom, delayed 250 ticks of the 25 MHz bus clock.",
                    f"The latest start whose write makes the bottom, {start_text(LEAD_BOTTOM_LATEST)}, is past the 8-bit delay's reach, {start_text(LEAD_REACH)}.",
                    f"The ADC ISR, VSOURCE's divide included, runs {R_BOTTOM['pwm_delay']:.1f} us past the top. The taking cycle runs no requests,",
                    f"so its write lands {R_BOTTOM['write']:.1f} us after the top, {LOAD - R_BOTTOM['write']:.1f} us before the bottom. The off cycle steps the requests at 10 kHz and writes the held V.",
                    "Each write is applied for a full PWM period: the taking cycle's from the bottom, the off cycle's from the next. The write must land within half a period,",
                    "25 us after the top: on the board (2026-11-03) it came at about 29 us, k about 1.2, and loaded at the next top, 64.0 us. Not met on KE06 for now.",
                    f"Each PWM ISR arms the next trigger's set after its write: the off cycle the I set, {R_BOTTOM['arm_set']:.1f} us before its trigger;",
                    f"the taking cycle the main-loop channels, here the throttle and brake, {R_BOTTOM['arm_others']:.1f} us before theirs. Nothing walks.",
                    f"Slower than k {k_range(SEQ_BOTTOM)[1]:.2f} it loads at the next top: {Sim(SEQ_BOTTOM, 1.1).run().result()['sample_to_load']:.1f} us. Dropped 2026-11-04 for 'before'."]

# VBus's divide in the off cycle. The ADC ISR stores VBus_Pu alone; the off cycle's PWM ISR divides for VBusInv_Pu, with the requests.
# The ADC ISR shortens by the divide: the latest start for the bottom moves later
SEQ_BOTTOM_V = variant(SEQ_BOTTOM, key="bottom, VBus off", title="Triggered, the requests on the off cycle, VBus's divide there too: loads at the bottom",
                       file="VBusOffCycle", vbus="off")
SEQ_BOTTOM_V.lead = latest_lead(SEQ_BOTTOM_V, 1.0, LOAD)
SEQ_BOTTOM_V.notes = [f"FTM2 starts the ADC {start_text(SEQ_BOTTOM_V.lead)}: the latest start whose taking cycle writes before the bottom, at k = 1."
                      f" The delay at {delay_clock(SEQ_BOTTOM_V.lead)}.",
                      f"The ADC ISR stores VBus_Pu alone, {BASE['vbus_store']:.1f} us, and the off cycle divides for VBusInv_Pu, {BASE['vbus_divide']:.1f} us, with the requests.",
                      f"The ADC ISR is {BASE['vsource'] - BASE['vbus_store']:.1f} us shorter. With the divide in it, the latest start for the bottom is {start_text(LEAD_BOTTOM_LATEST)}.",
                      f"The taking cycle runs no requests: its PWM ISR writes {sum(BASE[n] for n in ('entry', 'sensor', 'sm', 'theta', 'park', 'pi', 'out')):.1f} us in,"
                      " and the ADC ISR can overrun the top by the rest of the 25. Its out reads the previous set's VBusInv_Pu, 100 us older.",
                      f"A slower cycle, from k {k_range(SEQ_BOTTOM_V)[1] + 0.01:.2f}, misses the bottom: the cliff to the next top. A faster one finds the set sooner. Never built."]

# One set per PWM period. S32's form; its durations are unmeasured, so it runs at half KE's (k 0.5), on KE's ADC timing. Loads at the bottom.
# KE runs this placement at 100 us: 'before', the control cycle on every 2nd PWM ISR. Measured, not simulated: ke_isr_timeline.py (Kelly_KMC/Docs/Simulations)
K_EVERY = 0.5
SEQ_EVERY = Seq("every", "Triggered, one set per PWM period: loads at the bottom. S32's form at half KE's durations", "EveryCycle", PRIO_HIGH, True, "loop", "loop", "default", "hw", [], divider=1)
SEQ_EVERY.lead = latest_lead(SEQ_EVERY, K_EVERY, LOAD)
SEQ_EVERY_W = variant(SEQ_EVERY, key="every, wait", pi_in="wait")
LEAD_EVERY_W = latest_lead(SEQ_EVERY_W, K_EVERY, LOAD)
SEQ_EVERY.notes = [f"At half KE's durations (k = {K_EVERY}), a stand-in for S32's, unmeasured, on KE's ADC timing.",
                   f"FTM2 starts the ADC {start_text(SEQ_EVERY.lead)}: the latest start whose set the take finds, at that k. The write makes the bottom with time to spare,",
                   "so the take, not the write, sets the start. Every PWM cycle takes a set, and arms the next.",
                   f"No busy wait. 'every, wait' waits, and its latest start for the bottom is {start_text(LEAD_EVERY_W)}.",
                   "KE06 runs this placement at 100 us, the control cycle on every 2nd PWM ISR ('before'): measured on the board, ke_isr_timeline.py."]

# The LOOP's order: the requests before the take (default), or after the write (FOC first, the PI on requests 1 tick old; removed 2026-11-01).
# The off cycle plays no part, so 'before' and 'before+V' give the same rows
ORDERS = [
    ("1a, FOC first", Seq("1a", "", None, PRIO_HIGH, True, "loop", "loop", "foc_first", "sw", [])),
    ("1c, FOC first", Seq("1c", "", None, PRIO_HIGH, True, "loop", "wait", "foc_first", "sw", [])),
    ("before, FOC first", Seq("before", "", None, PRIO_HIGH, True, "loop", "loop", "foc_first", "hw", [], between="arm")),
]
# VBus's divide in the off cycle: the ADC ISR stores VBus_Pu alone, so the set can start that much later
HW_LEAD_VBUS_OFF = HW_LEAD - (BASE["vsource"] - BASE["vbus_store"])
VBUS_OFF = [
    ("before+V, VBus off", Seq("before+V", "", None, PRIO_HIGH, True, "loop", "loop", "default", "hw", [], vbus="off")),
    ("before+V, VBus off, later", Seq("before+V", "", None, PRIO_HIGH, True, "loop", "loop", "default", "hw", [], vbus="off", lead=HW_LEAD_VBUS_OFF)),
]
ORDER_DEFAULTS = [(f"{s.key}, default", s) for s in SEQS if s.key in ("1a", "1c")] + [(f"bottom, {LEAD_BOTTOM_KE:.1f} us, built", SEQ_BOTTOM),
                                                                                     (f"before, {LEAD_BUILT:.1f} us, built", SEQ_BEFORE)]

# Triggered by load point: (group, label, seq, k factor)
HT_ROWS = [
    ("the bottom", f"bottom, {start_text(LEAD_BOTTOM_LATEST)}, the latest", variant(SEQ_BOTTOM, lead=LEAD_BOTTOM_LATEST), 1.0),
    ("the bottom", f"bottom, {start_text(LEAD_BOTTOM_KE)}, built", SEQ_BOTTOM, 1.0),
    ("the bottom", "bottom, the same, all 7 main-loop channels on the off cycle", variant(SEQ_BOTTOM, others=7), 1.0),
    ("the bottom", "bottom, until 11-03, the throttle and brake walked", variant(SEQ_BOTTOM_WALK, walk=2), 1.0),
    ("the bottom", f"bottom, {start_text(LEAD_BOTTOM_CLEAR)}, the ISR before the top", variant(SEQ_BOTTOM, lead=LEAD_BOTTOM_CLEAR), 1.0),
    ("the bottom", f"bottom, VBus off, {start_text(SEQ_BOTTOM_V.lead)}, the latest", SEQ_BOTTOM_V, 1.0),
    ("the bottom", f"bottom, VBus off, {start_text(LEAD_BOTTOM_KE)}", variant(SEQ_BOTTOM_V, lead=LEAD_BOTTOM_KE), 1.0),
    ("the bottom", f"every, {start_text(SEQ_EVERY.lead)}, the latest", SEQ_EVERY, K_EVERY),
    ("the bottom", "every, wait, the same start", variant(SEQ_EVERY_W, lead=SEQ_EVERY.lead), K_EVERY),
    ("the bottom", f"every, wait, {start_text(LEAD_EVERY_W)}, the latest", variant(SEQ_EVERY_W, lead=LEAD_EVERY_W), K_EVERY),
    ("the next top", f"before, {start_text(LEAD_BUILT)}, built", SEQ_BEFORE, 1.0),
    ("the next top", f"after, {start_text(SEQ_AFTER.lead)}, the latest", SEQ_AFTER, 1.0),
    ("the next top", f"after, {start_text(LEAD_AFTER_FAST)}, the latest at k 0.8", variant(SEQ_AFTER, lead=LEAD_AFTER_FAST), 1.0),
    ("the next top", f"before+V, {start_text(LEAD_BUILT)}, built until 11-02", SEQ_BEFORE_V, 1.0),
    ("the next top", f"before+V, {start_text(LEAD_REACH)}, the bus clock's reach", variant(SEQ_BEFORE_V, lead=LEAD_REACH), 1.0),
    ("the next top", f"after+V, {start_text(SEQ_AFTER_V.lead)}, the latest", SEQ_AFTER_V, 1.0),
    ("the next top", "after+V, wait, the same start", SEQ_AFTER_VW, 1.0),
    ("the next top", f"after+V, {start_text(LEAD_AFTER_FAST)}, the latest at k 0.8", variant(SEQ_AFTER_V, lead=LEAD_AFTER_FAST), 1.0),
    ("the next top", f"after+V, wait, {start_text(LEAD_AFTER_VW)}, the latest", variant(SEQ_AFTER_VW, lead=LEAD_AFTER_VW), 1.0),
]
