# Wrapper Surface

**Level** — design.

**Force** — An outer module owns an inner one, or couples two peers, and its callers need some of
the inner operations. What varies is how much of the inner API the callers need, and whether the
inner type is a decision to keep changeable. What stays still is the outer's invariants over the
inner state. Each exposed operation takes one of three surfaces, picked by a test below; mixing
them without one is what scatters an API. `CLAUDE.md` holds the same rules under API Design.

**Shape**

| Surface | Also called | Form | Inner type |
|---|---|---|---|
| Forwarder | thin forwarding, "delegation", "pass-through" | `Outer_Op(p) { return Inner_Op(&p->Inner); }` | hidden |
| Getter | "exposed composition" | `Inner_T * Outer_Inner(p_outer)`; callers use the inner's own API | exposed |
| Adaptation | re-composed, transformed | `Outer_Op(p_outer)` composes inner operations with state only the outer holds | hidden |

- **Forwarder — the wrapper is the public facade.** Callers need to know only the outer type: the
  Law of Demeter, "don't talk to strangers". Callers of `VBus_T` need not know `RangeMonitor`
  exists, and the outer stays free to re-implement or re-route without touching them. Cost:
  duplication, one line per inner operation and another for each new one.
- **Getter — prefer composition to inheritance, and expose the composed object.** The whole inner
  API is reachable by construction, with no forwarder duplication. Cost: it leaks the abstraction
  and breaks Demeter. Callers can depend on inner details, which makes the inner type hard to
  replace, and can write inner state around the outer's invariants (`p->Status = …`).
- **Adaptation — the wrapper adds value.** It is not forwarding: the outer composes inner
  operations into a unit the inner cannot express alone, with state only the outer holds. It is
  not optional: without it the wrapper is only a namespace.

| Test | Surface |
|---|---|
| The operation needs the outer's state to be correct or atomic | Adaptation |
| Callers already know both types: Var tables, config, debug, tests | Getter |
| The inner API is large, stable and mostly read | Getter, plus forwarders for the hot paths |
| The inner is value-embedded, `Outer_State_T { Inner_State_T Inner; }`, small and bound to the outer's invariants | Forwarders, no getter |

- **Never return a mutable pointer to a value-embedded member.** A getter is for a component the
  outer holds in its descriptor or by pointer, whose lifetime is the outer's.

Coupling two peers, by the number of coordination points:

| Coordination | Shape |
|---|---|
| None | No wrapper, no bridge: callers compose directly |
| A few (≤ ~5), and the peers are public vocabulary | Free-function bridges `Subject_Collaborator_Verb()`, no wrapper type |
| Many, or the composite owns state of its own | Mediator: a wrapper type keeps only the coordination functions and exposes the peers by getter |
| The peers must be hidden from callers | Facade: forwarders for everything; rare |

A wrapper or bridge function takes at most two context pointers, most constant first, then the
values: `(p_const, p_state, value)`.

A wrapper type earns itself only when it adds a concept, not just indirection:

- it holds a changeable policy;
- it enforces a pairing invariant the bare calls can violate;
- it adds a cross-cutting decorator, such as locking or tracing.

Whichever it is, every pointer it holds is used at runtime, on the hot path: its members are
lifetime-coherent.

**In code**

- Adaptation — `AngleCounter_ResolveAngleDelta` in `Math/Angle/AngleCounter.h` scales `FreqD` by
  `UnitRef.AngleDt32PerCount`, which only the counter holds, into `Base.Delta`.
  `VBus_PollCaptureMonitor` in `Motor/Motor/VBus/VBus_Monitor.h` polls the monitor with a fresh
  sample averaged against the filtered `VBus_Pu`.
- Forwarders over a value-embedded member — `VBus_IsTriggeringEdge`, `VBus_PauseMonitor` and their
  siblings over `VBus_T.MonitorState`, in `Motor/Motor/VBus/VBus_Monitor.h`.
- Getter — `MotorController_Buzzer` in `Motor/MotorController/MotorController.h` returns the
  `BUZZER` descriptor member.
- Bridge — `Motor_Cia402_WriteControl(Motor_T *, Cia402_Adapter_T *, control)` in
  `Motor/MotProtocol/Cia402/Motor_Cia402.h`.
- Facade — none.

**Rejected**

- **Forwarders by default.** Every new inner predicate needs another forwarder; the commented-out
  `VBus_IsUnderNominal` and `VBus_IsOverNominal` in `VBus_Monitor.h` are that friction.
- **A getter to a value-embedded member.** Callers can write the inner fields (`p->Status = …`)
  around the outer's invariants, and the inner layout becomes the outer's contract.
- **A wrapper type for a few coordination points.** A type that only re-exports another type's API
  holds no state and enforces no invariant; free-function bridges say the same with no type.

**Source** — Information hiding (Parnas, "On the Criteria To Be Used in Decomposing Systems into
Modules", *CACM* 15(12), 1972): a module hides a design decision. An adaptation or a forwarder hides
the inner type and a getter exposes it, so the choice is whether the inner type is a decision to
keep changeable. The Law of Demeter (Lieberherr and Holland, "Assuring Good Style for
Object-Oriented Programs", *IEEE Software*, 1989): callers talk to the outer, not to what it holds.
Facade and Mediator (Gamma, Helm, Johnson and Vlissides, *Design Patterns*, 1994). Precedents: C++
`std::stack`, a facade forwarding to `std::deque`; Linux `netdev_priv()` and the smart pointer's
`.get()`, getters kept as escape hatches.
