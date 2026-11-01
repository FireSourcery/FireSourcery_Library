Wrapping Lower Components — Design Analysis

The two referenced slices illustrate three distinct strategies for exposing subcomponent functionality. Each has a principle behind it, and they shouldn't be used interchangeably — mixing them is what creates the "scattered API" feeling.

Strategy 1 — Thin forwarding ("delegation" / "pass-through")

 Principle: the wrapper is the public façade. The caller should only need to know the outer type. Applies the Law of Demeter / "don't talk to strangers" — callers of VBus don't need to know RangeMonitor exists.

When to use it:

- The inner function is part of the outer module's public contract.
- You want freedom to re-implement or re-route later without touching callers.
- There are few enough of them to enumerate, and they're stable.

Cost: duplication. Every new RangeMonitor predicate needs a new forwarder. The // ... forwarders commented out in earlier VBus revisions were exactly this friction.

Used by: STL std::stack (façade over deque), AUTOSAR MCAL wrappers around LLD, Zephyr shell over log.

Strategy 2 — Getter returning the sub-component ("exposed composition")

 Principle: "prefer composition to inheritance; expose the composed object." Callers who need the full RangeMonitor API call it directly on the returned pointer. No forwarder duplication — the entire RangeMonitor API is reachable by construction.

When to use it:

- The inner module has a large, open-ended API you don't want to shadow.
- Callers are sophisticated enough to know the inner type exists (e.g., debug UI, Var/protocol table, config editor).
- The inner object is owned by the outer and its lifetime is bound — you're not leaking an aliased pointer that someone else might free.

Cost: leaks abstraction — callers can now depend on RangeMonitor details, making the inner type hard to replace. Breaks Demeter. Makes it possible to mutate inner state without going through the outer invariants (e.g., direct p->Status = ...).

Used by: Linux netdev_priv(), Zephyr CONTAINER_OF macro families, C++ STL .get() on smart pointers. All three are pragmatic "escape hatches" that the docs explicitly label as lower-tier API.

Strategy 3 — Re-composed / transformed ("adaptation")

Principle: the wrapper adds value. This is not forwarding — the outer function composes inner operations into a unit the inner module couldn't express alone (FreqD → Base.Delta requires Ref, which lives at the outer scope; CaptureAndPoll couples two state structs through a fresh sample).

When to use it: whenever the outer module has information that makes the combined operation atomic and safer than calling the parts.

Not optional — this is the core value of wrapping at all. If you remove these, the wrapper is just a namespace.

Guiding Principles (which to pick when)

|   |   |
|---|---|
|Test|Strategy|
|Inner operation needs outer's ancillary state to be correct or atomic|3 — Adaptation|
|Inner operation is frequently used and part of outer's public contract|1 — Forwarder|
|Inner is queried from debug/config/test code that already knows both types|2 — Getter|
|Inner API is large, stable, mostly-read|2 — Getter, maybe + small set of 1 forwarders for hot paths|
|Inner API is small and tightly bound to outer invariants|1 — Forwarders, no getter|

Two published formulations of the same rule:

- Parnas (1972), On the criteria to be used in decomposing systems into modules: a module hides a design decision. A forwarder hides; a getter exposes. Choose based on whether the inner type is a decision you want to keep changeable.
- Meyer, Object-Oriented Software Construction (1988), §11.7 Open-Closed: the external interface of a module should be closed to modification but open to extension. Getters keep the module open (more inner behavior reachable without edits) at the price of being less closed (inner changes propagate).


## Collaborator
The Subject_Collaborator_* Collaborators follow, verb last.
When coupling two peers:
0 coordination points → no wrapper, no bridge. Callers compose directly.
≤ ~5 coordination points, peers are public vocabulary → Multi-context free functions (Subject_Collaborator_*). No wrapper type.
Many coordination points, or composite owns meaningful state → Mediator wrapper with peer getters (Option A).
Peers must be hidden from callers → Full-forwarder facade (Option B).



- Considerations:
2 pointer context API signatures at most - (p_const, p_state, value)
Does the wrapper hold a changeable policy?
Does the wrapper enforce a pairing invariant the bare form can violate?
Does the wrapper add a cross-cutting decorator like locking/tracing?
Is every pointer in the wrapper used at runtime on the hot path? (lifetime-coherent)
Adding indirection without adding concept vs enforcing pair correctness
