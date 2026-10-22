# Information Expert

**Statement** — Each layer answers the questions it alone has the knowledge for. A generic concern is
answered once, at one layer, rather than re-derived in every module that meets it.

**Rationale** — A question answered where its knowledge lives needs no other module's state, and its
answer has one home. A module that answers a question it has no unique knowledge of duplicates the
reasoning of every other module that meets the same question. Parameter writes are the example:
whether a write is safe while the system runs is a property of writing into any running system, not
of motor physics. Even PolePairs is generic in that sense.

**Implications**

- The motor layer answers motor-domain questions: physics, electrical safety, struct consistency,
  and the schema of its parameters.
- A parameter service at the controller layer answers system questions: sessions, lifecycle, write admission, and which
  parameters need which window. See [0001_ParameterWriteAdmission.md](../Decisions/0001_ParameterWriteAdmission.md).
- A domain-specific hazard stays with its domain: gating the phase outputs on motor state is motor
  logic.

**Source** — Information Expert, GRASP (Larman, *Applying UML and Patterns*).
