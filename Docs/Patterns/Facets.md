# Facets


Each facet is a complete view of the object along one axis. They don't replace each other; they coexist. A reader who needs the parameter view goes to *_Var.h and gets a complete answer for that axis. A document is classified by multiple orthogonal axes — subject, format, time, place, audience — rather than a single hierarchical category. This is the literal origin of the term "facet."


**Level** — architectural: every top-level object, [Motor_T] and [MotorController_T], is split this
way.

**Force** — One object serves several kinds of client: the user's commands, the variable system, the
control math, the lifecycle. Each client needs a complete view along its own concern. What stays
still is the object and its state, which all the views share.

**Shape** — One header per concern, each a complete view of the same object. A facet may build on
another: `Motor_User.h` includes `Motor_StateMachine.h` and `Motor_FOC.h`.

```text
Motor_User.h           external commands
Motor_Var.h            the variable system: ids to fields
Motor_FOC.h            control math on the motor's FOC state
Motor_StateMachine.h   lifecycle and transitions
```

A reader who needs one view opens one header and gets a complete answer for that axis. The views
coexist; none replaces another.

**In code**

- [Motor_T] — `Motor/Motor/`: `Motor_User.h`, `Motor_Var.h`, `Motor_FOC.h`, `Motor_StateMachine.h`
- [MotorController_T] — `Motor/MotorController/`: `MotorController_User.h`, `MotorController_Var.h`, `MotorController_StateMachine.h`

**Rejected**

- **One header for the whole object.** Every client depends on every concern, and the header grows with all of them.
- **One object per concern.** It splits the state the concerns share, and needs forwarders between the objects.
- **A file per subtype.** The concerns are views of one object, not kinds of it.

**Source** — Faceted classification (Ranganathan, *Colon Classification*, 1933). Role interfaces,
one interface per kind of client (Fowler, "RoleInterface", 2006). See also
[DominantDecomposition.md](../Principles/DominantDecomposition.md).
