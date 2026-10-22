# 0001 Parameter write admission at the controller layer

**Status** — accepted.

**Context** — Some parameters are unsafe to write while the motor runs. The question "is this write
safe now?" was answered inside the motor module, by predicates such as [Motor_IsConfig] gating the
writes. That reasoning is generic: active readers must not consume a value mid-mutation, for any
parameter in any running subsystem. The motor module has no unique knowledge of it; see
[InformationExpert.md](../Principles/InformationExpert.md).

**Decision**

- The motor module owns its schema (the Motor_Var ids and their mapping to fields), its setters and
  their consistency side effects, and the state machine that gates the phase outputs. Setters apply
  the value and return; they check no state.
- The controller layer owns write admission: the session and lifecycle, and which window each
  parameter's write needs. It calls the motor setters only on admission.
- Which window a parameter needs is metadata on the schema, read by the controller layer.

**Consequences**

- Each module exposes schema, setters and classification, never "is it safe to write now".
- The write gate in `Motor_MotVar.c` (line 278) is already commented out. The predicate itself,
  [Motor_IsConfig] in `Motor/Motor/Motor_StateMachine.h`, is still defined.
- Gating the outputs on motor state stays in the motor state machine: that hazard is motor-domain.
