# Decisions

What we chose, and why. One record per decision. A record is not rewritten once accepted:
- A refinement that keeps the decision is appended to the record as a dated **Amendment**, and the Status line notes
  it.
- A change of decision is a later record that supersedes it.

The format follows Nygard's architecture decision records ("Documenting Architecture Decisions", 2011).

Settled outcomes live here: accepted, rejected or superseded. An open question waits as a brief in
[../Proposals/](../Proposals/README.md), and its outcome lands here.

---

## Entry shape

`NNNN_Title.md`, numbered in order.

```markdown
# NNNN Title

The decision in one line; the index's About column quotes it.

**Status** — proposed, accepted, or superseded by NNNN.
**Context** — the forces, and the findings behind them.
**Decision** — what we do.
**Consequences** — what it costs, what it settles, what stays open.
```

Link the findings a decision rests on, in [../Findings/](../Findings/README.md).

---

## Records

| Record | About | Status |
|---|---|---|
| [0001_ParameterWriteAdmission.md](0001_ParameterWriteAdmission.md) | "Is this write safe now?" is answered once, at the controller layer, not inside each module | Accepted |
| [0002_LinearThrottleLayering.md](0002_LinearThrottleLayering.md) | Traction interprets the lever and emits a signed command; Motor carries no vehicle policy | Accepted, not implemented |
| [0003_PhaseOutputOwnership.md](0003_PhaseOutputOwnership.md) | Each state LOOP that produces FOC output writes its own duty; no blanket write while gates are on | Accepted, for now |
| [0004_PiSlidingAntiWindup.md](0004_PiSlidingAntiWindup.md) | The integral clamp is the sliding band clipped to the output range; the anchored form fails in field weakening | Accepted |
| [0005_ILimitFrames.md](0005_ILimitFrames.md) | Derates split per quadrant, each limit keyed by the frame that defines it | Accepted |
| [0006_RampLimitsOnInput.md](0006_RampLimitsOnInput.md) | Ramp limits clamp the input; an output outside new bounds ramps back at slope | Accepted |
| [0007_SpeedPuBound.md](0007_SpeedPuBound.md) | ω_pu never exceeds 1.0; field weakening runs inside the same speed PID | Accepted |
| [0008_PersistenceById.md](0008_PersistenceById.md) | Persisted values are addressed by id, never by struct offset; struct shape follows ownership alone | Accepted |
| [0009_DirectionPolicy.md](0009_DirectionPolicy.md) | Direction stays a selectable lock policy; the symmetric-VLimit end state comes later | Proposed, deferred |
| [0010_SharedWordWrites.md](0010_SharedWordWrites.md) | Multi-word cross-thread values stay unguarded for now; each site carries the pointer here | Proposed, open |
| [0011_StateMachineLockNesting.md](0011_StateMachineLockNesting.md) | The state-machine lock stays the raw disable/enable pair; decide with 0010 | Proposed, open |
| [0012_VarEnumNaming.md](0012_VarEnumNaming.md) | The runtime var set is `Module_Var_T`; the config var set is proposed as `Module_ConfigVar_T`, the struct it indexes first and the kind last | Proposed, open |
| [0013_TractionMotorAdapter.md](0013_TractionMotorAdapter.md) | Traction reaches each motor through `Motor_Traction` on the Motor's own interface; the leaves fan out and keep the all-motor decisions; no common `Motor_Input_T` | Accepted, amended 2026-11-24 |

Pending: `Peripheral/ADC/ADC_Batch_JoinVariants.md`, an open decision.
