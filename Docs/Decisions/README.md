# Decisions

What we chose, and why. One record per decision. A record is not rewritten once accepted; a later
record supersedes it. The format follows Nygard's architecture decision records ("Documenting
Architecture Decisions", 2011).

---

## Entry shape

`NNNN_Title.md`, numbered in order.

```markdown
# NNNN Title

**Status** — proposed, accepted, or superseded by NNNN.
**Context** — the forces, and the findings behind them.
**Decision** — what we do.
**Consequences** — what it costs, what it settles, what stays open.
```

Link the findings a decision rests on, in [../Findings/](../Findings/README.md).

---

## Records

| Record | Status |
|---|---|
| [0001_ParameterWriteAdmission.md](0001_ParameterWriteAdmission.md) | Accepted |
| [0002_LinearThrottleLayering.md](0002_LinearThrottleLayering.md) | Accepted, not implemented |
| [0003_PhaseOutputOwnership.md](0003_PhaseOutputOwnership.md) | Accepted, for now |
| [0004_PiSlidingAntiWindup.md](0004_PiSlidingAntiWindup.md) | Accepted |
| [0005_ILimitFrames.md](0005_ILimitFrames.md) | Accepted |
| [0006_RampLimitsOnInput.md](0006_RampLimitsOnInput.md) | Accepted |
| [0007_SpeedPuBound.md](0007_SpeedPuBound.md) | Accepted |
| [0008_PersistenceById.md](0008_PersistenceById.md) | Accepted |
| [0009_DirectionPolicy.md](0009_DirectionPolicy.md) | Proposed, deferred |

Pending: `Peripheral/ADC/ADC_Batch_JoinVariants.md`, an open decision.
