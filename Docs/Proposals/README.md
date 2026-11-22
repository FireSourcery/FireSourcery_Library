# Proposals

Deferred blocks of library work, one self-contained brief each, kept here until they land.

The pattern is [Patterns/PlannedItemBrief.md](../Patterns/PlannedItemBrief.md). The `/proposal` skill holds the shape
(**Status**, Context, Design, Sketch, Done already, Open for you) and the lifecycle. The owner answers by ticking the
boxes in **Open for you**. Applying an answer records the chosen option in the brief.

**Landing** — the library keeps no dated log folder. A landed brief's outcome goes where its **Status** line names
(a Decision, a convention, or a module's `.exp.md` record). The brief and its row here are then deleted. A brief
abandoned instead of landed is recorded the same way, with the reason.

| Brief | About | Waits on |
|---|---|---|
| [NoCommentsInSource.md](NoCommentsInSource.md) | Whether source files keep any comments, or all documentation moves to the `.md` pair beside the source. | Owner: rule A, B or C; parking and license rulings |
| [ShortCircuitFwBound.md](ShortCircuitFwBound.md) | Bound the field-weakening speed ceiling by the back-EMF a floating bridge can stand, and raise it toward I_sc = ψ / Ld only where V0 is the safe state. | Fault-type output selection; the motor's demagnetization current; V0-at-speed hardware tests |
| [StatorInductanceLs.md](StatorInductanceLs.md) | Settle what `Motor_Ls_PuTurn` means: the αβ average (Ld + Lq) / 2 it returns today, or the per-phase winding self-inductance. | A reader that needs Ls; owner: the definition |
| [DirectionEndState.md](DirectionEndState.md) | Replace the direction lock's anti-plugging half-plane with a symmetric v_q limit plus a window around the back-EMF, once the readers that are not about the lock stop reading the direction. | Moving the non-lock readers of `Motor_Direction()`; owner: end form, window |
| [SubStateIds.md](SubStateIds.md) | Allocate path ids to the motor's and Traction's sub-states, and give MAIN's depth-1 id space one owner so the controller's and the app's ids cannot collide. | Owner: MAIN's id split, motor numbering scope; the app mirror |
| [SafeStopGaps.md](SafeStopGaps.md) | Close or record the three gaps between `SAFE_STOP` and the stop functions it is mapped to: no restart interlock, an exact-zero standstill test, and no id to report Quick Stop Active. | [SubStateIds.md](SubStateIds.md) for the report; owner: end state, standstill test |
| [PhaseOutputImage.md](PhaseOutputImage.md) | Make the PWM thread the only phase-register writer: the states write a duty-domain output image, and the thread transfers it. | Owner: output `Flags` meaning; the one-period gate-off delay |
