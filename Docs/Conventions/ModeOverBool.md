# Mode over Boolean

A gate, enable or pause on a state struct is a value of an enum mode that the poll `switch`es on, not
a single-purpose `bool`.

- **Extend an existing mode, direction or state enum first.** `Monitor_T.Direction` carries
  `MONITOR_DISABLED` as one more direction. `_TimerT_Modal_Poll` has `TIMER_MODE_DISABLED` and
  `TIMER_MODE_STOPPED` as states.
- **Otherwise add a mode enum with `DISABLED = 0`**, so a zero-initialized struct is safe.
  `RangeMonitor_Mode_T {DISABLED, PAUSED, ACTIVE}` replaced `RangeMonitor_T.IsActive`.
- **Transitions are explicit.** `RangeMonitor_InitFrom` resolves `DISABLED` or `ACTIVE` from the
  Config and sets the mode last; Pause and Resume move only `ACTIVE` ↔ `PAUSED`, so `DISABLED` is
  sticky.
- **A runtime hold lives in state, never in Config.** `UserDIn_Mode_T` is a prepared mode enum,
  two live values and commented future modes; `Modal_Disable` overrides `P_STATE->Mode` without
  persisting.

The state diagram for `RangeMonitor_Mode_T` is in [Config_State_Ref.md](Config_State_Ref.md), Config
interface.

Why: a flag beside a state splits one concept across two fields and grows the struct. One enum keeps
the transitions in one switch and leaves room for the next mode. Rejected instances:
`RangeMonitor_T.IsActive`, `VBus_T.IsMonitorPaused`.
