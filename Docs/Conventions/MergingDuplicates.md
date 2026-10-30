# Merging Duplicates

Merge near-duplicate functions into one generic only when the merge is honest for every caller: the
quantities share a dimension, so every call passes real arguments.

- **Accepted**: `angle_freq_of_s_per_rad` / `s_per_rad_of_angle_freq` replace the ψ and L SI pairs.
  ψ per volt and L per ohm are both [s/rad], so one call serves both with no extra argument.
- **Rejected**: `pu_tau_of_angle_freq(base, v_base, i_base, rate)`, where ψ passed `i_base = 1`.
  Kept separate: `psi_pu_of_angle_freq` (no `i_base`), `l_pu_of_angle_freq` and
  `angle_freq_of_l_pu`.

Before proposing a collapse, check every caller's arguments. If one would pass a constant 1, 0 or
`NULL` only to fit the shape, keep the functions separate and name each by its quantity.

Why: a placeholder argument makes the call site misstate the quantity (ψ has no current base), and it
hides that the bases differ: Z_base is V_base for ψ and V_base / I_base for L. The same reasoning
rejects tramp parameters (`CLAUDE.md`, API Design).
