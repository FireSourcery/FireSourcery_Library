# Simulation

Numeric models, plants and fixed-point replays behind the motor docs. A simulation run during a
design lands here, not only in a scratchpad, with its plot saved beside it.

| Path | Holds |
|---|---|
| `Svpwm/` | SVPWM ceiling and blend sets, zero-sequence placement, low-side window and hold models, `MidClampBlend.md` |
| `ISample/` | PWM sample sequences |
| `pwm_sample_timing.py` | PWM sample timing |
| `atan2.py` | atan2 |

## Conventions

- **Names.** Script snake_case, PNG PascalCase: `low_side_window.py` → `LowSideWindow.png`.
- **A topic with several views follows the ceiling set** (`Svpwm/svpwm_ceiling.py`):
  - the model script mirrors the C function and draws the per-cycle panels;
  - `<name>_analysis.py` holds the sweep and prints the checks;
  - each further figure is its own `<name>_<plot>.py`, importing those.

  The blend set (`svpwm_blend*.py`) has the same shape.
- **Board-agnostic.** An MCU family as example hardware is fine: KE, S32, their FTM and ADC
  registers. Board names, schematic designators, the board's rated bus voltage, `BOARD_*` macros,
  board source paths and the gate driver's part number stay in the board repo; say "gate driver"
  and keep its numbers as the example's. A board's own simulations live in its repo's
  `Docs/Simulations/`, and link here by relative path.
- **Output.** Printed text is ASCII: the Windows console is cp1252, so `√` fails. Unicode is fine in
  plot labels. Call `savefig` before `plt.show()`, and run with `MPLBACKEND=Agg` when there is no
  display.
- **Cite.** The doc that states a result links the script and the PNG.
- **Environment.** matplotlib is installed for the system Python 3.14
  (`C:\Program Files\Python314\python`), not in a repo `.venv`.
- **Not here.** Build and verification byproducts (replays, objdiff) stay in the scratchpad.
  Non-motor tests go in `.Test/`, which is git-ignored.
