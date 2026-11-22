# Parameterized Function

A parameter earns its place in two ways: on the API it trims the surface, one function over the values of one body; behind the API it lets internals reuse one implementation, its optional slots never reaching a caller. A parameter that picks which logic runs is a flag, and its variants stay separate functions.

**Level** — idiom. C has no overloading and no default arguments, so a family of variants is either N
functions or one function with a parameter.

**Force** — API scoping and focused HAL signatures want each function to take only what it uses,
which pushes against optional parameters and the super function, a mode argument that picks the
logic inside. Variants written one function each pay the other way: a direct path from every call,
with no conditional inside, at the cost of N entries in the API and N copies of the body. Where the
parameter sits decides it: on the API, or behind it.

**Shape**

| The parameter sits | It buys | Accepted when | Instance |
|---|---|---|---|
| On the API, header-visible | A smaller API: one function in place of one per value | the value is data through one body, with no branch picking the logic, and every caller passes a real argument; a constant call folds to its variant when inlined | `_Phase_WriteGates(p_phase, id)`, over the eight gate states |
| Behind the API: file `static`, or reached only through a table | One implementation under several API functions, none of which shows the parameter | always: each API function still takes only what it uses | `ApplyUpUntil` under the `State_*Up` functions; `StartCmdWritePage`, `StartCmdEraseSector` as `START_CMD` |
| On the API, picking which logic runs | — | never: a flag | `Phase_Unipolar_ActivateOutput`, `Phase_Bipolar_ActivateOutput`; the `PolarMode` switch over them is commented out |

```c
/* On the API: one entry over the gate states. The id is the gate mask */
static inline void _Phase_WriteGates(Phase_VOut_T * p_phase, Phase_Id_T id)
{
    const Phase_Bitmask_T state = Phase_Bitmask(id);
    _Phase_WriteOnOffA(p_phase, state.A);
    _Phase_WriteOnOffB(p_phase, state.B);
    _Phase_WriteOnOffC(p_phase, state.C);
}
/* Not: _Phase_WriteGatesA(p_phase), _Phase_WriteGatesAB(p_phase), ... eight entries */

/* Behind the API: one traversal; each API function passes its own bound and visitor */
static inline void * ApplyUpUntil(State_T * p_start, State_T * p_end, void * p_context, State_Visitor_T fn, int opt);
State_Input_T State_AcceptInputUp(State_T * p_start, state_input_t id) { return ApplyUpUntil(p_start, NULL, NULL, AcceptInputVisitor, id); }
State_T * State_TransitionOfOutputUpTo(State_T * p_start, State_T * p_end, void * p_context) { return ApplyUpUntil(p_start, p_end, p_context, TransitionOfOutputVisitor, 0); }
```

The second block is simplified: `State_AcceptInputUp` goes through `ApplyUp`, the `p_end == NULL` form.

- **On the API.** `_Phase_WriteGates` is protected by its `_`, but it is the surface the Phase parts
  share: `Phase_VOut.c` and `Phase_Polar.c` call it. One entry serves the constant calls, named for
  the operation (`Phase_Deactivate` passes `PHASE_ID_0`, `Phase_ActivateOutput` passes `PHASE_ID_ABC`),
  and the runtime ones, where the commutation's id passes through (`Phase_Unipolar_ActivateOutput`).
  Function-level selection would give each constant call a direct path, and a runtime id a `switch`
  at the caller. The concise form is preferred.
- **Behind the API.** No API function shows the internal's parameters. `State_AcceptInputUp(p_start, id)`
  takes no context and no end, and `State_TransitionOfOutputUp(p_start, p_context)` no `opt`. On the
  flash, `Flash_Erase_Blocking(p_flash, address, size)` takes no data and `Flash_EraseAll_Blocking(p_flash)`
  nothing, though every `START_CMD` target takes all four. The HAL under them keeps its focused
  signature, `HAL_Flash_StartCmdEraseSector(p_regs, destAddress)`.
- **Merging near-duplicates on the API** is honest only when every caller passes a real argument:
  the quantities share a dimension. Before proposing a collapse, check every caller. If one would pass
  a constant 1, 0 or `NULL` only to fit the shape, keep the functions separate and name each by its
  quantity. A placeholder makes the call misstate the quantity (ψ has no current base), and it hides
  that the bases differ: Z_base is V_base for ψ and V_base / I_base for L. A tramp parameter fails the
  same test ([ApiScoping.md](../Conventions/ApiScoping.md): a signature states what the function
  depends on).
  - Accepted: one SI pair serves ψ and L, `_Motor_Electrical_SiOf` / `_Motor_Electrical_RateOfSi`
    over `angle16_arc_radius` / `angle16_of_arc`. ψ per volt and L per ohm are both [s/rad], so no
    call passes an extra argument.
  - Rejected: `pu_tau_of_angle_freq(base, v_base, i_base, rate)`, where ψ passed `i_base = 1`. Kept
    separate: `psi_pu_of_angle_freq` (no `i_base`), `l_pu_of_angle_freq` and `angle_freq_of_l_pu`.
- Behind the API no call site shows the slot, so the placeholder test does not apply there: `NULL` as
  the end of the path and a `(void)`ed slot are internal details.
- Folding is expected, not measured. `_Phase_WriteGates` carries "Let the compiler optimize/expand
  into the 8 derived functions". `ApplyUpUntil` reaches its visitor through a pointer that is
  constant once inlined. Neither is checked in the disassembly.

**In code**

- `_Phase_WriteGates` in `Motor/Motor/Phase/Phase_VOut.h`. Its constant callers `Phase_Deactivate`
  and `Phase_ActivateOutput` are in `Phase_VOut.c`, its runtime callers in `Phase_Polar.c`.
- `ApplyUpUntil`, `ApplyUp`, `ApplyUntilRoot` and the visitors in
  `Framework/StateMachine/_State_Node.c`, under the `State_*Up` / `State_*Until` functions of
  `_State_Node.h`.
- The `START_CMD` targets in `Peripheral/NvMemory/Flash/Flash.c`, typed by `HAL_NvMemory_StartCmd_T`
  (`Peripheral/NvMemory/NvMemory/NvMemory.h`), under the `Flash_*_Blocking` functions of `Flash.h`.
- Merges: `_Motor_Electrical_SiOf`, `_Motor_Electrical_RateOfSi` in `Motor/Motor/Motor_Electrical.h`,
  called for ψ, Ld and Lq in `Motor_Config.c`; the split kept in `Motor/Motor/Math/motor_electrical_math.h`.

**Rejected**

| Alternative | Why not |
|---|---|
| A function per value on the API, `_Phase_WriteGatesA(p_phase)` | Eight entries of one body; a runtime id needs a `switch` at the caller |
| A merge one caller feeds a placeholder, `pu_tau_of_angle_freq(..., i_base = 1, ...)` | The call misstates the quantity and hides that the bases differ |
| A mode argument over different operations, the super function | The callers pass literals and the body branches: the call no longer says what runs, and every caller depends on every path |
| Exposing the internal's shape, `State_AcceptInputUp(p_start, NULL, NULL, id)` | The API takes what it does not use: the optional parameter API scoping refuses |
| An implementation per API function: a traversal per bound and visitor, a `START_CMD` type per command shape | The same loop several times; the NvMemory engine dispatches one slot, and a type per shape needs a dispatch per shape (the alternative noted at `HAL_NvMemory_StartCmd_T`) |

**Source** — Parameterize Function and Remove Flag Argument (Fowler, *Refactoring*, 2nd ed., 2018),
and "FlagArgument" (Fowler, bliki, 2011). Fowler's test separates the first row from the third: an
argument is a flag when the callers pass a literal and the body uses it for control flow, not as
data. The hidden row is information hiding (Parnas, *CACM* 15(12), 1972): the shared implementation
is a decision the module keeps behind its interface.
