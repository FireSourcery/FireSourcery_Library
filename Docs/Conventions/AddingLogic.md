# Adding Logic

A refactor takes logic out of a function. Adding logic to a function that works is a decision, weighed before it is
written, and what the new logic is decides where it goes:

| The new logic | Goes |
|---|---|
| A repair: the function is wrong on inputs it already takes | in place, inside the expression the function has |
| A constraint every caller meets, arguments known at compile time included | an `assert`, no runtime logic (`CLAUDE.md`, Key Conventions) |
| A check for correctness on inputs no caller bounds: wire data, user input | a function that wraps the original |
| A different behavior | a function of its own name, beside the original: `*_Config_IsValid` faults, `*_Config_Validate` repairs |

```c
/* Motor_Electrical.h: the guard for wire input wraps the conversion, which divides by si unguarded */
static inline angle_freq_t _Motor_Electrical_RateOfSi(int32_t si, uint32_t scale) { return (si > 0) ? angle16_of_arc(scale, si) : 0; }
```

## Why

- A working function's body is the equation in its docstring, checked at a glance. A guard folded into it turns the
  equation back into a procedure, and every caller pays for a check only some of them need.
- Wrapped, the guard is named for what it adds, and only the callers that need it call it. A caller whose inputs are
  already correct, compile-time constants among them, keeps the direct form, and the `assert` states its contract.
- Changing a function's behavior in place changes every caller at once. A second name leaves each caller's contract
  where it was, and keeps the variant later work may need.

Related: [ConfigValidity.md](ConfigValidity.md), where conversions and derives carry no guards;
[MergingDuplicates.md](MergingDuplicates.md), the reverse case, when two functions may become one.
