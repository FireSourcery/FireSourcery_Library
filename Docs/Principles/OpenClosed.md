# Open–Closed

**Statement** — A working function is closed to modification and open to extension: a repair goes
in place, and other new logic extends it from outside, as a wrapping function or a sibling with its
own name. Adding logic to a working function is a decision.

**Rationale** — A working function is a contract every caller holds, and its body is the equation in
its docstring. Logic added in place changes the contract for every caller at once, makes each pay
for a check only some of them need, and turns the equation back into a procedure. Added outside,
the logic is named for what it adds and reached only by the callers that need it, and the original
keeps its expression and its callers' contracts. Added logic changes what some caller gets, so it
is weighed before it is written. A repair is the exception: a function wrong on inputs it already
takes has no correct contract to keep closed. The kinds of change, and what each needs:
[KindsOfChange.md](KindsOfChange.md).

**Implications**

- Where added logic goes, by what it is — in place, an `assert`, a wrapper, a name of its own:
  [AddingLogic.md](../Conventions/AddingLogic.md).
- Conversions and derives carry no guards; validity is checked once, at the config layer:
  [ConfigValidity.md](../Conventions/ConfigValidity.md).
- With [GeneralForm.md](GeneralForm.md): there the principle holds at the interface, whose consumers
  take a new type without modification; here at the function, whose callers keep their contract
  when logic is added.

**Instances**

- `_Motor_Electrical_RateOfSi` wraps `angle16_of_arc` with the guard for wire input; the conversion
  still divides unguarded: `Motor/Motor/Motor_Electrical.h`.
- `Motor_Config_IsValid` faults and `Motor_Config_Validate` repairs: two behaviors, two names,
  `Motor/Motor/Motor_Config.h`. The same pair in `Motor/Motor/VBus/VBus_Config.h`.

**Source** — The Open–Closed Principle (Meyer, *Object-Oriented Software Construction*, 1988),
applied to functions; the repair exception is this repo's.
