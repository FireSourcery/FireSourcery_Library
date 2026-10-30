# 0008 Persisted values are addressed by id

**Status** — accepted 2026-10-04.

**Context** — Persisted config structs (`Motor_Config_T`, `Encoder_Config_T`, `KMC_NvMemory_T`)
change shape as modules evolve. Earlier changes were bent to keep the stored layout: a same-size
in-place retype of `Encoder_Config_T.AngleFreqBase`, and "devices need a reset to defaults" listed
as a cost.

**Decision** — Tools and the app address persisted variables by id, through the VarId / ConfigId
interface (`_Encoder_ConfigId_Get/Set`, `Motor_Var`, `MotVarId`), never by struct offset. Struct
contents are designed on ownership and meaning alone.

**Consequences**

- Adding, removing, retyping or reordering fields of a persisted struct is free. Do not keep a
  field, pick a same-size type or preserve field order to protect the layout, and do not report a
  layout change as a cost.
- The id tables (`*_ConfigId_T`, `*_VarId_T`) are the stable interface. Adding, removing or
  renumbering ids is the change that needs care, together with the app's `mot_var_id.dart` mirror.
