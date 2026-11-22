# 0012 Var interface enum names

The runtime var set is `Module_Var_T`; the config var set is proposed as `Module_ConfigVar_T`, the struct it indexes first and the kind last.

**Status** — proposed, open. From the owner's note of 2026-11-22: `Module_Var_T` for the runtime set;
the config set open among `Module_ConfigId_T`, `Module_ConfigVar_T` and `Module_VarConfig_T`.

**Context** — A module serves the var interface ([0008](0008_PersistenceById.md)) through two id
enums. One indexes the runtime object: live state, virtual fields, getters. The other indexes
`Module_Config_T`, a field per id in most cases. Each is the base enum of one `MotVarId_T` type row
([VarIdMap.md](../../Motor/MotProtocol/MotVarId/VarIdMap.md)), and the app's hand-written id table
copies the C type and member names verbatim. The wire carries only (Prefix, Type, Base) numbers.

| Form in code | Runtime set | Config set | Members |
|---|---|---|---|
| `Module_VarId_T`, `Module_ConfigId_T` | 9: Hall, VBus, Encoder, Monitor, RangeMonitor, UserDIn, Shifter, Traction, MotAnalogUser | 16: the same 9, plus CAN, Socket, SinCos, OptDin, Thermistor, VDivider, UserAIn | runtime `MODULE_VAR_*` 6, `MODULE_VAR_ID_*` 1 (VBus), bare 2; config `MODULE_CONFIG_*` 10, `MODULE_CONFIG_ID_*` 2 (VBus, OptDin), bare 4 |
| `Module_Var_T`, `Module_ConfigVar_T` | `EmfObserver_Var_T` | `EmfObserver_ConfigVar_T` | `EMF_OBSERVER_VAR_*`, `EMF_OBSERVER_CONFIG_VAR_*` |
| Facet groups, `Module_Var_<Group>_T` | `Motor_Var_UserOut_T` and 6 more; `MotorController_Var_Output_T` and 3 more | `Motor_Var_Config{Calibration, Actuation, Pid, Electrical, Debug}_T`, `MotorController_Var_Config_T` | `MOTOR_VAR_*`, `MOT_VAR_*`: the group dropped |
| Named off their module | `Motor_Var_Foc_T` in `FOC.h`, the type `FOC_Var_Get` takes; `Motor_Var_Rotor_T` in `RotorSensor.h`; `Phase_InputVar_T`, the type `Phase_Input_Var_Get` takes | — | |

The id-keyed functions follow the enums: `Module_VarId_Get` at about 27 sites, `Module_ConfigId_Get/Set`
at about 85. Why the config set carries a second word at all: `Module_Config_T` is the struct, so the
enum over its fields needs another, as `Motor_StateId_T` does beside `State_T`. Nothing is named
`Module_Var_T` yet, so the runtime set can take it bare.

**Options** — the runtime set is `Module_Var_T` in each.

| | A `Module_ConfigId_T` | B `Module_ConfigVar_T` | C `Module_VarConfig_T` |
|---|---|---|---|
| Reads | the ids of the config's fields | the config struct's var set | the var interface's config set |
| Pairs with `Module_Var_T` | no: one ends in Id, the other in Var | yes: both end in the kind | yes: both start with Var |
| Last word, where the role suffixes sit (`_Config_T`, `_State_T`, `_Ref_T`) | Id | Var | Config, the persisted-struct suffix: `Hall_VarConfig_T` beside `Hall_Config_T` reads as a second config |
| Sorts beside | `Module_Config_T` | `Module_Config_T` and the `Module_Config_*` functions | `Module_Var_T` |
| Id-keyed functions | `Module_ConfigId_Get(const Module_Config_T *, id)`, as today | `Module_ConfigVar_Get(const Module_Config_T *, id)`: the prefix names the struct taken | `Module_VarConfig_Get(const Module_Config_T *, id)` |
| Members by [Naming.md](../Conventions/Naming.md) § Types | `MODULE_CONFIG_ID_*`, 2 of 16 today | `MODULE_CONFIG_VAR_*` | `MODULE_VAR_CONFIG_*`, inside the runtime set's `MODULE_VAR_` prefix |
| Precedent | 16 enums; [Config_State_Ref.md](../Conventions/Config_State_Ref.md), 0008 | `EmfObserver_ConfigVar_T` | the facet groups; the app's own `MotVarConfig*` classes |
| Renames past `VarId` to `Var` | none | 16 enums, their functions, the app table | as B |

All three rename the 9 `Module_VarId_T`, their `_VarId_Get` functions and VBus's `VBUS_VAR_ID_*`
members. Keeping `Module_VarId_T` with `Module_ConfigId_T` renames nothing and pairs by Id, but leaves
the owner's runtime choice and the `Var` of Motor, MotorController and EmfObserver.

**Recommendation** — B, `Module_ConfigVar_T`.

1. One family by the last word. Naming sets a type's kind last; B ends both sets in Var. C ends the
   config set in Config, the suffix this codebase gives a persisted struct (`VMonitor_Config_T`,
   `Motor_Config_T`).
2. The scope is the struct indexed. The set maps `Module_Config_T`'s fields, and
   [ApiScoping.md](../Conventions/ApiScoping.md) names a scope by its struct:
   `Module_ConfigVar_Get(const Module_Config_T *, ...)` says what it takes and sorts with
   `Module_Config_IsValid`.
3. Members part at the first differing segment: `HALL_VAR_SENSOR_ID`, `HALL_CONFIG_VAR_SENSOR_TABLE_1`.
   Under C a config member extends the runtime prefix, and Motor's facet form shows the cost:
   `MOTOR_VAR_POLE_PAIRS` (config) and `MOTOR_VAR_SPEED` (runtime) look alike.

C's case is the facet groups, and it does not carry over: there the `Var` segment is the facet
([Facets.md](../Patterns/Facets.md), `Motor_Var.h`), a scope of a top-level object that holds more
sets than one name serves. A single-set module has no Var facet; its two sets are one per struct. A's
case is no config rename; it loses the pairing, `Hall_Var_T` beside `Hall_ConfigId_T`.

**Owner's boxes**

- [x] Config set: A / **B** / C
- [x] Members: **the full type name, `MODULE_CONFIG_VAR_*`**, since every config enum is renamed in
  the same pass / keep the short `MODULE_CONFIG_*` of 10 of 16 as an allowed form
- [x] Sets named off their module (`Motor_Var_Foc_T`, `Motor_Var_Rotor_T`, `Phase_InputVar_T`):
  **named by the module whose header defines them**, `FOC_Var_T`, `RotorSensor_Var_T`,
  `Phase_Input_Var_T`, matching `FOC_Var_Get` and `Phase_Input_Var_Get` / left in Motor's facet

**Decision** — none until the owner rules.

**Consequences**

- One rename pass, the C and the app's id table together, carrying the 7 format-suffix renames that
  wait for the same table (`VBUS_VAR_ID_*_FRACT16/32`, `MOTOR_VAR_V_SPEED_RATED_FRACT16`,
  `OPT_DIN_CONFIG_ID_*_FRACT16`). Values and order stay, so the wire ids do not change; the image
  should be byte-identical, checked on both boards.
- Docs updated with it: Naming.md § Types gains the var set row; Config_State_Ref.md § Config
  interface renames `Module_ConfigId_Get/Set`. 0008 stays as written.
- Unchanged: the facet groups `Motor_Var_<Group>_T` and `MotorController_Var_<Group>_T`, the
  `MotVarId` type rows (`MOTOR_VAR_TYPE_*`), and the holder-level setter that takes the runtime
  (`VBus_Config_Set(VBus_T *, id, value)`), which [ApiScoping.md](../Conventions/ApiScoping.md) names
  by the widest struct it reaches.
