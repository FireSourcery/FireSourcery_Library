# Naming

How names are built. An underscore separates scope segments, and case separates the words inside a
segment, so `Motor_FOC_ProcAngleV` reads Motor › FOC › ProcAngleV. Upper case gives a second
delimiter, which leaves the underscore free to mark scope.

---

## What a name says

**A name says what the thing is, not how it relates to something else.** A qualifier names a
relation: the rate a value is measured against, a direction, where the thing is used. It may come
after the name to refine it, but never takes the name's place. Names then sort by concept, the way
an object's methods sit under the object.

**Backed by** a principle that has no file yet ([README.md](README.md)).

| A name for | Says what it is | Instead of |
|---|---|---|
| A quantity's unit or base | `_pu_turn`, `_pu_rad`: the angle per Ts at ω_pu = 1.0, in turns or in radians (2026-10-15) | `_pu_nyquist`, a qualifier; `_pu_tick`, which names time for an angle base |
| Each of two asymmetric operations | what triggers it, whether it answers a request, what goes in and what comes out | a direction pair: Tx/Rx, Producer/Consumer (2026-09-27) |
| A function | its group before the verb, its variant after ([§ Functions](#functions)) | `StateMachine_CallInput_Branch` |
| A compound | the noun first, then its qualifiers ([§ Word order](#word-order)) | a qualifier first: MinOutput |

**Asymmetric operations get asymmetric names.** For the CAN handlers, the Rx handler takes a peer's
frame as a request and may answer it. The Tx builder turns node state into a frame on the node's
own clock, unsolicited. A direction pair would name them as mirror images, and they are not. Look
in the library's vocabulary first (`Protocol_Req_T`, `Protocol_ProcReqResp_T`). Then try standard
terms: J1939-21 Request, Command and Broadcast-Response, or OSI confirmed and unconfirmed services.

---

## Case

| Kind | Form | Example |
|---|---|---|
| Function, type, struct field | PascalCase segments joined by `_` | `Motor_FOC_ProcAngleV`, `Stator_Winding_T`, IntegralAccum |
| Pure function, value typedef | snake_case | `stator_emf`, `fract16_t` |
| Local variable, parameter | camelCase | vqCircleLimit, idReq |
| Macro, enum member, build-time definition | UPPER_SNAKE | `PID_MODE_PI`, `MOTOR_STATE_RUN` |
| Field of a `const` descriptor | UPPER_SNAKE | BASE, P_STATE in `Hall_RotorSensor_T` |

**Acronyms.** An acronym that forms a whole segment stays upper case: `FOC_T`, `PID_T`, `ADC_T`,
`HAL_`, `MotorController_PWM_Thread`. Inside a word it is written as a word: `AnglePll_T`,
`Phase_WriteSvpwm`, Heat_Adcu.

**Upper case means fixed.** A variable is never all caps, so it cannot be mistaken for a constant:
`PwmA`, not `PWM_A`. `PWM_A` is right as a field of a `const` descriptor.

---

## Types

- **A struct or enum** is typedef'd as `Module_Name_T` over the tag `Module_Name`:
  `typedef struct Stator_Axis { ... } Stator_Axis_T;`.
- **Fields** are PascalCase. A `typedef const struct` descriptor's fields are UPPER_SNAKE, because
  upper case marks what is always `const`.
- **An enum member** is the type name in UPPER_SNAKE, then the member: `Motor_StateId_T` →
  `MOTOR_STATE_ID_OPEN_LOOP`, `PID_Mode_T` → `PID_MODE_PI`.

### Value or object

**A type name's case says how the type is passed and which functions it takes.** What the type
means decides the form, not its size alone. The kinds behind these forms are in
[StructSemantics.md](../Principles/StructSemantics.md).

| Form | Is | Passed | Its functions | Example |
|---|---|---|---|---|
| `snake_case_t` | a value type: a named scalar, a "fat register", at most 64 bits. A record, not a descriptor. No pointers, callbacks, ownership or lifecycle | by value, in and out | snake_case: `module_verb(type_t a, type_t b)` returns `type_t` | `fract16_t`, `angle16_t`, `accumulator_state_t`: `module_type_t`, as in `stdint.h` |
| `struct snake_case`, no typedef | a pure function's multiple return values | returned by value; the caller unpacks it into its own fields at the call | the pure function that returns it | `struct foc_dq`, `struct foc_abc` |
| `Module_Name_T` | an aggregate over 64 bits, or a type that has behavioral functions: runtime state, descriptors, configs, interfaces | by pointer: `Module_Verb(Module_T * p_module)` | `_Init`, `_Capture`, `_Set`, `_Get` ([§ Functions](#functions)) | `PID_T`, `Motor_Config_T` |
| `Module_Name_T` with UPPER_SNAKE fields | a descriptor that is constant at compile time | by pointer | as above | `Encoder_T` |

- **A value type is at most 64 bits.** AAPCS returns a fundamental type of up to 64 bits in
  r0–r1. A struct of plain values within that size may also take the snake_case form, as long as
  it has no pointer-based API and no state to manage: `fract_xy_t`. A struct, though, returns in r0
  only up to 4 bytes; a larger one returns through memory. So passing a struct by value means
  `static inline` ([StructSemantics.md](../Principles/StructSemantics.md)).
- **`struct snake_case` stays in the pure layer.** It is never a field of a `Module_Name_T`, and
  never a parameter or return of a `Module_Verb` function. Known exception: `Angle_UnitVector`
  ([Angle.h](../../Math/Angle/Angle.h)) returns `struct fract16_xy` to `AnglePll_ProcVector`.
- The representations, such as `fract16_t` as Q1.15, are in [FixedPoint.md](FixedPoint.md).

### Role suffixes

The suffix after the type's noun names its role. Suffixes come in two families, matching
[StructSemantics.md](../Principles/StructSemantics.md). Objects have a module that keeps their
invariants. Data objects have no invariants, so every field is writable.

**Objects**

| Suffix | Means | Lifetime | Written by | Example |
|---|---|---|---|---|
| `Module_T` | the primary object, the struct the module's functions take: either a stateful object with behavioral functions, or the `const` descriptor that is the module's full context | the program | its functions; a descriptor's initializer | `PID_T`, `FOC_T`; `Encoder_T`, `Motor_T` |
| `Module_State_T`, `Module_Context_T` | the mutable runtime a descriptor points to | the program | its module | `Encoder_State_T` at `Encoder_T.P_STATE`; `Motor_Context_T` at `Motor_T.P_MOTOR` |
| `Module_Config_T` | persisted parameters that can be stored in NVM | persistent | the user, the protocol, an NVM load | `Motor_Config_T` |
| `Module_Ref_T` | values precomputed from a Config | rebuilt at Init and Reinit | the Init function | `AngleCounter_Ref_T` |
| `Module_Status_T` | an enum: a function's result, or the condition a module reports | one return; or as long as the module | the function; the module | `NvMemory_Status_T`, `Monitor_Status_T` |
| `Module_Flags_T` | a bitfield of status or control bits | as long as the module | the module | `Version_Flags_T`, `Motor_FaultFlags_T` |

Config and Ref are detailed in [ConfigStateRef.md](ConfigStateRef.md). A `switch` dispatches on a
`State_T` node of the state machine, on its id `Module_StateId_T`, or on a mode
([BooleanState.md](BooleanState.md)). Whether `_State_T` may also name that state is open
([§ Not decided](#not-decided)).

**Data objects**

| Suffix | Means | Written by | Read by | Example |
|---|---|---|---|---|
| `Module_Data_T` | plain data with no invariants | its producer | its consumer | `Phase_Data_T` |
| `Module_Input_T` | data another module writes for this one | the other module | this module | `Phase_Input_T`, `Traction_Input_T` |
| `Module_Output_T` | data this module writes for another | this module | the other module | none yet |
| `Module_Packet_T` | a wire format | the wire, or a builder | a parser | none by suffix: the Mot protocol's packets take its module prefix, `MotPacket_Header_T` |

In `Module_Var_<Group>_T`, such as `MotorController_Var_Output_T`, the word is a var group, an enum
of ids, not a data object ([0012](../Decisions/0012_VarEnumNaming.md)).

### Not decided

- **The suffix for runtime state.** `_State_T` names two things today. One is the mutable runtime a
  descriptor points to, in about two dozen types such as `Encoder_State_T`. The other is the state
  a `switch` dispatches on, as in the state machine's `State_T` and the enum `Cia402_State_T`.
  Motor and MotorController use `_Context_T` instead.

  | Word | For it | Against it |
  |---|---|---|
  | State | most of the library uses it today | the collision above |
  | Context | already used: `Motor_Context_T`, `MotorController_Context_T` | |
  | Runtime | mutable at runtime, in contrast with Config | too qualitative |
  | Object | the struct with an invariant interface | |

- **A state within a state:** Stage, as the note proposed, or Substate, as the code has it
  (`StateMachine_Branch_EnterSubstate`, `MotorController_LockSubStateId_T`). The ids are in
  [SubStateIds.md](../Proposals/SubStateIds.md).

### Not taken

| Option | Why not |
|---|---|
| `Module_Calib_T`, parameters persisted in the user's units | user units belong to the app; the device stores values in the form it runs on ([ConfigStateRef.md](ConfigStateRef.md#what-the-device-stores)) |

---

## Variables

- **Locals** are camelCase. Loop and trivial variables are single letters: `i`, `n`, `k`.
- **Pointers** take a `p_` prefix:
  - parameters and locals: `p_motor`;
  - a mutable struct field: p_ActiveSensor;
  - a `const` descriptor field: P_STATE.
- **Parameters** carry `const` only on the pointee (`const FOC_T * p_foc`). A by-value parameter
  takes no `const` and is never reassigned. A quantity written with a subscript in the literature
  may keep it: `i_alpha`, `lq_turn`.
- **Booleans** answer a yes/no question: IsQuadratureCaptureEnabled. Prefer an enum mode to an on/off
  flag.
- **Static duration:**

| Kind | Form | Example |
|---|---|---|
| File-scope mutable | PascalCase, no module prefix | TxPdoTimestamps |
| Extern mutable | `Module_Name` | Cia402_PdoConfig |
| Extern, for the module's own headers | `_Module_Name` | _Critical_InterruptDisableCount |
| Defined at build time: `const` objects, state nodes, tables, service instances | UPPER_SNAKE | `MOTOR_STATE_RUN`, `MOTOR_CONTROLLER_CIA402_SERVICE` |

---

## Functions

| Kind | Form | Example |
|---|---|---|
| Public | `Module_Verb(Module_T * p_module, ...)` | `Motor_SetFeedbackMode` |
| Scoped | `Module_Scope_Verb`, rules in [ApiScoping.md](ApiScoping.md) | `StateMachine_Tree_Input`, `Motor_Config_IsValid` |
| Header-visible helper | `_Module_Verb` | `_FOC_ProcVCircle` |
| File-scope `static` | `Verb`, no module prefix | CalcPI in `PID.c` |
| Pure | `module_verb`, parameters in the published notation | `stator_emf`, `foc_park` |

**A function that changes state or touches hardware starts with its verb.** A query has no side
effect and names what it returns:
- `Get` plus the value, or the value alone: `PID_GetOutput`, `FOC_Vd`, `Motor_RotorSensor`.
- `Of` when the value depends on an argument: `Linear_InvOf`, `Hall_CommutationIdOf`.
- `Is` for a yes/no answer: `AnglePll_IsLocked`, `Accumulator_IsSaturated`. Never `GetIs`.

**A segment before the verb names a group; a segment after it names a variant.**

| Segment | Names | How callers use it | Example | Instead of |
|---|---|---|---|---|
| before the verb: `Module_Group_Verb` | an API group, one use case independent of the module's other groups | take the group's functions as a set, without the others | `StateMachine_Branch_CallInput`, `StateMachine_Branch_ProcSyncInput` | `StateMachine_CallInput_Branch` |
| after the verb: `Module_Verb_Variant` | a variant of one operation, an overload ([§ Units and variants](#units-and-variants)) | call any of the variants, side by side | `Motor_GetSpeed_Rpm`; `Motor_GetSpeedVNominalRef_Pu` beside `_Rpm` | |

A group reads as an object and its method, `branch.CallInput()`. The scope prefixes in
[ApiScoping.md](ApiScoping.md) are one kind of group: the scope is the struct the function takes.

### Suffixes and prefixes for the kind of function

| Form | Means | Example |
|---|---|---|
| `_Thread` | A complete thread of execution, placed in a scheduler | `MotorController_PWM_Thread` |
| `_ISR` | An interrupt handler | `ADC_OnComplete_ISR` |
| `On` plus an event | Called when the event occurs | `ADC_OnComplete_ISR` |
| `HAL_` | Hardware abstraction layer | `HAL_Pin_ToggleOutput` |
| Platform name | A platform function outside the HAL interface, called by the Board; becomes `HAL_` when a Peripheral module needs it ([Notes.md](../../Peripheral/Notes.md)) | `S32_PDB_ConfigChannel`, `KE0x_ADC_InitHwTriggerFtm2` |

### Verbs on the module's own data

| Verb | Means | Example |
|---|---|---|
| Get | Read a value; no change | `PID_GetOutput` |
| Set | Write a value from the caller, keeping the struct's invariants | `PID_SetOutputLimits` |
| Capture | Store a value as it becomes available, for later processing: data produced on another clock, for the next step / proc | `Stator_Axis_CaptureI`, `FOC_CaptureIData` |
| Init | Set every field; no allocation. InitFrom takes a config, Init plus a part sets one part | `Motor_Init`, `PID_InitFrom`, `AnglePll_InitPid` |
| Reset | Clear runtime state; keep the config | `PID_Reset`, `AnglePll_Reset` |
| Zero | Set one quantity to zero | `Angle_ZeroAngle` |
| Proc | The module's regular update | `PID_ProcPI`, `AnglePll_Proc` |
| Poll | A Proc that may do nothing; returns whether it did | `TimerT_Periodic_Poll`, `Hysteresis_Poll` |
| Step | Advance an equation by one tick | `Accumulator_Step` |
| Start, Stop | Begin or end what a Proc or Poll runs | `Motor_FOC_StartOpenLoop`, `Blinky_Stop` |
| Resolve | Derive further state from known state, usually from several sources; the result may be stored for later processing | `Angle_ResolveSpeed_Pu`, `Motor_ResolveILimits`, `Motor_FOC_ResolveISupplyLimits` |
| Validate, IsValid | Bring a config into range; check it | `Motor_Config_Validate`, `Motor_Config_IsValid` |

**Resolve or Capture.** Both can store state that later processing needs, so either can fit. Capture
says the value is taken as it becomes available: `FOC_CaptureIData` takes the I set at its completion.
Resolve says the value is derived from state already held: `Motor_FOC_ResolveISupplyLimits` derives
ISupplyLimits from the FOC operating point and VBus on whichever thread the build selects, and stores
it for `Motor_ResolveILimits`. Name the one the call depends on: the arrival, or the derivation.

### Verbs on hardware or external data

| Verb | Means | Example |
|---|---|---|
| Read | Input from a register or external data | `FastPin_Input_ReadPhysical` |
| Write | Output to a register or external data | `Phase_WriteSvpwm`, `Flash_WriteVirtual` |
| Activate | Start a physical action | `Phase_ActivateV0`, `ADC_ActivateMarked` |
| Enable, Disable | Turn a function on or off; no argument | `Encoder_EnableQuadratureMode` |
| Toggle | Invert the current state | `HAL_Pin_ToggleOutput` |

A descriptive verb is better where it says more: Send, Commutate, Enqueue, Seed, Integrate.

**No boolean arguments that hide the operation.** `SetEnable(bool)` could assign the state or OR it
in; `SetToggle(bool)` could set the toggle or request one. Use Enable and Disable without an
argument, or an enum mode: `Motor_SetFeedbackMode`. A `bool` parameter fits only a model that is
itself binary, such as a switch or a binary sensor.

---

## Units and variants

**A `_Unit` suffix names the unit or base; the type names the representation.** IAlign_Pu is a
`fract16_t` per-unit current. Other units: AlignTime_Cycles, Heat_Adcu, SpeedMax_Rpm. See
[FixedPoint.md](FixedPoint.md).

**A function suffix after `_` names a variant of the same operation:**
- unit or representation: `Angle_ResolveSpeed_Pu`, `PID_SetKp_Fixed16`;
- behavior: `Flash_WriteOnce_Blocking`.

**The pure layer** spells the same thing in snake_case: `g_l_pu_turn`. A conversion reads as `y_of_x`:
`l_pu_of_angle_freq`.

**Size and count words:**

| Word | Means |
|---|---|
| Size | Bytes, as `sizeof` |
| Length | Number of elements |
| Count | How many so far, or now |
| Max | The upper bound of a value, e.g. `UINT16_MAX` |

---

## Word order

**General to specific.** The noun comes first, then its qualifiers, so related names sort and align
together: OutputMin and OutputMax, LimitUpper and LimitLower, ErrorPrev, LockHoldCount.

**A compound with routines of its own stays whole.** SpeedLimitForward_Pu keeps SpeedLimit together
because SpeedLimit has its own routines (`Motor_SetSpeedLimits`). LimitSpeedForward would make it one
of many Limits.

**Types** follow the same order: module, noun, qualifier: `Motor_FeedbackMode_T`,
`AnglePll_FilterTuning_T`.

**Functions** put the verb first, then the noun, then its qualifier: `Motor_SetILimitMotoring`,
`PID_SetOutputLimits`. Without `.` chaining, the descriptive noun comes before its adjective: a C
`GetRectangleBlue(p_this)` for an object-oriented `getBlueRectangle().Area()`. Two leading verbs are
two actions in sequence: `Hall_PollCaptureSensors` polls, then captures.

---

## Build options

**A macro is a build option only if it changes what compiles.** Options are named
`MODULE_FEATURE_ENABLE` / `MODULE_FEATURE_DISABLE`, or `MODULE_CHOICE_VARIANT` for a choice among
variants: `MOTOR_FOC_FIELD_WEAKENING_ENABLE`, `MOTOR_V_MATCH_SENSOR`.

**The module's main header names every option and defines its default** when the build sets none, as
`Motor/Motor/Motor.h` does. An overridable constant uses `#ifndef` and keeps its own name:
`MOTOR_CALIBRATION_BUFFER_SIZE`.

---

## Module directory

A folder is a module, and its public header takes the folder's name: `Motor.h`, `StateMachine.h`.
A leading `.` marks a folder or file outside the library proper ([LeadingDot.md](LeadingDot.md)). A
leading `_` marks a file private to its module: `_StateMachine_Tree.h`.

What sits under a module relates to it in one of four ways:

| Relation | Depends | Is | Instance |
|---|---|---|---|
| Subtype | the child on the parent | the child IS-A parent: a variant, a specialization | `Motor/Motor/Sensor/Hall/` of `RotorSensor` |
| Component | the parent on the child | the child is part of the parent | `Motor/Motor/VBus/` of `Motor` |
| Part | either way: inner or outer | a related group of the module's own functions, in a `Module_Part` file | `Motor_Electrical.h`, `Motor_Var.h` |
| Extension | neither: the module works without it | an optional feature | `Framework/StateMachine/Extension/` |

- **A part is either inner or outer.** The core includes an inner part and depends on it: `Motor.h`
  includes `Motor_Electrical.h`. An outer part includes the core, and the core does not need to know
  about it: `Motor_Var.h` includes `Motor.h`.
- **An extension is opt-in.** The core neither includes it nor calls it. A build opts in, and a
  guard, if any, goes on the call only ([ConditionalCompilation.md](ConditionalCompilation.md)):
  `_StateMachine_FaultState.c` under `STATE_MACHINE_FAULT_STATE_ENABLE`.

### Not decided: the plain folder

A plain folder name can carry only one relation, so the other one needs a marker. Under both
options, `Extension/` marks an extension and the file name marks a part.

| | A: subtypes unmarked | B: components unmarked |
|---|---|---|
| `SomeModule.h` | the public API, and the base | the public API |
| `Name/` | a subtype: a specialization of this module | a component: a dependency of this module |
| `_Name/` | a private component | not used |
| `Base/` | not used | the base of a module that has subtypes, each subtype a folder beside it |
| `Extension/` | opt-in features | opt-in features |
| `SomeModule_Part.h` | a part | a part |
| In the tree today | `Motor/Motor/Sensor/`: `RotorSensor.h` at the root, one folder per sensor | `Motor/Motor/`: `Phase/`, `VBus/`, `Sensor/`. `Transducer/Monitor/`: `Base/`, `Heat/`, `Voltage/` |

`Peripheral/NvMemory/` follows neither option. Its base sits in `NvMemory/NvMemory/`, beside
`EEPROM/`, `Flash/` and `FlashOnce/`. No `_Name/` folder exists yet.
