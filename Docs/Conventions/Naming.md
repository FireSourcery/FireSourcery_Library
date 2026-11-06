# Naming

How names are built. An underscore separates scope segments, and case separates the words inside a
segment, so `Motor_FOC_ProcAngleV` reads Motor › FOC › ProcAngleV. Upper case gives a second
delimiter, which leaves the underscore free to mark scope.

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
- **A value typedef** is snake_case with `_t`, as `stdint.h` does: `fract16_t`, `angle16_t`,
  `accumulator_state_t` (`module_type_t`).
- **An untypedef'd `struct snake_case`** is only a pure function's multiple return, unpacked at the
  call: `struct foc_dq`. It never becomes a field, parameter or return in the stateful layer.
- **Role suffixes** (`_Config_T`, `_State_T`, `_Ref_T`) are in [Config_State_Ref.md](Config_State_Ref.md).
-
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
[FixedPoint.md](../../Motor/Docs/FixedPoint.md).

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


todo

## Types


Module_T             — Primary stateful object (has behavioral functions) OR Full descriptor context
Module_State_T or Module_Context_T     — mutable runtime.
     naming collision with State_T. alternatively:
        runtime - runtime mutable, contract with config, too qualitative
        object - has invariant interface

Module_State_T       — switch() to determine processing. either as StateMachine module State_T or enum id
    Stage - Substate

Module_Status_T      — Enum return
Module_Flags_T       — Bitfield status/control
Module_Config_T      — Persistent parameters (NVM-storable)

Module_Calib_T       — User-unit parameters
Module_Ref_T         — Precomputed/derived from Config

alternatively for data objects:
Module_Data_T        — Pure passive data (no invariants)
Module_Input_T       — Data written by external, read by module
Module_Output_T      — Data written by module, read by external
Module_Packet_T      — Wire/serialization format

Suffix	Meaning	Lifetime	Who writes
_Ref	Derived/precomputed	Computed at init	Init function
_Data suffix = no behavioral invariants, any field writeable.
// "_Data" suffix — signals "this is a passive data container, not a stateful object"
// "_Input" suffix — this module reads from it, another module writes to it
// "_Output" suffix — this module writes to it, another module reads from it
<!-- _Calib	User-facing units	Persistent (NVM)	User/host tool -->

Function Prefixes by API group, use case independent from other group.
StateMachine_Branch_ProcInput() rather than StateMachine_ProcInput_Branch
Function Suffixes by overload, concurrent use.
GetSpeed_Rpm

Naming by what it is not qualifiers.
organizes by concept axis, consistent with OOP

Semantic names:
- Asymmetric operations get asymmetric names: what triggers each, whether it answers a request, what goes in and comes out. Not a direction pair.
  Tx/Rx and Producer/Consumer were rejected for the CAN handlers (2026-09-27): the Rx handler is request → optional response on a peer's frame, the Tx builder is node state → frame on the node's own clock, unsolicited.
  Check the library's vocabulary first (`Protocol_Req_T`, `Protocol_ProcReqResp_T`), then standard terms (J1939-21 Request / Command / Broadcast-Response; OSI confirmed vs unconfirmed services).
- A quantity is named by what it physically is: `_pu_turn`, `_pu_rad`, the angle per Ts at ω_pu = 1.0 (2026-10-15).
  Rejected: `_pu_nyquist`, a qualifier; `_pu_tick`, which names time for an angle base.


typedef int16_t fract16_t;      /*!< Q1.15 [-1, 1) */
typedef uint16_t ufract16_t;    /*!< Q1.15 [0, 2) */
typedef int32_t accum32_t;      /*!< Q17.15 2*[INT16_MIN:INT16_MAX] */

    `snake_case_t`: Value types, ≤ 64 bits (ARM r0+r1), no behavioral functions.
        - Returned and passed by value.
        - Conceptually a "fat register" or "named scalar".
        - Functions use snake_case: module_verb(type_t a, type_t b) -> type_t
        - No pointers, callbacks, ownership, or lifecycle.
        - Records, not descriptors

    `struct snake_case`, no typedef: a pure function's multiple returns, e.g. `struct foc_dq`, `struct sta_axis`.
        - Returned by value from a snake_case function, unpacked into the caller's fields at the call.
        - Never a field of a `CamelCase_T`, never a parameter or return of a `Module_Verb` function.
        - Known exception: `Angle_UnitVector` (Angle.h) returns `struct fract16_xy` to `AnglePll_ProcVector`.

    `CamelCase_T`: Aggregate types, > 64 bits, OR has associated behavioral functions.
        - Passed by pointer: Module_Verb(Module_T * p_module)
        - May have _Init, _Capture, _Set, _Get functions.
        - Used for runtime state, descriptors, config objects, interfaces.

    `CamelCast_T.CAPS`: compile time constant descriptors.

    Boundary: 64 bits hard limit (ARM AAPCS returns r0+r1).
        Structs ≤ 64 bits MAY use snake_case IF they are pure values
        with no pointer-based API and no associated state management.
        Size alone does not decide naming; semantics do.



## Module Directory

Subtypes (child depends on parent, child IS-A parent)
Components (parent depends on child, child is part-of parent)
Parts (Related group, outer or inner)
Extensions (optional, guarded by #ifdef)

Subtypes as the unmarked case.

    SomeModule/
    ├── SomeModule.h            # Module public API
    ├── [_Internal]/            # _ = private component, dependency OF this module
    ├── [Variant]/              # plain = specialization/subtype OF this module
    └── Extension/              # opt-in features, #ifdef guarded
    ├── SomeModule_Part         # Part, dependent or independent of SomeModule core
        Dependent Part -> Include by core
        Independent Part -> Core does not need to know

    alternatively Components as the unmarked case.
    ├── SomeModule.h            # Module public API
    ├── [Component]/            # private component, dependency OF this module
    └── Extension/              # opt-in features, #ifdef guarded
    ├── SomeModule_Part         # Part, dependent or independent of SomeModule core

    Subtyped use Base
    SomeModule/
    ├── Base/
    ├── [Variant]/            # plain = specialization/subtype OF this module
    └── Extension/            # opt-in features, #ifdef guarded






