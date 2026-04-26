



_Data or _Payload suffix = no behavioral invariants, any field writeable.
// "_Data" suffix — signals "this is a passive data container, not a stateful object"
// "_Input" suffix — this module reads from it, another module writes to it
// "_Output" suffix — this module writes to it, another module reads from it


Module_T             — Primary stateful object (has behavioral functions)

Module_State_T       — mutable runtime.
     naming collision with State_T. alternatively:
        runtime - runtime mutable, contract with config, too qualitive
        object - has invariant interface

Module_State_T       — switch() to determine processing. either as StateMachine module State_T or enum id
Module_Status_T      — Enum return
Module_Flags_T       — Bitfield status/control
Module_Config_T      — Persistent parameters (NVM-storable)

Module_Calib_T       — User-unit parameters
Module_Ref_T         — Precomputed/derived from Config

Module_Data_T        — Pure passive data (no invariants)
Module_Input_T       — Data written by external, read by module
Module_Output_T      — Data written by module, read by external
Module_Packet_T      — Wire/serialization format



Suffix	Meaning	Lifetime	Who writes
_Calib	User-facing units	Persistent (NVM)	User/host tool
_Config	Runtime units	Persistent or init-time	Init function
_Ref	Derived/precomputed	Computed at init	Init function
_State or none	Runtime mutable	Volatile	Runtime functions


    FLASH (const):
    ┌─────────────────────────────┐
    │ Hall_T                      │
    │   PIN_A: { P_HAL, ID, ... }│
    │   PIN_B: { P_HAL, ID, ... }│
    │   PIN_C: { P_HAL, ID, ... }│
    │   P_STATE ──────────────────┼──► RAM
    │   P_NVM_CONFIG ─────────────┼──► NVM/FLASH (another section)
    └─────────────────────────────┘

    RAM (mutable):
    ┌─────────────────────────────┐
    │ Hall_State_T                │
    │   Config: { SensorsTable }  │  ◄── copied from NVM at init
    │   Sensors: 0b101            │  ◄── written by ISR
    │   SensorsPrev: 0b001        │
    │   Direction: CCW            │
    │   Angle: 21845              │
    └─────────────────────────────┘

    NVM (persistent):
    ┌─────────────────────────────┐
    │ Hall_Config_T               │
    │   SensorsTable[8]           │  ◄── calibration data
    └─────────────────────────────┘


typedef int16_t fract16_t;      /*!< Q1.15 [-1, 1) */
typedef uint16_t ufract16_t;    /*!< Q1.15 [0, 2) */
typedef int32_t accum32_t;      /*!< Q17.15 2*[INT16_MIN:INT16_MAX] */

    snake_case_t: Value types, ≤ 64 bits (ARM r0+r1), no behavioral functions.
        - Returned and passed by value.
        - Conceptually a "fat register" or "named scalar".
        - Functions use snake_case: module_verb(type_t a, type_t b) -> type_t
        - No pointers, callbacks, ownership, or lifecycle.

    CamelCase_T: Aggregate types, > 64 bits, OR has associated behavioral functions.
        - Passed by pointer: Module_Verb(Module_T * p_module)
        - May have _Init, _Capture, _Set, _Get functions.
        - Used for runtime state, descriptors, config objects, interfaces.

    Boundary: 64 bits hard limit (ARM AAPCS returns r0+r1).
        Structs ≤ 64 bits MAY use snake_case IF they are pure values
        with no pointer-based API and no associated state management.
        Size alone does not decide naming; semantics do.


##

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
    ├── SomeModule_Part         # Part, dependent or independt of SomeModule core
        Dependent Part -> Include by core
        Independent Part -> Core does not need to know

    alternatively Components as the unmarked case.
    ├── SomeModule.h            # Module public API
    ├── [Component]/            # private component, dependency OF this module
    └── Extension/              # opt-in features, #ifdef guarded
    ├── SomeModule_Part         # Part, dependent or independt of SomeModule core

    Subtyped use Base
    SomeModule/
    ├── Base/
    ├── [Variant]/            # plain = specialization/subtype OF this module
    └── Extension/            # opt-in features, #ifdef guarded



##

Multiple API Access Points — Architecture Principle
The principle: signatures as capability contracts
    e.g. Traction_Input_T

every module interface should expose the minimum information a client needs. Narrow parameter types are the function-level expression of this.
Interface Segregation Principle: "clients should not be forced to depend on methods they do not use." The function-signature analogue: don't force callers to supply state the function doesn't read.
prefer narrower parameter types to reduce physical dependencies. A function taking Config_T * can live in a lower-tier header than one taking Outer_T *; the compiler enforces the layering.

Leading-underscore for "inner" (current pattern at line 129): _Motor_DegOfRpm
    Suffix-by-struct: when naming collisions likely
    A function's parameter type is the narrowest struct that contains every field it reads or writes. Name the function Module_Scope_Verb() where Scope identifies that struct when multiple scopes share the module namespace.
    exempt for main structs (Config, State, etc.)

A public function's scope prefix is determined by the widest struct its effects reach, not the struct it takes as a parameter.

##

The Subject_Collaborator_* form matches how Linux names cross-subsystem calls (netdev_master_upper_dev_link, skb_copy_datagram_iter): primary subject first, collaborators follow, verb last.
When coupling two peers:
0 coordination points → no wrapper, no bridge. Callers compose directly.
≤ ~5 coordination points, peers are public vocabulary → Multi-context free functions (Subject_Collaborator_*). No wrapper type.
Many coordination points, or composite owns meaningful state → Mediator wrapper with peer getters (Option A).
Peers must be hidden from callers → Full-forwarder facade (Option B).


## Data Type by semantics

struct defines the shape of memory
add sematics: "types" of structs by purpose and access patterns

static polymorism context / compile time configuration constants
    TimerT,
HAL wrapper - configuration mix in to register access
    PWM_T

Runtime Configuration - runtime selectable configuration. mostly transparent data - in user domain units or internal domain units?
    MotorController_Config_T

Runtime optimization - derived
    Angle_SpeedFractRef_T
    Linear_T

General runtime model - most common
    Motor_State_T
    Angle_T

Data Transfer Interface - data is the interface

    almost primitive - 2 register or less pass by value
        foc_abc

    Transparent Data / Data Model
        Phase_Data_T

    Packets - also transparent data
        meta data encoded in struct shape.
        placement offset, size, matters

    struc Tag only for internal?

Data - Tranparent without invarant
Obj - with interface


#
2 pointer context API signitures at most - (p_const, p_state, value)
Does the wrapper hold a changeable policy?
Does the wrapper enforce a pairing invariant the bare form can violate?
Does the wrapper add a cross-cutting decorator like locking/tracing?
Is every pointer in the wrapper used at runtime on the hot path? (lifetime-coherent)


#
ownership vs pointer reference determined by thread of operation



#
Adding indirection without adding concept vs enforcing pair correctness

