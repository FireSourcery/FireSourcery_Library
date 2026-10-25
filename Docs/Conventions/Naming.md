
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






