# Struct Semantics

**Statement** — A struct defines the shape of memory; the module around it defines what the struct
means. Each struct has a kind, set by its purpose and access pattern: who writes it, when, where it
lives and how it is passed. The kind, not the shape, decides its storage, its passing form and its
API. Two structs of one shape in different kinds are different types.

**Rationale** — C has one aggregate, and the compiler checks only its layout. A descriptor folded
from FLASH, a config the user writes, coefficients derived at init, state stepped every tick and a
packet whose offsets are the wire format are all `struct`, and any access to any of them compiles.
The kind carries the contract the type system cannot: which fields change, who writes them, how
often, and whether the layout itself is the interface. Named, the kind tells the reader that
contract from the type, and each kind takes the storage and passing form that suit it.

**Implications**

| Kind | What it holds | Written by | Lives in, passed | In code | Rules in |
|---|---|---|---|---|---|
| Compile-time context | A generic module's constants paired with its state, folded into each inlined call: static polymorphism | initializer, `const` | FLASH, pointer | `TimerT_T` | [DescriptorTemplate.md](../Patterns/DescriptorTemplate.md), [Descriptor.md](../Patterns/Descriptor.md) |
| HAL descriptor | A peripheral instance: register base and constants, mixed into each register access | initializer, `const` | FLASH, pointer | `PWM_T` | [DescriptorPattern.md](../Patterns/DescriptorPattern.md) |
| Runtime configuration | Parameters selectable at runtime, persisted, read back as written | user, protocol, NVM load | NVM, working copy in RAM; pointer | `MotorController_Config_T` | [Config_State_Ref.md](../Conventions/Config_State_Ref.md) Option 1, [ConfigValidity.md](../Conventions/ConfigValidity.md) |
| Derived (Ref) | A runtime optimization: coefficients precomputed from config for the step | Init and Reinit only | RAM; pointer to `const`, or by value when built | `Angle_SpeedPuRef_T`, `Linear_T` | [Config_State_Ref.md](../Conventions/Config_State_Ref.md) Option 2, [EquationStruct.md](../Patterns/EquationStruct.md) § Coefficients |
| Runtime model | State with invariants, stepped by its module; the default kind | its module's functions | RAM, pointer | `Motor_Context_T`, `Angle_T` | [EquationStruct.md](../Patterns/EquationStruct.md), [DescriptorPattern.md](../Patterns/DescriptorPattern.md) |
| Value | Almost a primitive: a pure function's multiple returns | a pure function, returning | registers, or unpacked into the caller's fields; by value | `struct foc_abc` | [Naming.md](../Conventions/Naming.md) § Types |
| Data model | Transparent data handed from a producer to a consumer | the producer | RAM, or by value | `Phase_Data_T` | [0003_PhaseOutputOwnership.md](../Decisions/0003_PhaseOutputOwnership.md) |
| Packet | A wire format: each field's offset and size is the contract, and the shape encodes metadata (`sizeof` is a length) | the wire, or a builder | a buffer, pointer; `MOT_PACKET_PACKED` | `MotPacket_Header_T`, `MotPacket_Var16WriteReq_T` | none yet |

- **The last three are data transfer: the data is the interface.** No invariants, every field
  public, no function guards a write. A value crosses a function return, a data model crosses
  modules, a packet crosses the wire.
- **By value means `static inline`.** Out of line, AAPCS passes up to 16 bytes of struct in r0–r3
  but returns a struct over 4 bytes through memory, at an address the caller passes in r0.
  `struct foc_abc` (12 bytes) and `Phase_Data_T` (8 bytes) stay in registers only when inlined.
  Checked with `arm-none-eabi-gcc` 15.2.1, `-mcpu=cortex-m0plus -O2`, 2026-10-31.
- **Same shape, different kind:** an argument bundle that is never persisted is a Ref, not a Config
  (`AngleCounter_Ref_T`); the Config holds the published coefficients and the runtime the derived
  ones (Kp, Ki, Kd in the Config, Kp and KiPerTick in `PID_T`).
- **A second place is not a second kind.** The Config's NVM image and its RAM working copy are one
  type in two places. A second type is due when the operations differ: persisted and written by the
  user, or rebuilt at Init and never persisted. Simonyi's test: if the same operations apply, the
  types are the same.
- **What a second type buys.** Distinct tags are distinct types (C11 §6.7.2.3p5), so the compiler
  rejects a Ref where a Config is expected. The write rules of a kind (Init only, written by the
  user, stepped by its module) it never sees: they hold only inside the owning module. For the data
  transfer kinds a second type prevents mix-ups; it guards no invariant.
- **The converse: one meaning, several shapes.** A Config field and the `{MotVarId, Value16}` pair
  that writes it over the wire are two types joined by a conversion, `MotorController_WriteVar16s`,
  not one type in two places.
- Names carry the kind where a suffix exists: [Naming.md](../Conventions/Naming.md) § Types.
- Open: whether runtime configuration holds user units or internal units. Both occur today:
  `VBus_Config_T.VSupplyNominal_V` is in volts, the `Motor_Config_T` limits are `_Pu`. Prior art:
  AUTOSAR and ASAM A2L store the representation the code consumes and carry the unit as metadata
  (CompuMethod, `COMPU_METHOD`).

**Source**

| For | Source |
|---|---|
| Shape and meaning apart, the meaning owned by one module | Parnas, "On the Criteria To Be Used in Decomposing Systems into Modules", *CACM* 15(12), 1972; Morris, "Protection in Programming Languages", *CACM* 16(1), 1973, §5; Cardelli and Wegner, "On Understanding Types, Data Abstraction, and Polymorphism", *Computing Surveys* 17(4), 1985, §1.1–1.2 |
| Same shape, different type | Ada 83 LRM §3.3: two type definitions are distinct "even if they are textually identical"; sparse `__bitwise` (Torvalds, linux-kernel, 2004) |
| One meaning, several shapes | Ada 83 LRM §13.6: packed and unpacked descriptors, two types joined by a conversion |
| A name alone guards no invariant | King, "Names are not type safety", 2020: a newtype is safe only inside its home module |
| The kind in the type name | Simonyi, "Hungarian Notation", MSDN, 1999 reprint: a structure's operations are independent of the fields that set its representation, and each structure takes its own tag; Spolsky, "Making Wrong Code Look Wrong", 2005, for the word *kind* |
| The kind on the instance, not the type | AUTOSAR Classic R22-11: structure fixed on the type, implementation policy set per prototype (TPS_SWCT_01273); memory section by data kind (SWS_MemoryMapping §7.2.1.2); configuration passed to `<Mip>_Init()` by const pointer (SWS_BSW_00160) |
| Runtime model, value | Entity and Value Object (Evans, *Domain-Driven Design*, 2003) |
| Data transfer kinds | Data Transfer Object (Fowler, *Patterns of Enterprise Application Architecture*, 2002) |
| Data-only interfaces | Temporal firewalls (Kopetz and Nossal, FTDCS 1997); state and event information (Kopetz, "Time-Triggered Real-Time Computing", IFAC 2002, §3.3) |
| Result return | *Procedure Call Standard for the Arm Architecture* (Arm IHI 0042) |

Not published, as far as found: the kind table itself, and binding the kind to the type. The
standards that classify data by writer and binding time (AUTOSAR, A2L, IEC 61131-3) attach the
kind to the instance.
