# Struct Semantics Prior Art

The shape/meaning split behind [StructSemantics.md](../Principles/StructSemantics.md) and its same-shape, distinct-type rule are 1972–1989 prior art; the kind table and binding the kind to the type have no stated precedent in the sources reviewed.

**Question** — What have prior publications said about a struct as the shape of memory, with its kind as the meaning wrapped on top, and which parts of the principle are new?

**Method** — Source reads on 2026-11-22, in four passes: type theory, layout mechanisms in languages
and tools, design practice, embedded and automotive standards. Each quote was checked against the
primary text where one could be obtained; the Check column records how. AUTOSAR is Classic Platform
R22-11; its page numbers are the PDFs' own footers and may be off by one, the requirement IDs are
the stable reference. The page with the figures (clause marking, timeline, one shape under four
kinds, kind on the type against kind on the role) is
[2026-11-22_StructSemanticsPriorArt.html](2026-11-22_StructSemanticsPriorArt.html).

| Check | Meaning |
|---|---|
| P | matched against the primary text |
| F | through a web-fetch summary, probably exact, not checked against raw text |
| S | secondary source only |
| U | not read, or metadata only |

**Result**

| Clause | Verdict | Strongest precedents | New in the principle |
|---|---|---|---|
| The struct is the shape of memory; meaning comes from outside | Anticipated almost verbatim | Cardelli & Wegner 1985 §1.1–1.2; C11 §6.2.5p1; Simonyi 1999; AUTOSAR TPS_SWCT_01229–01236; Kopetz 2002 §3.4 | none |
| The module around it defines the meaning | Anticipated | Parnas 1972 p. 1056; Morris 1973 §5; King 2020; Hoare 1972 (U) | Three kinds keep the shape public; the classic line hides it |
| One shape in different kinds gives different types | Anticipated as a type rule | Morris §5; Ada 83 §3.3; Modula-3 brands; Breitner et al. 2014; sparse; C++ Core Guidelines I.4 | The criterion for declaring the second type |
| Kind set by who writes it, when, where it lives, how it is passed | Axes found in standards, attached to instances | AUTOSAR prototypes and ECUC classes; Kopetz 2002 §3.3; Twelve-Factor; C `volatile` | The four axes as the definition of a kind |
| The kind decides storage, passing form and API | Anticipated by AUTOSAR, on the role | MemMap §7.2.1.2; SWS_BSW_00160; SWS_RTE §5.6; TPS_SWCT_01273 | Binding the kind to the type; all three under one kind |
| Names carry the kind | Anticipated | Simonyi 1999; Spolsky 2005; Stroustrup 2003; kernel coding style §5 | none |

Key quotes, by clause:

| Clause | Source | Where | Says | Check |
|---|---|---|---|---|
| Shape, meaning | Cardelli & Wegner, *Computing Surveys* 17(4), 1985 | §1.1, reprint p. 3 | "The meaning of a piece of memory is critically determined by an external interpretation of its contents." | P |
| Shape, meaning | Cardelli & Wegner 1985 | §1.2, p. 3 | a type is "a set of clothes (or a suit of armor) that protects an underlying untyped representation from arbitrary or unintended use" | P |
| Shape, meaning | ISO C11, N1570 | §6.2.5p1 | "The meaning of a value stored in an object or returned by a function is determined by the type of the expression used to access it" | P |
| Shape, meaning | Simonyi, "Hungarian Notation", MSDN reprint 1999 | Naming Rules | "the invariant property of data structure, the set of operations in which the constructions participate, seems to be largely independent of the fields of the structure that determine only the representation" | P |
| Shape, meaning | Parnas, *CACM* 15(12), 1972 | p. 1056 | "A data structure, its internal linkings, accessing procedures and modifying procedures are part of a single module." | P |
| Shape, meaning | AUTOSAR TPS SWCT | TPS_SWCT_01230, 01236 | application types "do not become visible as data types in the code"; an ImplementationDataType "finally boils down to a typedef" | P |
| Shape, meaning | Kopetz, IFAC 2002 | §3.4 | conceptual mismatches "cannot be resolved by connection systems but require human intervention" | P |
| Shape public | PADS 2005; DataScript 2002; EverParse3D 2022 | §1.1 (EverParse3D) | layout described "as it is"; "no attempt is made to hide such details"; the C cast of a byte array to a packed struct | P |
| Same shape | Morris, *CACM* 16(1), 1973 | §5, p. 17 | "would regard as a type violation anything assuming the latter" (pairs used as intervals) | P |
| Same shape | Ada 83 LRM | §3.3 | "Two type definitions always define two distinct types, even if they are textually identical" | P |
| Same shape | Modula-3 reference, DEC SRC | branded types | "Brands distinguish types that would otherwise be the same; they have no other semantic effect" | P |
| Same shape | Breitner, Eisenberg, Peyton Jones & Weirich, ICFP 2014 | pp. 4–5 | "this distinction cannot be made by the compiler; it can only be specified by the programmer" | P |
| Same shape | Torvalds, sparse posts, 2004 | yarchive | "Each typedef creates a type of it's own, and will thus create a bitwise type that is not compatible with anything else" | P |
| Same shape, criterion | Simonyi 1999 | Type Calculus | "could the same set of operations be meaningfully applied to the quantities in questions? If so, the types are thought to be the same" | P |
| Same shape, limit | King, "Names are not type safety", 2020 | — | "names are not type safety"; the home module is "a trust boundary where internal invariants are enforced by restricting clients to a safe API" | P |
| Converse | Ada 83 LRM | §13.6 | "two different types with identical characteristics, apart from their representation", joined by conversion | P |
| Counter | BARR-C:2018 | §5.1, p. 33 | "this same type could then be used to create a shadow copy of the timer registers" | P |
| Counter | Hickey, 2011 | Simple Made Easy | "Don't feel like I have to write a class now because I have a new piece of information." | S |
| Access pattern | Cardelli & Wegner 1985 | §1.1 | types "categorize objects according to their usage and behavior" | P |
| Access pattern | AUTOSAR TPS SWCT | TPS_SWCT_01266 | VariableDataPrototypes "are changed by the application software at runtime"; ParameterDataPrototypes "are constant (except for calibration access)" | P |
| Access pattern | AUTOSAR TPS ECUC | Table 2.15; TPS_ECUC_02056–02058 | PreCompile, Link, PostBuild, PublishedInformation; derived parameters take their sources' class | P |
| Access pattern | Kopetz, IFAC 2002 | §3.3 | state: "an update-in-place and a non-consumable read"; event: "queued on sending and consumed on reading" | P |
| Access pattern | ASAM MCD-2 MC wiki | — | CHARACTERISTIC "a tunable parameter"; MEASUREMENT "a recordable, ECU-internal variable" | F |
| Storage | AUTOSAR SWS MemoryMapping | §7.2.1.2, p. 31 | "two different kinds of constants": CONFIG_DATA for configurable behaviour, CONST for fixed values | P |
| Storage | AUTOSAR TPS SWCT | Table 5.46, p. 347 | swImplPolicy `const` "can be applied for parameters (not for those in NVRAM)" | P |
| Passing form | AUTOSAR SWS BSWGeneral | SWS_BSW_00160 | configuration passed to `<Mip>_Init()` "via a const-qualified function parameter" | P |
| API | AUTOSAR SWS RTE | §5.6.4–5.6.17 | `Rte_Write` for data semantics, `Rte_Send` for event semantics; `Rte_Prm`, `Rte_CData`: "Access is read-only" | P |
| On the role | AUTOSAR TPS SWCT | TPS_SWCT_01273, pp. 346–347 | "the properties covering the structure shall not be redefined on DataPrototype. Implementation policy, semantics and code generation policy may be changed" | P |
| Passing at the type | Diatchki & Jones, Haskell Workshop 2006 | §2.1 | a memory area "cannot be used directly as a function argument or result, and must instead be identified by a reference or a pointer type" | P |
| Passing at the type | Framework Design Guidelines (Cwalina & Abrams 2008) | struct vs class | a struct "logically represents a single value", "has an instance size under 16 bytes", "is immutable" | P |
| Naming | Spolsky, "Making Wrong Code Look Wrong", 2005 | — | "I'm using the word kind on purpose, there, because Simonyi mistakenly used the word type in his paper" | P |
| Naming | Simonyi 1999 | Naming Rules | "I recommend the use of a new tag for every new data structure" | P |

AUTOSAR attaches the kind to the prototype and the role it is aggregated in, and keeps one
ImplementationDataType across kinds: "different kind, different role". Alignment of the kinds, by
this finding, not stated by AUTOSAR:

| Kind | Nearest AUTOSAR form |
|---|---|
| Compile-time context, HAL descriptor | ParameterDataPrototype in constantMemory, swImplPolicy `const` or `fixed`, MemMap CONST; module constants as CONFIG_DATA_PREBUILD passed by const pointer to `<Mip>_Init()` |
| Runtime configuration | NvBlockDescriptor romBlock and ramBlock, or CALIB with swCalibrationAccess `readWrite`; never swImplPolicy `const` |
| Derived (Ref) | ECUC derived configuration parameters; VAR_INIT or CLEARED when built by an initialization routine |
| Runtime model | staticMemory or arTypedPerInstanceMemory via `Rte_Pim`; MemMap VAR |
| Value | ArgumentDataPrototype |
| Data model | SenderReceiverInterface dataElement or inter-runnable variable, swImplPolicy `standard` |
| Packet | ImplementationDataType mapped to a SystemSignal through COM |

No stated precedent was found for: the kind table, spanning opaque state and transparent data; the
four axes as the definition of a kind; the kind as a type property in plain C, with no generator or
checker; one kind fixing storage, passing form and API together; by value meaning `static inline`
under AAPCS.

Not verified, not to be cited as read: Hoare 1972; Liskov & Zilles 1974 (S); Morris, "Types Are Not
Sets", POPL 1973; Reynolds 1983; TAPL §19.3; Mealy and Balzer 1967; Kennedy 1997, House 1983, Siff
et al. 1999, Chandra & Reps 1999 (metadata only); Grenning (S); White, *Making Embedded Systems*;
Evans's Specification, Fowler's Quantity, Refactoring's Primitive Obsession; IEC 61131-3, MISRA
C:2012 and ASAM MCD-2 MC texts (third-party pages only); Kopetz 2011 (F); Kopetz & Suri 2003 and
Stewart, Volpe & Khosla (search snippets); AADL port semantics; IP-XACT; the Ada 83 Rationale; the
Habit report; SRS_BSW_00414.

**Conclusion** — The split and the nominal separation of same-shape types belong to Parnas, Morris,
Cardelli and Wegner, Simonyi and Ada 83; the kind table and binding it to the type are the
principle's own. AUTOSAR shows binding the kind to the role is a coherent alternative, and the
Runtime configuration row (one type in NVM and in RAM) already follows it. Bears on
[StructSemantics.md](../Principles/StructSemantics.md): its Source table, the bullets *A second
place is not a second kind*, *What a second type buys* and *The converse*, and the prior art on the
open units item come from this finding (applied 2026-11-22).
