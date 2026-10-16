



## Motor Module boundary:
even PolePairs is generic. The fact that some parameters are dangerous to write while the system is active is not a motor-physics concern — it's a general property of writing parameters into a running system.
The MC layer should own all of it. The motor layer's job is to expose the schema and the setter side-effects, nothing more.

Motor module — schema + setters + FSM. All three are intrinsic to "what a motor module is": the data it holds, how it mutates that data consistently, and how it controls the hardware it's connected to.

MC layer — session lifecycle + admission.

The motor layer should not be in the business of answering generic systems questions. It should answer motor-domain questions. The split:

Motor: physics, electrical safety, struct consistency, schema.
MC (or a parameter service at the MC layer): session, lifecycle, write admission, classification of "which parameters need which window."
Each layer answers exactly the questions it has unique knowledge to answer. Generic concerns are unified at one layer. Motor-domain concerns stay at the motor layer. Motor_IsConfig was crossing that line and should go.



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



## Facets:
A document is classified by multiple orthogonal axes — subject, format, time, place, audience — rather than a single hierarchical category. This is the literal origin of the term "facet."

In software: a single domain object (Motor_T) can be sliced along orthogonal concern axes:

User-facing → Motor_User (external commands)
Parameter access → Motor_Var (the variable system's view)
Control math → Motor_FOC (FOC computations)
State driver → Motor_StateMachine (lifecycle/transitions)
Each facet is a complete view of the object along one axis. They don't replace each other; they coexist. A reader who needs the parameter view goes to Motor_Var.h and gets a complete answer for that axis.

## A folder tree axis:
- So the goal is not "clean separation on all axes" — that's unreachable — it's decide which mechanism carries which axis, and push the leftover leakage to the cheapest spot.
- There is no arrangement that makes all three load-bearing in the same mechanism.
- A folder tree and a class hierarchy are one-dimensional. When you project a 3-D classification onto a 1-D spine: whichever axis you make structural, the other two have to leak out somewhere — as naming conventions, comment banners, parallel files, or property flags.
"Data source," "configurability," and "raw/processed" are not three ways to slice the same cake — they're three independent dimensions that cross-cut every value.
Motor speed is source: device-const-frame + config; configurability: read-only output; raw/processed: Fract16 bits ⇄ rpm. Every variable has a coordinate on all three axes at once.
