
## Facets:

A document is classified by multiple orthogonal axes — subject, format, time, place, audience — rather than a single hierarchical category. This is the literal origin of the term "facet."

In software: a single domain object (Motor_T) can be sliced along orthogonal concern axes:

User-facing → Motor_User (external commands)
Parameter access → Motor_Var (the variable system's view)
Control math → Motor_FOC (FOC computations)
State driver → Motor_StateMachine (lifecycle/transitions)
Each facet is a complete view of the object along one axis. They don't replace each other; they coexist. A reader who needs the parameter view goes to Motor_Var.h and gets a complete answer for that axis.


## Motor Module boundary:
even PolePairs is generic. The fact that some parameters are dangerous to write while the system is active is not a motor-physics concern — it's a general property of writing parameters into a running system.
The MC layer should own all of it. The motor layer's job is to expose the schema and the setter side-effects, nothing more.

Motor module — schema + setters + FSM. All three are intrinsic to "what a motor module is": the data it holds, how it mutates that data consistently, and how it controls the hardware it's connected to.

MC layer — session lifecycle + admission.

The motor layer should not be in the business of answering generic systems questions. It should answer motor-domain questions. The split:

Motor: physics, electrical safety, struct consistency, schema.
MC (or a parameter service at the MC layer): session, lifecycle, write admission, classification of "which parameters need which window."
Each layer answers exactly the questions it has unique knowledge to answer. Generic concerns are unified at one layer. Motor-domain concerns stay at the motor layer. Motor_IsConfig was crossing that line and should go.



