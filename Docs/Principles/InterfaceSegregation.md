# Interface Segregation

**Statement** — A signature is a capability contract: it states what the function depends on. Each
module interface exposes the minimum a client needs, and each function takes the narrowest struct
that holds every field it reads or writes. A module may therefore offer several access points into
the same state, e.g. [Traction_Input_T].

**Rationale** — A client should not be forced to depend on what it does not use; for a function,
that means not supplying state it does not read. A narrower parameter type is also a smaller
physical dependency: a function taking a Config_T can live in a lower-tier header than one taking
the outer object, and the compiler then enforces the layering.

**Implications**

- The naming and scoping rules in [ApiScoping.md](../Conventions/ApiScoping.md).
- One object read through several complete views, [Facets.md](../Patterns/Facets.md).

**Source** — The Interface Segregation Principle (Martin, *Agile Software Development: Principles,
Patterns, and Practices*, 2002), applied to function parameters. Physical dependencies and
levelization (Lakos, *Large-Scale C++ Software Design*, 1996).
