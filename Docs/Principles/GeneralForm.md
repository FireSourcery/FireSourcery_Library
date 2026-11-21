# General Form

**Statement** — Before adding a type, layer or adapter, ask which existing interface or published
pattern it is an instance of. Conforming to that interface lets every consumer of it apply
unchanged. Simplicity is measured on the resulting design, its concepts and adapters, not on the
diff.

**Rationale** — An interface is worth its consumers: the engine, framework functions and tables
written against it. A parallel type is cut off from all of them, so each consumer is either written
again for it or reached through an adapter. Conforming costs once, at the type; not conforming costs
at every consumer, now and later. A minimal diff that adds a parallel type keeps the line count low
and the concept count up. The limit is the contract: a type that would fake half the interface, or
break its rules, is not an instance of it, and the direct code wins. The principle names an
interface that already exists with consumers; it does not invent one for a single case.

**Implications**

- The sign in a draft: an adapter from a framework type to a module's own, or a parallel struct
  beside one the framework already defines. Either means the two are one interface.
- The general form is proposed beside the minimal change, naming what it unlocks, and the owner
  chooses. Minimal impact keeps unrelated code untouched; it does not veto the general form.
- The Mot protocol is a `Packet_Codec_T` (`MOT_PACKET_CODEC`), so the `Framework/Protocol` engine
  parses and frames it unchanged. See [ProtocolStack.md](../Patterns/ProtocolStack.md).
- A function given an interface table's shape, context first, sits in the row without an adapter
  wrapper: the API Design rule on interface tables in `CLAUDE.md`.

**Source** — "Program to an interface, not an implementation" (Gamma, Helm, Johnson and Vlissides,
*Design Patterns*, 1994). The Open–Closed Principle (Meyer, *Object-Oriented Software
Construction*, 1988): the consumers take a new type without modification. The Liskov Substitution
Principle (Liskov, "Data Abstraction and Hierarchy", 1987) for the limit.
