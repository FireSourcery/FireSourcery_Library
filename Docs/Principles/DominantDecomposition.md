# Dominant Decomposition

**Statement** — A folder tree or a type hierarchy carries one axis of a classification. Where several
independent axes cross-cut the same items, decide which mechanism carries which axis, and push the
leftover to the cheapest spot. Clean separation on every axis is not reachable.

**Rationale** — A tree is one-dimensional. Projecting a classification of several dimensions onto
it makes one axis structural, and the others leak out as naming conventions, comment banners,
parallel files or property flags. The axes do not slice the same cake three ways: each item has a
coordinate on all of them at once. A motor variable has a data source (device constant, frame,
config), a configurability (here a read-only output) and a representation (fract16 bits or rpm),
all three at once.

**Implications**

- Choose the structural axis on purpose, and name the mechanism that carries each other axis.
- `Docs/` itself: the folders carry the kind of note (principle, convention, pattern, decision,
  finding), and the module rides in the paths under `Modules/` and in the names.
- One object's concerns split across files, [Facets.md](../Patterns/Facets.md).

**Source** — The tyranny of the dominant decomposition (Tarr, Ossher, Harrison and Sutton, "N Degrees
of Separation: Multi-Dimensional Separation of Concerns", ICSE 1999). Faceted classification
(Ranganathan, *Colon Classification*, 1933).
