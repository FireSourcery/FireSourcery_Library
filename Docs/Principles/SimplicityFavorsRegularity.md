# Simplicity Favors Regularity

**Statement** — When one case would make a member differ in shape from its siblings — an optional
where the others are plain, a variant row in a uniform table — prefer the design that keeps the
family regular: pay a bounded cost once, inside the one case, rather than tax every consumer with
the exception. Simplicity is measured on the resulting design: a regular family stays one concept;
an exceptional member is a second concept at every point the family flows through.

**Rationale** — A family's consumers are written against its common shape: the dispatcher, the
table walk, the display loop. One irregular member forces each of them to carry a branch for it,
now and later — a diffuse, permanent cost. Folding the exception into the regular shape as a
designated member (an unknown, an identity, a zero row) costs once, where it is named. The limit is
honesty: when consumers must *act differently* on the exceptional outcome, hiding it inside the
regular shape is the lie, and the explicit split — a tagged union, a separate path — wins.
Regularity serves consumers that all act the same; the split serves consumers that cannot.

**Implications**

- The sign in a draft: an optional, a null, or a sentinel-checked branch on one member of a family
  whose siblings are plain; a special case repeated at each consumer, keyed to that one member.
- The remedy: a designated member inside the regular shape — a value outside a table degrades to a
  named unknown member, the raw value kept beside it; the family's shape does not change.
- Every Mot status table keeps 0 = ok, so one error test serves them all; a table that cannot is
  named as the exception, not silently different.
- Sibling paths (read beside write, encode beside decode) keep one shape; one of them growing a
  private construction the other lacks is the sign.
- With [GeneralForm.md](GeneralForm.md): the general form picks the existing interface so its
  consumers apply unchanged; regularity keeps siblings on one shape so their consumers stay
  single-shaped. Both measure simplicity on the resulting design, not on the diff.

**Source** — "Simplicity favors regularity": the first of the four hardware design principles
(Patterson and Hennessy, *Computer Organization and Design*). Instruction formats kept uniform so
decode stays simple, paying with the occasional unused field: the one consumer every instruction
flows through stays single-shaped.
