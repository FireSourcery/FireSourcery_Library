# Principles

Why the code is shaped the way it is. A principle is the reason behind conventions, patterns and
decisions. It is not applied every time like a convention, nor kept for one force like a pattern,
nor tied to one point in time like a decision. The format is TOGAF's architecture principle: name,
statement, rationale, implications.

---

## Entry shape

One file per principle, named for what it is (`InformationExpert.md`).

```markdown
# <Principle>

**Statement** — the principle, in a sentence or two.
**Rationale** — why it holds.
**Implications** — what follows: the conventions, patterns and decisions that apply it.
**Source** — the published form, if any.
```

---

## Written

| File | Statement |
|---|---|
| [InformationExpert.md](InformationExpert.md) | Each layer answers the questions only it has the knowledge for; a generic concern is answered once, at one layer |
| [InterfaceSegregation.md](InterfaceSegregation.md) | A signature states what a function depends on: the narrowest struct that holds what it reads or writes |
| [DominantDecomposition.md](DominantDecomposition.md) | A tree carries one axis of a classification; choose which mechanism carries each of the others |
