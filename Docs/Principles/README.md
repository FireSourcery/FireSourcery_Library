# Principles

Why the code is shaped the way it is. A principle is the reason behind conventions, patterns and
decisions, and it holds whatever the notation: breaking it costs something in itself, even for a
reader who expected the break. A convention may rest on a principle or on coordination alone, a
pattern is kept for one force, a decision is tied to one point in time. A principle is named for
its reason, not for the rule it yields. The format is TOGAF's architecture principle: name,
statement, rationale, implications.

---

## Entry shape

One file per principle, named for what it is (`InformationExpert.md`).

```markdown
# <Principle>

**Statement** — the principle, in a sentence or two.
**Rationale** — why it holds.
**Implications** — what follows: the conventions, patterns and decisions that apply it.
**Instances** — where it shows in this repo's code, by path; a consumer's copy carries its own.
**Source** — the published form, if any.
```

---

## Written

| File | Statement |
|---|---|
| [InformationExpert.md](InformationExpert.md) | Each layer answers the questions only it has the knowledge for; a generic concern is answered once, at one layer |
| [InterfaceSegregation.md](InterfaceSegregation.md) | A signature states what a function depends on: the narrowest struct that holds what it reads or writes |
| [DominantDecomposition.md](DominantDecomposition.md) | A tree carries one axis of a classification; choose which mechanism carries each of the others |
| [StructSemantics.md](StructSemantics.md) | A struct's kind, set by who writes it, when, where it lives and how it is passed, decides its storage, passing form and API; its shape does not |
| [GeneralForm.md](GeneralForm.md) | A new type first asks which existing interface it is an instance of; conforming lets every consumer apply unchanged |
| [SimplicityFavorsRegularity.md](SimplicityFavorsRegularity.md) | A family keeps one shape: an exception folds into a designated member, costing once, instead of taxing every consumer; the explicit split is for outcomes consumers act on differently |
| [BooleanBlindness.md](BooleanBlindness.md) | A `bool` says that something holds, not what: an on/off is carried by whatever already has the meaning, never by a flag beside it |
| [OpenClosed.md](OpenClosed.md) | A working function is closed to modification and open to extension: a repair goes in place, and other new logic extends it from outside, as a wrapping function or a sibling with its own name |
| [KindsOfChange.md](KindsOfChange.md) | A change is one kind at a time, refactor, repair, extension or optimization, and its kind decides what it needs, where it goes and how it is verified |
