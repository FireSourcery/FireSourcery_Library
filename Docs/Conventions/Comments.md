# Comments

Code expresses everything code can express. A comment that restates a declaration, a constant, a bit
layout or a range is duplicated code: two sources of truth that drift. Comments hold the why the
language cannot state. `CLAUDE.md` holds the short form under Core Principles and Documentation.

- **Before writing a comment, ask what in it is not already in the code.** If nothing, delete it;
  otherwise keep only that.
- **Move a fact into the code**: a name, an inline field comment, a derived macro, a
  `static_assert`.
  - `static_assert(CIA402_OD_PROFILE_LAST == 0x9FFFU)` replaces "eight pages fill the device
    profile area".
  - A bitfield declaration replaces a bit diagram: `{Offset:12, Area:4}` already says which bits.
  - A `switch` over named enum values carries a range map better than a table in a comment.
  - A field-list block above a struct whose fields carry `/* Index : 0x605A */` inline is the copy
    that goes stale.
- **Express is not compute.** A mask literal is the declarative statement of a bit extent:
  `(0x07FFU)` reads at a glance, `((1U << CIA402_OD_OBJECT_BITS) - 1U)` has to be evaluated. Derive
  a constant only when it states a relationship that must hold
  (`PROFILE_LAST = BASE + COUNT * AXIS_OFFSET`); write the primitive fact as a literal and tie it to
  its relationship with `static_assert`.
- **The domain formula over the bit trick.** `(index.Index - CIA402_OD_PROFILE_BASE) /
  CIA402_OD_AXIS_OFFSET` states the addressing rule; `index.Page - CIA402_OD_PROFILE_PAGE` makes the
  reader know that a page is an axis step. GCC folds the first into the second, so check the codegen
  rather than hand-optimizing.
- **Keep**: rationale no declaration states ("Axis is a biased page; bits [13:11] alone wrap
  4,5,6,7,0,1,2,3"), a one-line orientation at a file or section head, and a cross-reference to
  where the full explanation lives.
