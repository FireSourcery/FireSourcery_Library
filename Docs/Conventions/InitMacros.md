# Init Macros

An `*_INIT` macro does something a designated initializer at the config site cannot do as clearly.
It earns its place when it:

- **casts** a typed handler to its registration type ([../Patterns/FunctionPointerCasting.md](../Patterns/FunctionPointerCasting.md));
- **allocates** sub-state inline, `.P_STATE = X_STATE_ALLOC()`;
- **derives** a field the caller would compute by hand and get wrong: a marker bit from an index,
  a size from a type;
- **builds a constant expression** an inline function cannot, such as a mask in a static
  initializer.

`RING_T_INIT_AS(T, Length)` in `RingT.h` is the reference: it derives `sizeof(T)` and allocates the
state buffer in one place.

If every argument lands in a field unchanged, delete the macro and write designated initializers at
the config site. Keep the `_ALLOC()` macro separately so the initializer can still use it.

Why: a macro that only renames fields positionally hides the field names, makes argument order one
more thing to get wrong, and breaks the grep from a struct member back to its config site.
