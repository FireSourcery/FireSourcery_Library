# API Scoping

The rules that apply [InterfaceSegregation.md](../Principles/InterfaceSegregation.md). `CLAUDE.md`
holds the same rules under "API Scoping".

- **A parameter is the narrowest struct that contains every field the function reads or writes.**
  Taking a Config_T states "reads only config".
- **Name by scope where several scopes share a module's namespace:** `Module_Scope_Verb()`, where
  Scope names the struct, e.g. [Motor_Config_IsValid]. The primary structs, the module's
  `_State_T` and `_Config_T`, are exempt.
- **A public function's scope prefix follows the widest struct its effects reach**, not the struct
  it takes. A mutator whose change propagates scopes up even when its parameter is narrow.
- **A leading underscore marks a module-internal helper**, e.g. [_FOC_ProcVCircle]. It is not used for
  a narrower-scope public API.
- **Promote to a wider scope only on demand:** add a thin forwarder taking the wider handle only when
  a real caller holds it often enough that the extra typing matters.
