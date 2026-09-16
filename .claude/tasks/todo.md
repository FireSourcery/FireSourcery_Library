- [x] Preserve the sampled speed counter while adding a dedicated homing-travel counter.
- [x] Apply configured timing and direction state before the encoder begins capturing edges.
- [x] Drain all coalesced channel flags and make decoder-only helpers use the owning encoder descriptor.
- [x] Remove the unresolved ModeDT API declaration or implement it only if an active caller needs it.
- [x] Validate with focused diagnostics and the configured build path.

## Review

- ARM GCC syntax checks pass for active KE06 emulated encoder sources and hardware-decoder Encoder.c/Encoder_ModeDT.c paths.
- The full CMake build remains unavailable because its Unix Makefiles generator cannot locate make on Windows.
