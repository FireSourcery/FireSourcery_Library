# Descriptor

A `typedef const struct`, defined once with a designated initializer, holding what is fixed for the
image: bindings, compile-time constants, and pointers to the instance's mutable parts. It is C's
stand-in for a template argument. A descriptor the compiler can see at an inlined call folds into
that call, so one function body compiles to a hard-coded copy per instance.

**Level** — architectural: every hardware binding and every generic primitive takes this form.

**Force** — An instance has parts fixed for the build (which peripheral, what element size, which
time base) and parts that change at runtime. The fixed parts should cost what a literal costs: no
RAM and no copy at init. C has no template parameter, and `CLAUDE.md` bans token pasting, so no code
is generated per instance. A `const` object the optimizer can see is the compile-time argument
that is left.

**Shape**

```c
typedef const struct Module
{
    HAL_Module_T * P_HAL;                   /* binding */
    uint32_t CHANNEL_ID;                    /* constant */
    Module_State_T * P_STATE;               /* RAM */
    const Module_Config_T * P_NVM_CONFIG;   /* NVM, if persisted */
}
Module_T;

static inline void Module_Op(Module_T * p_module) { HAL_Module_Op(p_module->P_HAL, p_module->CHANNEL_ID); }
```

- `typedef const struct` makes every `Module_T *` a pointer to const, and lets the object live in
  FLASH.
- Members are `UPPER_SNAKE_CASE`, pointers prefixed `P_` ([Naming.md](../Conventions/Naming.md)
  § Types).
- State is allocated at the initializer, `.P_STATE = MODULE_STATE_ALLOC()`
  ([InitMacros.md](../Conventions/InitMacros.md)).
- Descriptors nest by value: `Hall_T` holds three `Pin_T`, `Motor_T` holds `TimerT_T CONTROL_TIMER`.
  The board's top-level descriptors compose them.
- Only for hardware bindings and static polymorphism (`CLAUDE.md`). A struct that could be
  mutable-only is state, not a descriptor.

**Forms**

| Form | Functions take | Entry |
|---|---|---|
| Module descriptor: a hardware-bound module's handle, owning the pointers to its state and config | the descriptor alone, `Module_Op(p_module)` | [DescriptorPattern.md](DescriptorPattern.md) |
| Template descriptor: the shape of a generic primitive | descriptor and state as separate parameters; a pairing handle joins them | [DescriptorTemplate.md](DescriptorTemplate.md) |

**Folding**

A descriptor is visible when it is `const`, defined at file scope in the caller's translation unit,
and the call is inlined.

| Member | Visible | Behind a pointer |
|---|---|---|
| Integer constant: size, length, channel | an immediate; a stride becomes a shift, a length a mask | a load, then the arithmetic: `muls` for a stride |
| Pointer: state, register base, time source | a literal-pool word | a load from the descriptor |
| The descriptor itself | not emitted, if nothing else takes its address | in `.rodata` |

- One source, two specializations: `RingT_PushBack` on a `uint32_t` × 16 ring compiles to `lsls #2`,
  `cmp #16` and a mask of 15; on a `uint8_t` × 64 ring, a `strb` and a mask of 63. Behind a pointer
  it loads `TYPE_SIZE` and `LENGTH` and multiplies.
- The gain is in the integer members. `TimerT_IsElapsed` has only pointer members and compiles to
  the same instruction count either way.
- A pointer member folds when it is the address of a named object or of a plain compound literal
  (`TIMER_STATE_ALLOC()`). `RING_STATE_ALLOC`'s `&(union { … }){ }.State` does not: the descriptor
  stays in `.rodata` and each call loads `P_STATE`.
- Out-of-line module code (`UART.c`, the `Motor` `.c` files) receives its descriptor by pointer and
  pays the right-hand column. The fold is for inline call sites in the unit that defines the
  descriptor. HAL register bases: [Peripheral/Notes.md](../../Peripheral/Notes.md).
- Checked with `arm-none-eabi-gcc` 15.2.1, `-mcpu=cortex-m0plus -mthumb`, `-O2` and `-Os`,
  2026-10-31. Not measured: Cortex-M4, LTO.

**In code**

- `Pin_T` — `Peripheral/Pin/Pin.h`; `PWM_T` — `Peripheral/PWM/PWM.h`
- `Hall_T` — `Motor/Motor/Sensor/Hall/Hall.h`; `Motor_T` — `Motor/Motor/Motor.h`
- `RingT_T` — `Framework/Ring/RingT.h`; `TimerT_T` — `Framework/Timer/TimerT.h`

**Rejected**

- **Constants copied into RAM state at init**, the commented-out `Ring_InitFrom` in
  `Framework/Ring/Ring.c`. RAM per instance, writable, and nothing folds.
- **Code generated per instance** by token-pasting macros. Banned by `CLAUDE.md`, and each instance
  duplicates the body anyway.
- **A function-pointer table per instance.** The dispatch costs more than a register op. A table
  goes where a second implementation enters the image (Peripheral/Notes.md).
- **File-scope singletons the functions reach directly.** Hidden state, one instance per image, not
  reentrant.

**Source** — Partial evaluation, specializing a program on its static inputs (Jones, Gomard and
Sestoft, *Partial Evaluation and Automatic Program Generation*, 1993). Here the optimizer's constant
propagation through an inlined call does it. `CLAUDE.md` calls the pattern static polymorphism.
