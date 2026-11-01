# Descriptor Template

A generic primitive specialized by a [Descriptor](Descriptor.md). The descriptor holds the shape
(element size and length; time source and frequency), the primitive takes it beside the state, and
the fold at each inlined call compiles one body to a hard-coded copy per shape. The `T` suffix names
the module: `RingT`, `TimerT`.

**Level** — idiom: C has no template parameter.

**Force** — A container or primitive runs one body over many shapes. What varies per instance is the
state; what stays still is the shape, fixed per instance at build time. A shape kept in the state
costs RAM and never folds, and a generator macro per shape is banned. The shape has to be a `const`
argument the optimizer can see.

**Shape**

```c
typedef const struct Ring_Type { size_t TYPE_SIZE; size_t LENGTH; } Ring_Type_T;              /* descriptor */
typedef const struct RingT { Ring_Type_T TYPE; Ring_State_T * P_STATE; } RingT_T;            /* pairing handle */

typedef const struct Timer_Base { const volatile uint32_t * P_TIME; uint32_t FREQ; } Timer_Base_T;
typedef const struct TimerT { Timer_Base_T BASE; Timer_State_T * P_STATE; } TimerT_T;
```

| Layer | Takes | Ring | Timer |
|---|---|---|---|
| Pure | values | `array_index_of_counter`, `void_array_at` | `_Timer_Elapsed(time_prev, time)` |
| Primitive | descriptor, state, values | `RingT_PushBack(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit)` | `_TimerT_IsElapsed(const Timer_Base_T * p_base, const Timer_State_T * p_state)` |
| Handle | the pairing handle | `RING_T_ARGS(ring)`, expanding to `(ring).TYPE, (ring).P_STATE` | `TimerT_IsElapsed(TimerT_T * p_timer)` |

- The primitive takes descriptor and state as separate parameters, descriptor first:
  `(p_const, p_state, value)` ([WrapperSurface.md](WrapperSurface.md)). It is the shared layer:
  `TimerT_T` in FLASH and `Timer_T` in RAM (`Base` and `State` by value, `Timer_General.h`) both call
  the same `_TimerT_*`.
- The handle is for the call site, not a structural requirement. It earns itself by the pairing
  invariant: a `TYPE` and a `P_STATE` taken from different rings is silent corruption, and drawing
  both from one handle makes it unrepresentable.
- The descriptor embeds by value in the handle. The handle is `const`, lives in FLASH and nests by
  value in a module descriptor: `Motor_T.CONTROL_TIMER`, `UART_T.TX_RING`.
- One descriptor value serves any number of states; the state never holds the shape.
- The handle's state may live inside its owner's state: `BLINKY_INIT_FROM` points `TIMER.P_STATE`
  at `&p_State->TimerState`.
- The initializer checks the shape at translation: `RING_TYPE_INIT` fails for a `LENGTH` that is not
  a power of 2. `RING_T_INIT_AS(T, Length)` derives `sizeof(T)` and sizes the state from the same
  arguments.

**Passing form** — proposed, not yet followed by every instance:

- A type's passing form follows its weight. Small descriptors travel by value, heavy ones by
  reference; state always travels by reference.
- Storage follows the same weight. A small, bounded descriptor embeds by value in the handle, and
  the handle is passed by pointer. A heavy or growing descriptor is held by pointer, and the handle,
  two pointers, may be passed by value.
- The threshold is the AAPCS argument registers, 16 bytes in r0–r3, not symmetry with peer modules.
- It applies out of line only. Inlined with a visible descriptor, both forms fold: Ring by value and
  Timer by pointer ([Descriptor.md](Descriptor.md) § Folding;
  [StructSemantics.md](../Principles/StructSemantics.md), by value means `static inline`).

| | Descriptor | Primitive takes it | Handle | Handle passed |
|---|---|---|---|---|
| Ring | `Ring_Type_T`, 8 B | by value | `RingT_T`, 12 B | unpacked at the call by `RING_T_ARGS` |
| Timer | `Timer_Base_T`, 8 B | by `const *` | `TimerT_T`, 12 B | by pointer |

**Open**

1. `_TimerT_*` takes the 8-byte `Timer_Base_T` by pointer; the passing form says by value.
2. The layer names differ. Timer: `_TimerT_` is the primitive, `TimerT_` the handle. Ring: `_RingT_`
   is the unchecked primitive and `RingT_` the checked one; the handle layer is only `RING_T_ARGS`,
   marked `todo replace`. `RingT.h` numbers its layers Level 1 to 3 on a different split.
3. The heavy-descriptor branch has no instance.
4. A `RING_STATE_ALLOC` state pointer does not fold; a named state does ([Descriptor.md](Descriptor.md)
   § Folding).

**In code**

- `RingT` — `Framework/Ring/_RingT.h`, `Framework/Ring/RingT.h`. Handles: `UART_T.RX_RING`,
  `UART_T.TX_RING` in `Peripheral/UART/UART.h`.
- `TimerT` — `Framework/Timer/_TimerT.h`, `Framework/Timer/TimerT.h`. Handles:
  `Motor_T.CONTROL_TIMER` and `SPEED_TIMER`, `MotorController_T.MILLIS_TIMER`, `Blinky_T.TIMER`.

**Rejected**

- **The shape in the state**, the commented-out `Ring_InitFrom` in `Framework/Ring/Ring.c`, which
  copied a `Ring_Type_T` into the ring. RAM per ring, and the stride is a load and a `muls` at every
  access.
- **Generator macros** that paste a typed copy per shape, `DEFINE_RING(uint32_t)`. Token pasting is
  banned (`CLAUDE.md`).
- **`_Generic` selection.** It picks a pre-written function by static type: it cannot specialize on a
  value such as a length, and needs one function per type.

**Source** — Flyweight (Gamma, Helm, Johnson and Vlissides, *Design Patterns*, 1994): the shared,
immutable intrinsic state lives in the flyweight, and the client passes the extrinsic state in. The
descriptor is the intrinsic part, the state the extrinsic; `_RingT.h` calls `Ring_Type_T` a
flyweight shape descriptor. C++ non-type template parameters are the form C lacks. For stride from
the caller, `RingT.h` cites Linux `DECLARE_KFIFO`, Stroustrup, *The C++ Programming Language* 4e
§25.3.4.1, and Stepanov, *Elements of Programming* §7.
