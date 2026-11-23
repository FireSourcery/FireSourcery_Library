# Enum Named Values

The enumerators are values of the quantity the enum classifies, in that quantity's units. Converting
between the quantity and the enum is then arithmetic, such as a mask, a sign, `abs` or a product,
instead of a table or a switch. The bound-value form makes each enumerator the lower bound of its range.

**Level** — idiom.

**Force** — An enum classifies a quantity: which quadrant an angle is in, which side and level of a
band a reading is in, which direction something turns. The input varies. The mapping between the
quantity and its class has to stay fixed, and so does every query on the class. With sequential
enumerators 0, 1, 2, … that mapping has to be written in both directions (to classify a value and
to recover it), plus a switch for each query.

**Shape** — two forms.

Bound value: each enumerator is the lower bound of its range. When the ranges are equal powers of
two, a mask classifies, and the masked value is the enumerator.

```c
#define ANGLE16_QUADRANT_MASK (0xC000U)

typedef enum angle16_quadrant
{
    ANGLE16_QUADRANT_I      = 0U,       /* 0_90 */
    ANGLE16_QUADRANT_II     = 0x4000U,  /* 90_180 */
    ANGLE16_QUADRANT_III    = 0x8000U,  /* 180_270 */
    ANGLE16_QUADRANT_IV     = 0xC000U,  /* 270_360 */
}
angle16_quadrant_t;

static inline angle16_quadrant_t angle16_quadrant(angle16_t theta) { return (angle16_quadrant_t)((uint16_t)theta & ANGLE16_QUADRANT_MASK); }
```

Signed level: the sign gives the direction, the magnitude gives the level, and zero is the neutral
case. One value holds both, and queries are `abs`, the sign and comparison.

```c
typedef enum RangeMonitor_Status
{
    MONITOR_STATUS_FAULT_UNDER_LIMIT = (MONITOR_STATUS_FAULT * -1),
    MONITOR_STATUS_WARNING_LOW = (MONITOR_STATUS_WARNING * -1),
    RANGE_MONITOR_STATUS_NORMAL = MONITOR_STATUS_NORMAL,
    MONITOR_STATUS_WARNING_HIGH = MONITOR_STATUS_WARNING,
    MONITOR_STATUS_FAULT_OVER_LIMIT = MONITOR_STATUS_FAULT,
}
RangeMonitor_Status_T;

static inline Monitor_Status_T RangeMonitor_GetSeverity(const RangeMonitor_T * p_monitor) { return (Monitor_Status_T)abs(p_monitor->Status); }
static inline bool RangeMonitor_IsSeverityRising(const RangeMonitor_T * p_monitor) { return abs(p_monitor->Status) > abs(p_monitor->StatusPrev); }
```

A direction of ±1 is the signed form with a single level, and the value is used as a multiplier.
`Motor_QuadrantId` builds a class as a product: `motion * (1 + generating + plugging)`.

**Rules**

- **The set is closed.** Every value the arithmetic can produce is an enumerator: the quadrant mask
  gives one of four, and `abs` of a range status gives one of the three `Monitor_Status_T`. A
  quantity that can take values between the names, such as a circle radius in
  `FOC_V_CIRCLE_PER_V_BUS`, is not an enum type. Its names are constants of its own type.
- **Choose the value the consumers use.** Use the quantity's value where callers compare or compute
  with it, and an index where the enum indexes an array.
- **Zero is the neutral case**: `ANGLE16_QUADRANT_I`, `RANGE_MONITOR_STATUS_NORMAL`,
  `MOTOR_QUADRANT_IDLE`, `MOTOR_DIRECTION_NULL`. A zero-initialized field is then neutral, the same
  reasoning as `DISABLED = 0` in [BooleanState.md](../Conventions/BooleanState.md).
- **Derive values from the base, not from repeated literals.** `RangeMonitor_Status_T` is built from
  `Monitor_Status_T`, and `MOTOR_DIRECTION_REVERSE` is defined as `MOTOR_DIRECTION_CW`.
  `angle16_quadrant_t` repeats `0x4000U` because `ANGLE16_90` is a `static const` object, which
  is not a constant expression.
- **Cast once, inside the arithmetic.** The only cast back to the enum type is in the function that
  classifies (`angle16_quadrant`, `Motor_QuadrantId`, `RangeMonitor_GetSeverity`). Callers
  compare against the names.
- **A signed level is read with `abs`, never with a mask.** In two's complement −1 is `…11` and +1
  is `…01`, so the low bits do not carry the level.
- **A switch can still hold work that differs per case.** `fract16_sin` switches on
  `angle16_quadrant`: the named value does the classifying, and the switch does the work for each
  case.
- **Unequal ranges are classified by their high bits, and each enumerator is still the bound.**
  `OD_Area_Of` switches on `OD_AREA_ID(index)` and lists `OD_AREA_ID(OD_AREA_MANUFACTURER) + 1U`
  and so on for the wider areas.
- **The values are the contract.** Reordering or renumbering the enumerators breaks the arithmetic
  without any compiler error. The enum states the relation its arithmetic relies on, as the
  `@warning` on `RangeMonitor_Status_T` does.

**In code**

| Enum | Form | Arithmetic | Path |
|---|---|---|---|
| `angle16_quadrant_t` | bound | `theta & ANGLE16_QUADRANT_MASK` | `Math/Fixed/angle16.h` |
| `OD_Area_T` | bound, unequal ranges | switch on `OD_AREA_ID(index)` | `Motor/MotProtocol/CANopen/OD.h` |
| `RangeMonitor_Status_T` | signed level | `abs`, comparison of `abs` | `Transducer/Monitor/Base/RangeMonitor.h` |
| `Motor_QuadrantId_T` | signed level | `motion * (1 + generating + plugging)` | `Motor/Motor/Model/Motor_Types.h` |
| `sign_t`, `Motor_Direction_T` | sign | `OpenLoopRampSpeedFinal_Pu * Motor_Direction(p_context)`, `Config.DirectionForward * Motor_Direction(p_motor)` | `Math/math_general.h`, `Motor/Motor/Motor.h` |

**Rejected**

- **Sequential enumerators, with a table or switch in each direction.** Classifying and recovering
  the value become two mappings that must be kept in step, and each query (severity, direction)
  becomes a switch.
- **Separate fields for direction and level.** That is two fields for one reading, and they can
  disagree, for example NORMAL with a direction set. The signed form has a single zero, so this
  state cannot be represented.

**Cost** — MISRA C:2012's essential type model treats an enum as a type of its own. Arithmetic on an
enumerator (Rule 10.1) and the cast back to the enum (Rule 10.5) are deviations, one for each
classifying function.

**Source** — Precedents rather than a named idiom:

- The signum decomposition x = sgn(x)·|x|.
- C `memcmp`, `strcmp` and `qsort` comparators, and C++20 `std::strong_ordering`: in each, the sign
  of the result is the result.
- CiA 301, which defines the object dictionary areas by the index each one starts at.
