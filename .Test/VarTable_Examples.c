/******************************************************************************/
/*!
    @file   VarTable_Examples.c
    @author FireSourcery
    @brief  Worked [VField_T] tables, one per calling shape, with text export.

    Each table is declared once as a list and expanded twice:

        firmware        -> the [VField_T] table. The text costs nothing: an
                           unreferenced macro argument never reaches the compiler.
        VAR_TEXT_EXPORT -> the manifest feeding the host schema and the CSV.

    What the export knows, and from where:

        "accessor"  derived  - the function pointer's spelling. Preprocessor.
        "sig"       derived  - width and signedness of its return type, via
                               SIG_OF(typeof(get(ctx))). Needs a compile, not
                               just a preprocess.
        "writable"  derived  - whether a setter was supplied.
        "units"     declared - no compiler has an opinion on this.
        "type"      declared - what the value means. fract16_t and int16_t share
                               a "sig"; only this column separates them, and the
                               host needs it to pick EnumFormat vs BoolFormat vs
                               BitStructFormat vs a plain int.

    Build:
        arm-none-eabi-gcc -c -std=c23 -mcpu=cortex-m0plus -mthumb VarTable_Examples.c
        clang -std=c23 -DVAR_TEXT_EXPORT VarTable_Examples.c -o export && ./export

    "sig" is toolchain-dependent, not merely target-dependent: arm-none-eabi-gcc
    defaults to -fshort-enums (EABI), so Motor_StateId_T is one byte there and
    four under host clang. An exporter not built with the firmware's toolchain
    publishes the wrong width for every enum-returning accessor.
*/
/******************************************************************************/
#include "Var_Stub.h"

#ifdef VAR_TEXT_EXPORT
#include <stdio.h>
#endif

/******************************************************************************/
/*
    [MotVar_Sig_T] - (width, signedness). Not the C type name: on this target an
    enum is one byte and int32_t is `int`, so any mapping keyed on type identity
    collides. sizeof/signedness asks the question that decides the wire.
*/
/******************************************************************************/
typedef enum MotVar_Sig { SIG_I8, SIG_U8, SIG_I16, SIG_U16, SIG_I32, SIG_U32, _SIG_END } MotVar_Sig_T;

#define IS_SIGNED(ctype)  ((ctype)-1 < (ctype)0)
#define SIG_OF(ctype)                                                  \
    ((sizeof(ctype) == 1u) ? (IS_SIGNED(ctype) ? SIG_I8  : SIG_U8)  :  \
     (sizeof(ctype) == 2u) ? (IS_SIGNED(ctype) ? SIG_I16 : SIG_U16) :  \
                             (IS_SIGNED(ctype) ? SIG_I32 : SIG_U32))

typedef const struct VField
{
    void (*GET)(void);
    void (*SET)(void);
    MotVar_Sig_T SIG;
}
VField_T;

#ifdef VAR_TEXT_EXPORT
static const char * const SIG_NAMES[_SIG_END] = { "int8", "uint8", "int16", "uint16", "int32", "uint32" };

/*  Names arrive already stringified, before NULL can expand to ((void*)0). */
#define EXPORT_ROW(index, id, get, set, units, ctype, sig, writable)                    \
    printf("%s    { \"index\": %d, \"id\": \"%s\", \"get\": \"%s\", \"set\": \"%s\","   \
           " \"units\": \"%s\", \"type\": \"%s\", \"sig\": \"%s\", \"writable\": %s }", \
           (index) ? ",\n" : "", (index), id, get, set, units, ctype, SIG_NAMES[sig], writable)

/*  Stamp the ABI the manifest was produced under. "sig" is only meaningful when
    these match the firmware build: host clang reports enum_bytes 4 where
    arm-none-eabi-gcc reports 1, and -fshort-enums does not change that. A
    consumer that does not check this silently accepts the wrong widths. */
#define EXPORT_ABI()                                                                      \
    printf("  \"abi\": { \"enum_bytes\": %d, \"int_bytes\": %d, \"ptr_bytes\": %d },\n",  \
           (int)sizeof(Motor_StateId_T), (int)sizeof(int), (int)sizeof(void *))
#endif

/******************************************************************************/
/*
    (1) State context - [Motor_Context_T]

    One adapter per signature, not per variable. [void(*)(void)] round-trip back
    to the original type is defined by 6.3.2.3p8.
*/
/******************************************************************************/
typedef int32_t (*GetAdapter_State_T)(void (*)(void), const Motor_Context_T *);

#define GET_ADAPTER_STATE(name, ctype)                                     \
    static int32_t name(void (*fn)(void), const Motor_Context_T * p_motor) \
        { return (int32_t)((ctype (*)(const Motor_Context_T *))fn)(p_motor); }

GET_ADAPTER_STATE(GetState_I8,  int8_t)     GET_ADAPTER_STATE(GetState_U8,  uint8_t)
GET_ADAPTER_STATE(GetState_I16, int16_t)    GET_ADAPTER_STATE(GetState_U16, uint16_t)
GET_ADAPTER_STATE(GetState_I32, int32_t)    GET_ADAPTER_STATE(GetState_U32, uint32_t)

static const GetAdapter_State_T GET_ADAPTERS_STATE[_SIG_END] =
{
    [SIG_I8]  = GetState_I8,  [SIG_U8]  = GetState_U8,
    [SIG_I16] = GetState_I16, [SIG_U16] = GetState_U16,
    [SIG_I32] = GetState_I32, [SIG_U32] = GetState_U32,
};

/*  Naming the context is what lets SIG_OF see the return type, and what makes a
    wrong-context accessor a compile error rather than a bad call. */
#define SIG_STATE(get)  SIG_OF(typeof(get((const Motor_Context_T *)0)))

#define ROW_STATE(id, get, set, units, ctype) \
    { .GET = (void (*)(void))get, .SET = (void (*)(void))set, .SIG = SIG_STATE(get) },

typedef enum Motor_Var_UserOut
{
    MOTOR_VAR_SPEED, MOTOR_VAR_I_PHASE, MOTOR_VAR_STATE, MOTOR_VAR_SUB_STATE,
    _MOTOR_VAR_USER_OUT_END,
}
Motor_Var_UserOut_T;

/*  Implicitly indexed - row order is the id.      id, get, set, units, ctype */
#define USER_OUT_LIST(X)                                                          \
    X(MOTOR_VAR_SPEED,     Motor_User_GetSpeed_Fract16, NULL, Rpm,  accum32_t)    \
    X(MOTOR_VAR_I_PHASE,   Motor_GetIPhase_Fract16,     NULL, Amps, fract16_t)    \
    X(MOTOR_VAR_STATE,     Motor_GetStateId,            NULL, None, Motor_StateId_T) \
    X(MOTOR_VAR_SUB_STATE, Motor_GetPathId,             NULL, None, state_t)

static const VField_T USER_OUT_FIELDS[] = { USER_OUT_LIST(ROW_STATE) };

int32_t Motor_Var_UserOut_Get(const Motor_Context_T * p_motor, Motor_Var_UserOut_T id)
{
    if ((size_t)id >= _MOTOR_VAR_USER_OUT_END) { return 0; }
    VField_T field = USER_OUT_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS_STATE[field.SIG](field.GET, p_motor) : 0;
}

/******************************************************************************/
/*
    (2) Dev context - [Motor_T]

    A second context shape needs its own adapter family and its own row macro.
    [MotVar_Sig_T] encodes the return type only; the parameter type lives in the
    macro, which is why mixing shapes in one table cannot compile.
*/
/******************************************************************************/
typedef int32_t (*GetAdapter_Dev_T)(void (*)(void), const Motor_T *);

#define GET_ADAPTER_DEV(name, ctype)                              \
    static int32_t name(void (*fn)(void), const Motor_T * p_dev)  \
        { return (int32_t)((ctype (*)(const Motor_T *))fn)(p_dev); }

GET_ADAPTER_DEV(GetDev_I16, int16_t)    GET_ADAPTER_DEV(GetDev_I32, int32_t)

static const GetAdapter_Dev_T GET_ADAPTERS_DEV[_SIG_END] =
{
    [SIG_I16] = GetDev_I16, [SIG_I32] = GetDev_I32,   /* sparse: only what this group uses */
};

#define SIG_DEV(get)  SIG_OF(typeof(get((const Motor_T *)0)))

#define ROW_DEV(id, get, set, units, ctype) \
    { .GET = (void (*)(void))get, .SET = (void (*)(void))set, .SIG = SIG_DEV(get) },

typedef enum Motor_Var_DevOut { MOTOR_VAR_I_BUS, _MOTOR_VAR_DEV_OUT_END } Motor_Var_DevOut_T;

#define DEV_OUT_LIST(X) \
    X(MOTOR_VAR_I_BUS, Motor_GetIBus_Fract16, NULL, Amps, fract16_t)

static const VField_T DEV_OUT_FIELDS[] = { DEV_OUT_LIST(ROW_DEV) };

int32_t Motor_Var_DevOut_Get(const Motor_T * p_dev, Motor_Var_DevOut_T id)
{
    if ((size_t)id >= _MOTOR_VAR_DEV_OUT_END) { return 0; }
    VField_T field = DEV_OUT_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS_DEV[field.SIG](field.GET, p_dev) : 0;
}

/******************************************************************************/
/*
    (3) No context - board constants
*/
/******************************************************************************/
typedef int32_t (*GetAdapter_Const_T)(void (*)(void));

#define GET_ADAPTER_CONST(name, ctype)     \
    static int32_t name(void (*fn)(void))  \
        { return (int32_t)((ctype (*)(void))fn)(); }

GET_ADAPTER_CONST(GetConst_I16, int16_t)    GET_ADAPTER_CONST(GetConst_U16, uint16_t)

static const GetAdapter_Const_T GET_ADAPTERS_CONST[_SIG_END] =
{
    [SIG_I16] = GetConst_I16, [SIG_U16] = GetConst_U16,
};

#define SIG_CONST(get)  SIG_OF(typeof(get()))

#define ROW_CONST(id, get, set, units, ctype) \
    { .GET = (void (*)(void))get, .SET = NULL, .SIG = SIG_CONST(get) },

typedef enum Motor_Var_Board
{
    MOTOR_VAR_BOARD_V_RATED, MOTOR_VAR_BOARD_I_RATED_PEAK, MOTOR_VAR_BOARD_V_MAX_VOLTS,
    _MOTOR_VAR_BOARD_END,
}
Motor_Var_Board_T;

#define BOARD_LIST(X)                                                                                 \
    X(MOTOR_VAR_BOARD_V_RATED,      Phase_VRated_Fract16,     NULL, Volts, fract16_t)  \
    X(MOTOR_VAR_BOARD_I_RATED_PEAK, Phase_IRatedPeak_Fract16, NULL, Amps,  fract16_t)  \
    X(MOTOR_VAR_BOARD_V_MAX_VOLTS,  Phase_VMaxVolts,          NULL, Volts, uint16_t)

static const VField_T BOARD_FIELDS[] = { BOARD_LIST(ROW_CONST) };

int32_t Motor_Var_Board_Get(Motor_Var_Board_T id)
{
    if ((size_t)id >= _MOTOR_VAR_BOARD_END) { return 0; }
    VField_T field = BOARD_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS_CONST[field.SIG](field.GET) : 0;
}

/******************************************************************************/
/*
    (4) Setters - already uniform

    Every setter is void(Motor_Context_T *, int), so there is one signature, and
    therefore no adapter table and no [MotVar_Sig_T] on the write path. The sig
    column still exports, describing what the wire value is narrowed to.
*/
/******************************************************************************/
typedef void (*Set_State_T)(Motor_Context_T *, int);

#define ROW_SETPOINT(id, get, set, units, ctype) \
    { .GET = NULL, .SET = (void (*)(void))set, .SIG = SIG_OF(ctype) },

typedef enum Motor_Var_Setpoint
{
    MOTOR_VAR_SETPOINT_SPEED, MOTOR_VAR_SETPOINT_TORQUE, MOTOR_VAR_SETPOINT_I,
    _MOTOR_VAR_SETPOINT_END,
}
Motor_Var_Setpoint_T;

#define SETPOINT_LIST(X)                                               \
    X(MOTOR_VAR_SETPOINT_SPEED,  NULL, Motor_SetSpeedCmd,  Rpm,  int)  \
    X(MOTOR_VAR_SETPOINT_TORQUE, NULL, Motor_SetTorqueCmd, Amps, int)  \
    X(MOTOR_VAR_SETPOINT_I,      NULL, Motor_SetICmd,      Amps, int)

static const VField_T SETPOINT_FIELDS[] = { SETPOINT_LIST(ROW_SETPOINT) };

void Motor_Var_Setpoint_Set(Motor_Context_T * p_motor, Motor_Var_Setpoint_T id, int32_t value)
{
    if ((size_t)id >= _MOTOR_VAR_SETPOINT_END) { return; }
    VField_T field = SETPOINT_FIELDS[id];
    if (field.SET != NULL) { ((Set_State_T)field.SET)(p_motor, (int)value); }
}

/******************************************************************************/
/*
    (5) Union return

    [Motor_FaultFlags_T] is a union, so IS_SIGNED cannot be evaluated on it. A
    .Value accessor selects the wire member rather than forwarding, so it earns
    its place where a plain pass-through would not. The declared type stays the
    union - that is what tells the host to render it as a bit struct.
*/
/******************************************************************************/
static inline uint8_t Motor_GetFaultFlags_Value(const Motor_Context_T * p) { return Motor_GetFaultFlags(p).Value; }

typedef enum Motor_Var_Status { MOTOR_VAR_FAULT_FLAGS, _MOTOR_VAR_STATUS_END } Motor_Var_Status_T;

#define STATUS_LIST(X) \
    X(MOTOR_VAR_FAULT_FLAGS, Motor_GetFaultFlags_Value, NULL, None, Motor_FaultFlags_T)

static const VField_T STATUS_FIELDS[] = { STATUS_LIST(ROW_STATE) };

int32_t Motor_Var_Status_Get(const Motor_Context_T * p_motor, Motor_Var_Status_T id)
{
    if ((size_t)id >= _MOTOR_VAR_STATUS_END) { return 0; }
    VField_T field = STATUS_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS_STATE[field.SIG](field.GET, p_motor) : 0;
}

/******************************************************************************/
/*
    Text export - the second expansion of the same lists.
    Absent from the firmware build; none of these strings reach the compiler.
*/
/******************************************************************************/
#ifdef VAR_TEXT_EXPORT

#define WRITABLE(set)  (((set) != NULL) ? "true" : "false")

/*  #get here, not inside EXPORT_ROW: by the time the argument reaches another
    macro parameter it has already expanded, and NULL becomes ((void*)0). */
#define TEXT_STATE(id, get, set, units, ctype)  EXPORT_ROW(i, #id, #get, #set, #units, #ctype, SIG_STATE(get), WRITABLE(set)); i++;
#define TEXT_DEV(id, get, set, units, ctype)    EXPORT_ROW(i, #id, #get, #set, #units, #ctype, SIG_DEV(get),   WRITABLE(set)); i++;
#define TEXT_CONST(id, get, set, units, ctype)  EXPORT_ROW(i, #id, #get, #set, #units, #ctype, SIG_CONST(get), WRITABLE(set)); i++;
#define TEXT_SET(id, get, set, units, ctype)    EXPORT_ROW(i, #id, #get, #set, #units, #ctype, SIG_OF(ctype),  WRITABLE(set)); i++;

#define EXPORT_TABLE(name, list, row, tail) \
    do { int i = 0; printf("  \"%s\": [\n", name); list(row) printf("\n  ]%s\n", tail); } while (0)

int main(void)
{
    printf("{\n");
    EXPORT_ABI();
    EXPORT_TABLE("MOTOR_USER_OUT", USER_OUT_LIST, TEXT_STATE, ",");
    EXPORT_TABLE("MOTOR_DEV_OUT",  DEV_OUT_LIST,  TEXT_DEV,   ",");
    EXPORT_TABLE("MOTOR_BOARD",    BOARD_LIST,    TEXT_CONST, ",");
    EXPORT_TABLE("MOTOR_SETPOINT", SETPOINT_LIST, TEXT_SET,   ",");
    EXPORT_TABLE("MOTOR_STATUS",   STATUS_LIST,   TEXT_STATE, "");
    printf("}\n");
    return 0;
}

#endif
