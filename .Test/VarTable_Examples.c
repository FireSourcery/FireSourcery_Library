/******************************************************************************/
/*!
    @file   VarTable_Examples.c
    @author FireSourcery
    @brief  Worked [VField_T] tables, one per calling shape.

    Compiles standalone against [Var_Stub.h]:
        arm-none-eabi-gcc -c -std=c23 -mcpu=cortex-m0plus -mthumb -Wall -Wextra -Werror VarTable_Examples.c

    Each table demonstrates one thing the previous one could not express.
*/
/******************************************************************************/
#include "Var_Stub.h"

/******************************************************************************/
/*
    [MotVar_Sig_T] - (width, signedness). Not the C type name: on this target
    an enum is 1 byte (-fshort-enums, ARM EABI) and int32_t is `int`, so any
    mapping keyed on type identity collides.
*/
/******************************************************************************/
typedef enum MotVar_Sig { SIG_I8, SIG_U8, SIG_I16, SIG_U16, SIG_I32, SIG_U32, _SIG_END } MotVar_Sig_T;

#define IS_SIGNED(ctype)  ((ctype)-1 < (ctype)0)
#define SIG_OF(ctype)                                                  \
    ((sizeof(ctype) == 1u) ? (IS_SIGNED(ctype) ? SIG_I8  : SIG_U8)  :  \
     (sizeof(ctype) == 2u) ? (IS_SIGNED(ctype) ? SIG_I16 : SIG_U16) :  \
                             (IS_SIGNED(ctype) ? SIG_I32 : SIG_U32))

/*
    The declared [ctype] exists for text export - it carries what [MotVar_Sig_T]
    cannot (fract16_t vs int16_t, an enum vs a byte). Assert it agrees with the
    accessor at the level the compiler can see, so it cannot silently rot.
*/
#define ASSERT_SIG(ctype, actual) \
    (0 * sizeof(struct { static_assert(SIG_OF(ctype) == SIG_OF(actual), "declared type disagrees with accessor"); int _; }))

typedef const struct VField
{
    void (*GET)(void);
    void (*SET)(void);
    MotVar_Sig_T SIG;
}
VField_T;

/******************************************************************************/
/*
    (1) State context - [Motor_Context_T]

    The baseline. One adapter per signature, not per variable.
    [void(*)(void)] round-trip back to the original type is defined by 6.3.2.3p8.
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

/* Naming the context is what makes a wrong-context accessor a compile error. */
#define VAR_FIELD_STATE(get, set, units, ctype)                                    \
    { .GET = (void (*)(void))get, .SET = (void (*)(void))set,                      \
      .SIG = (MotVar_Sig_T)(SIG_OF(ctype) +                                        \
             ASSERT_SIG(ctype, typeof(get((const Motor_Context_T *)0)))) }

typedef enum Motor_Var_UserOut
{
    MOTOR_VAR_SPEED, MOTOR_VAR_I_PHASE, MOTOR_VAR_STATE, MOTOR_VAR_SUB_STATE,
    _MOTOR_VAR_USER_OUT_END,
}
Motor_Var_UserOut_T;

/* Designated: the index is the enum name, so row order carries no meaning. */
static const VField_T USER_OUT_FIELDS[_MOTOR_VAR_USER_OUT_END] =
{
    [MOTOR_VAR_SPEED]     = VAR_FIELD_STATE(Motor_User_GetSpeed_Fract16, NULL, Rpm,  accum32_t),
    [MOTOR_VAR_I_PHASE]   = VAR_FIELD_STATE(Motor_GetIPhase_Fract16,     NULL, Amps, fract16_t),
    [MOTOR_VAR_STATE]     = VAR_FIELD_STATE(Motor_GetStateId,            NULL, None, Motor_StateId_T),
    [MOTOR_VAR_SUB_STATE] = VAR_FIELD_STATE(Motor_GetPathId,             NULL, None, state_t),
};

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
    [MotVar_Sig_T] encodes the return type only; the parameter type lives in
    the macro, which is why mixing shapes in one table cannot compile.
*/
/******************************************************************************/
typedef int32_t (*GetAdapter_Dev_T)(void (*)(void), const Motor_T *);

#define GET_ADAPTER_DEV(name, ctype)                                 \
    static int32_t name(void (*fn)(void), const Motor_T * p_dev)     \
        { return (int32_t)((ctype (*)(const Motor_T *))fn)(p_dev); }

GET_ADAPTER_DEV(GetDev_I16, int16_t)    GET_ADAPTER_DEV(GetDev_I32, int32_t)

static const GetAdapter_Dev_T GET_ADAPTERS_DEV[_SIG_END] =
{
    [SIG_I16] = GetDev_I16, [SIG_I32] = GetDev_I32,   /* sparse: only what this group uses */
};

#define VAR_FIELD_DEV(get, set, units, ctype)                                      \
    { .GET = (void (*)(void))get, .SET = (void (*)(void))set,                      \
      .SIG = (MotVar_Sig_T)(SIG_OF(ctype) +                                        \
             ASSERT_SIG(ctype, typeof(get((const Motor_T *)0)))) }

typedef enum Motor_Var_DevOut { MOTOR_VAR_I_BUS, _MOTOR_VAR_DEV_OUT_END } Motor_Var_DevOut_T;

static const VField_T DEV_OUT_FIELDS[_MOTOR_VAR_DEV_OUT_END] =
{
    [MOTOR_VAR_I_BUS] = VAR_FIELD_DEV(Motor_GetIBus_Fract16, NULL, Amps, fract16_t),
};

int32_t Motor_Var_DevOut_Get(const Motor_T * p_dev, Motor_Var_DevOut_T id)
{
    if ((size_t)id >= _MOTOR_VAR_DEV_OUT_END) { return 0; }
    VField_T field = DEV_OUT_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS_DEV[field.SIG](field.GET, p_dev) : 0;
}

/******************************************************************************/
/*
    (3) No context - board constants

    Adapter takes no context. Nothing else changes.
*/
/******************************************************************************/
typedef int32_t (*GetAdapter_Const_T)(void (*)(void));

#define GET_ADAPTER_CONST(name, ctype)                       \
    static int32_t name(void (*fn)(void))                    \
        { return (int32_t)((ctype (*)(void))fn)(); }

GET_ADAPTER_CONST(GetConst_I16, int16_t)    GET_ADAPTER_CONST(GetConst_U16, uint16_t)

static const GetAdapter_Const_T GET_ADAPTERS_CONST[_SIG_END] =
{
    [SIG_I16] = GetConst_I16, [SIG_U16] = GetConst_U16,
};

#define VAR_FIELD_CONST(get, units, ctype)                                         \
    { .GET = (void (*)(void))get, .SET = NULL,                                     \
      .SIG = (MotVar_Sig_T)(SIG_OF(ctype) + ASSERT_SIG(ctype, typeof(get()))) }

typedef enum Motor_Var_Board
{
    MOTOR_VAR_BOARD_V_RATED, MOTOR_VAR_BOARD_I_RATED_PEAK, MOTOR_VAR_BOARD_V_MAX_VOLTS,
    _MOTOR_VAR_BOARD_END,
}
Motor_Var_Board_T;

static const VField_T BOARD_FIELDS[_MOTOR_VAR_BOARD_END] =
{
    [MOTOR_VAR_BOARD_V_RATED]      = VAR_FIELD_CONST(Phase_Calibration_GetVRated_Fract16,    Volts, fract16_t),
    [MOTOR_VAR_BOARD_I_RATED_PEAK] = VAR_FIELD_CONST(Phase_Calibration_GetIRatedPeak_Fract16, Amps, fract16_t),
    [MOTOR_VAR_BOARD_V_MAX_VOLTS]  = VAR_FIELD_CONST(Phase_Calibration_GetVMaxVolts,         Volts, uint16_t),
};

int32_t Motor_Var_Board_Get(Motor_Var_Board_T id)
{
    if ((size_t)id >= _MOTOR_VAR_BOARD_END) { return 0; }
    VField_T field = BOARD_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS_CONST[field.SIG](field.GET) : 0;
}

/******************************************************************************/
/*
    (4) Setters - already uniform

    Every setter is void(Motor_Context_T *, int), so there is one signature and
    therefore no adapter table and no [MotVar_Sig_T] on the write path. The
    static_assert is what keeps that true: add a narrower setter and it fails
    here rather than silently widening at the call.
*/
/******************************************************************************/
typedef void (*Set_State_T)(Motor_Context_T *, int);

#define VAR_FIELD_SETPOINT(set, units)                                             \
    { .GET = NULL, .SET = (void (*)(void))set, .SIG = SIG_I32 +                    \
      0 * sizeof(struct { static_assert(_Generic(set, Set_State_T : 1, default : 0), \
                          "setpoint setters must be void(Motor_Context_T *, int)"); int _; }) }

typedef enum Motor_Var_Setpoint
{
    MOTOR_VAR_SETPOINT_SPEED, MOTOR_VAR_SETPOINT_TORQUE, MOTOR_VAR_SETPOINT_I,
    _MOTOR_VAR_SETPOINT_END,
}
Motor_Var_Setpoint_T;

static const VField_T SETPOINT_FIELDS[_MOTOR_VAR_SETPOINT_END] =
{
    [MOTOR_VAR_SETPOINT_SPEED]  = VAR_FIELD_SETPOINT(Motor_SetSpeedCmd,  Rpm),
    [MOTOR_VAR_SETPOINT_TORQUE] = VAR_FIELD_SETPOINT(Motor_SetTorqueCmd, Amps),
    [MOTOR_VAR_SETPOINT_I]      = VAR_FIELD_SETPOINT(Motor_SetICmd,      Amps),
};

void Motor_Var_Setpoint_Set(Motor_Context_T * p_motor, Motor_Var_Setpoint_T id, int32_t value)
{
    if ((size_t)id >= _MOTOR_VAR_SETPOINT_END) { return; }
    VField_T field = SETPOINT_FIELDS[id];
    if (field.SET != NULL) { ((Set_State_T)field.SET)(p_motor, (int)value); }
}

/******************************************************************************/
/*
    (5) Union return

    [Motor_FaultFlags_T] is a union, so IS_SIGNED cannot be evaluated on it.
    A .Value accessor is not a pass-through - it selects the wire member - so
    it earns its place where a plain forwarder would not.
*/
/******************************************************************************/
static inline uint8_t Motor_GetFaultFlags_Value(const Motor_Context_T * p) { return Motor_GetFaultFlags(p).Value; }

typedef enum Motor_Var_Status { MOTOR_VAR_FAULT_FLAGS, _MOTOR_VAR_STATUS_END } Motor_Var_Status_T;

static const VField_T STATUS_FIELDS[_MOTOR_VAR_STATUS_END] =
{
    [MOTOR_VAR_FAULT_FLAGS] = VAR_FIELD_STATE(Motor_GetFaultFlags_Value, NULL, None, uint8_t),
};

int32_t Motor_Var_Status_Get(const Motor_Context_T * p_motor, Motor_Var_Status_T id)
{
    if ((size_t)id >= _MOTOR_VAR_STATUS_END) { return 0; }
    VField_T field = STATUS_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS_STATE[field.SIG](field.GET, p_motor) : 0;
}
