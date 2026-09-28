
#include "Motor/Motor/Motor_Var.h"

#include "Motor/Motor/Analog/Motor_Analog.h" /* for calibration cmd */
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor.h"


// typedef struct VarGroup
// {
//     uint16_t Index;
//     int32_t (*Get)(const void *, int id);
//     void (*Set)(const void *, int id, int32_t);
// }
// VarGroup_T;

// #if   defined(MOTOR_VAR_META_CHECK)   /* CI only - typos become errors */
// #define MOTOR_VAR_META(type, instances, accessGroup, units, description)  , sizeof(type), units, accessGroup
// #elif defined(PARSER_SIDE)            /* exporter */
// #define MOTOR_VAR_META(type, instances, accessGroup, units, description)  , #type, #instances, #accessGroup, #units, description
// #else                                 /* firmware - vanishes */
// #define MOTOR_VAR_META(type, instances, accessGroup, units, description)
// #endif




#define MOTOR_VAR_META_STRUCT(get, set, units, ...)  get, set, units, type, typeof(get)
// MOTOR_VAR_DEF(Motor_User_GetSpeed_Fract16, NULL, Rpm, fract16_t,)
#define MOTOR_VAR_SPEED_META MOTOR_VAR_META_STRUCT(Motor_User_GetSpeed_Fract16, NULL, Rpm, )
#define MOTOR_VAR_I_META    MOTOR_VAR_META_STRUCT(Motor_GetIPhase_Fract16, NULL, Rpm, )
#define MOTOR_VAR_V_META    MOTOR_VAR_META_STRUCT(Motor_GetVPhase_Fract16, NULL, Rpm, )
// #undef MOTOR_VAR_DEF

// #define _MOTOR_VAR_EXPAND(...)  __VA_ARGS__

#define _MOTOR_VAR_FN(a, b, ...)  { .GET = a, .SET = b }
// #define MOTOR_VAR_FN(args)  _MOTOR_VAR_FN(args)
#define MOTOR_VAR_FN(...)  _MOTOR_VAR_FN(__VA_ARGS__)

#define _MOTOR_VAR_FN_TYPE(get, set, units, type, fntype, ...) fntype
#define MOTOR_VAR_FN_TYPE(...) _MOTOR_VAR_FN_TYPE(__VA_ARGS__)

#define _MOTOR_VAR_GET(get, set, units, type, fntype, ...) get
#define MOTOR_VAR_GET(args) _MOTOR_VAR_GET(args)

// #define MOTOR_VAR_TYPED_GET(...)  _MOTOR_VAR_TYPED_GET(__VA_ARGS__)


typedef const struct VField
{
    void (*GET)(void);
    void (*SET)(void);
}
VField_T;


static const VField_T  MOTOR_USER_TEST_VARS[] =
{
    MOTOR_VAR_FN(MOTOR_VAR_SPEED_META) , /* expand to fn only */
    MOTOR_VAR_FN(MOTOR_VAR_I_META) , /* expand to fn only */
    MOTOR_VAR_FN(MOTOR_VAR_V_META) , /* expand to fn only */
};

// #define CALL(fn, ...) ((typeof(fn)*)fn)(__VA_ARGS__)
#define CALL(fn, ...) (fn(__VA_ARGS__))

int CallTableTest(const Motor_Context_T *p_motor, int index)
{
//    void (*get)(void) = MOTOR_USER_TEST_VARS[index].GET ;
    // MOTOR_VAR_TYPED_GET(MOTOR_VAR_SPEED_META)

   switch (index)
   {
       case 0:
           typeof(MOTOR_VAR_GET(MOTOR_VAR_SPEED_META)) * fn = MOTOR_VAR_GET(MOTOR_VAR_SPEED_META);
           return fn(p_motor);
           return CALL(MOTOR_VAR_GET(MOTOR_VAR_SPEED_META), p_motor);
   }
}


/* This list each entry describes one variable */
// static const VField_T  MOTOR_USER_OUT_VARS[] =
// {
//     [MOTOR_VAR_SPEED]       = { Motor_User_GetSpeed_Fract16, NULL, MOTOR_VAR_FIELD_META(Rpm, fract16_t, ) },
//     [MOTOR_VAR_I_PHASE]     = { Motor_GetIPhase_Fract16, NULL, MOTOR_VAR_FIELD_META(Rpm, fract16_t) },
//     [MOTOR_VAR_V_PHASE]     = { Motor_GetVPhase_Fract16, NULL, MOTOR_VAR_FIELD_META(Rpm, fract16_t) },
//     [MOTOR_VAR_STATE]       = { Motor_GetStateId, NULL, MOTOR_VAR_FIELD_META(Rpm, Motor_StateId_T) },
//     [MOTOR_VAR_SUB_STATE]   = { Motor_GetPathId, NULL, MOTOR_VAR_FIELD_META(Rpm, Motor_StateId_T) }
// };


#define MOTOR_VAR_FIELD_META(get, set,   ...)   { .GET = get, .SET = set }

static const VField_T  MOTOR_USER_TEST_VARS[] =
{
   MOTOR_VAR_FIELD_META(Motor_User_GetSpeed_Fract16, NULL, Rpm, fract16_t,) ,
   MOTOR_VAR_FIELD_META(Motor_GetIPhase_Fract16, NULL, Rpm, fract16_t) ,
   MOTOR_VAR_FIELD_META(Motor_GetVPhase_Fract16, NULL, Rpm, fract16_t) ,
   MOTOR_VAR_FIELD_META(Motor_GetStateId, NULL, Rpm, Motor_StateId_T) ,
   MOTOR_VAR_FIELD_META(Motor_GetPathId, NULL, Rpm, Motor_StateId_T)
};

int CallTableTest(const Motor_Context_T *p_motor, int index)
{
    void (*get)(void) = MOTOR_USER_OUT_VARS[index].GET;
    switch (index)
    {
        case INDEX_OF_FN(MOTOR_USER_TEST_VARS, Motor_User_GetSpeed_Fract16):
            {
                typeof(Motor_User_GetSpeed_Fract16) *fn = (typeof(Motor_User_GetSpeed_Fract16)*)get;
                fn(p_motor);

                break;
            }
    }
}


#define MOTOR_VAR_OBJ_META(...)
/* This list each entry describes the Object Groups or struct,  */
static const VarGroup_T MOTOR_VAR_GROUPS[] =
{
    //Motor_VarType_Base_T
    [MOTOR_VAR_TYPE_USER_OUT]       = { _Motor_Var_UserOut_Get,     NULL, &MOTOR_USER_OUT_VARS[0], MOTOR_VAR_OBJ_META(Motor_Var_UserOut_T, Motor_T    ) },
    [MOTOR_VAR_TYPE_USER_CONTROL]   = { _Motor_Var_UserControl_Get, NULL, &MOTOR_USER_CONTROL_VARS[0], MOTOR_VAR_OBJ_META(Motor_Var_UserControl_T, Motor_T) },
    // MOTOR_VAR_TYPE_USER_SETPOINT, /* Setpoint Input only */
    // MOTOR_VAR_TYPE_STATE_CMD, /* Non polling Cmds */
    // MOTOR_VAR_TYPE_OPEN_LOOP_CMD,
    // MOTOR_VAR_TYPE_CALIBRATION_CMD,
    // MOTOR_VAR_TYPE_CMD_RESV,
    // MOTOR_VAR_TYPE_CONFIG_CALIBRATION,
    // MOTOR_VAR_TYPE_CONFIG_ACTUATION,
    // MOTOR_VAR_TYPE_CONFIG_PID,
    // MOTOR_VAR_TYPE_CONFIG_DEBUG,
    // MOTOR_VAR_TYPE_CONFIG_RESV,
};


// unlike the static polymorphism case, runtime mapped indexs need type to materialize through an enum selection.
// rather compile time optimization. widening to a unifrom function signature int32_t get(void*) means deriving return type through manual assignment.
//
// into a call site (name it, typeof works, hand assign switch indexing)
// into data (a tag — indirect step)
// out of existence (normalize every signature, hand assign type)

/*  A signature is (width, signedness) — NOT the C type name. Deriving it from
    the type name breaks: on a target where int32_t is `int`, an enum whose
    underlying type is also `int` makes two _Generic arms collide.            */
typedef enum MotVar_Sig { SIG_I8, SIG_U8, SIG_I16, SIG_U16, SIG_I32, SIG_U32, _SIG_END } MotVar_Sig_T;

#define IS_SIGNED(ctype)  ((ctype)-1 < (ctype)0)
#define SIG_OF(ctype)                                                  \
    ((sizeof(ctype) == 1u) ? (IS_SIGNED(ctype) ? SIG_I8  : SIG_U8)  :  \
     (sizeof(ctype) == 2u) ? (IS_SIGNED(ctype) ? SIG_I16 : SIG_U16) :  \
                             (IS_SIGNED(ctype) ? SIG_I32 : SIG_U32))

typedef int32_t (*MotVar_GetAdapter_T)(void (*)(void), const Motor_Context_T *);

/*  The only casts in the design — one per signature, not per variable.
    void(*)(void) round-trip back to a compatible type is defined by 6.3.2.3p8. */
#define GET_ADAPTER(name, ctype)                                           \
    static int32_t name(void (*fn)(void), const Motor_Context_T * p_motor) \
        { return (int32_t)((ctype (*)(const Motor_Context_T *))fn)(p_motor); }

GET_ADAPTER(GetAdapter_I8,  int8_t)     GET_ADAPTER(GetAdapter_U8,  uint8_t)
GET_ADAPTER(GetAdapter_I16, int16_t)    GET_ADAPTER(GetAdapter_U16, uint16_t)
GET_ADAPTER(GetAdapter_I32, int32_t)    GET_ADAPTER(GetAdapter_U32, uint32_t)

static const MotVar_GetAdapter_T GET_ADAPTERS[_SIG_END] =
{
    [SIG_I8]  = GetAdapter_I8,  [SIG_U8]  = GetAdapter_U8,
    [SIG_I16] = GetAdapter_I16, [SIG_U16] = GetAdapter_U16,
    [SIG_I32] = GetAdapter_I32, [SIG_U32] = GetAdapter_U32,
};



typedef const struct VField
{
    void (*GET)(void);
    void (*SET)(void);
    MotVar_Sig_T SIG;
}
VField_T;

#define SIG_(ctype) \
    _Generic((ctype)0, \
             int8_t:  SIG_I8,  uint8_t:  SIG_U8, \
             int16_t: SIG_I16, uint16_t: SIG_U16, \
             int32_t: SIG_I32, uint32_t: SIG_U32)

#define VAR_FIELD(get, set, units, ...) { .GET = (void (*)(void))get, .SET = (void (*)(void))set, .SIG = SIG_(typeof(get())) }

/* ---- (A) POSITIONAL: the index is row order. Count still matches. -------- */
static const VField_T VARS_POSITIONAL[] =
{
    VAR_FIELD(Motor_User_GetSpeed_Fract16, NULL, Rpm ),
    VAR_FIELD(Motor_GetIPhase_Fract16,     NULL, Amps ),

    VAR_FIELD(Motor_GetStateId,            NULL, None, Motor_StateId_T ),
    VAR_FIELD(Motor_GetPathId,             NULL, None ),
};
static_assert(sizeof(VARS_POSITIONAL)/sizeof(VARS_POSITIONAL[0]) == _MOTOR_VAR_USER_OUT_END, "count");

int32_t Motor_Var_UserOut_Get(const Motor_Context_T * p_motor, Motor_Var_UserOut_T id)
{
    if ((size_t)id >= _MOTOR_VAR_USER_OUT_END) { return 0; }
    MotVar_Field_T field = USER_OUT_FIELDS[id];
    return (field.GET != NULL) ? GET_ADAPTERS[field.SIG](field.GET, p_motor) : 0;
}



/* ---- (B) DESIGNATED: the index is the enum name. Row order irrelevant. --- */
// static const VField_T VARS_DESIGNATED[_MOTOR_VAR_USER_OUT_END] =
// {
// #ifndef SWAP
//     [MOTOR_VAR_SPEED]     = FIELD(Motor_User_GetSpeed_Fract16, NULL, Rpm,  accum32_t),
//     [MOTOR_VAR_I_PHASE]   = FIELD(Motor_GetIPhase_Fract16,     NULL, Amps, fract16_t),
// #else                                    /* same transposition */
//     [MOTOR_VAR_I_PHASE]   = FIELD(Motor_GetIPhase_Fract16,     NULL, Amps, fract16_t),
//     [MOTOR_VAR_SPEED]     = FIELD(Motor_User_GetSpeed_Fract16, NULL, Rpm,  accum32_t),
// #endif
//     [MOTOR_VAR_STATE]     = FIELD(Motor_GetStateId,            NULL, None, Motor_StateId_T),
//     [MOTOR_VAR_SUB_STATE] = FIELD(Motor_GetPathId,             NULL, None, state_t),
// #ifdef DUPLICATE
//     [MOTOR_VAR_I_PHASE]   = FIELD(Motor_GetIBus_Fract16,       NULL, Amps, fract16_t),
// #endif
// };


/*

    A2L files are usually generated from the build artifact, not hand-written.
    Which answers "compiler outputs the function name with its index" — it already does. From your own toolchain, on a test object:
    $ arm-none-eabi-gcc -E -DEXPORT enum.h | grep @@

    [0] "Motor_User_GetSpeed_Fract16" , "Rpm" , "accum32_t" ,       <-
    [1] "Motor_GetIPhase_Fract16" , "Amps" , "fract16_t" ,
    [2] "Motor_GetStateId", "None" , "Motor_StateId_T" ,

    // [0] "Motor_User_GetSpeed_Fract16" , "Rpm" , "accum32_t" ,       <- accum32_t resolve by the function pointers return type
*/
