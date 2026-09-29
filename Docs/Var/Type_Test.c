
#include "Motor/Motor/Motor_Var.h"

#include "Motor/Motor/Analog/Motor_Analog.h" /* for calibration cmd */
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor.h"


typedef const struct VField
{
    void (*GET)(void);
    void (*SET)(void);
    int SIG_TYPE; /* calling convention at runtime */
}
VField_T;

typedef const struct VarGroup
{
    uint16_t Index;
    int32_t (*Get)(const void *, int id);
    void (*Set)(const void *, int id, int32_t);
}
VarGroup_T;

// unlike the static polymorphism case, runtime mapped indexs need type to materialize through an enum selection.
// in the static polymorphism case, compiler optimization may strip away the widened wrapper. materializing select through an enum type id is not necessary.
// here widening to a unifrom function signature int32_t get(void*) means deriving return type through manual assignment.
//
// into a call site (name it, typeof works, hand assign switch indexing)
// into data (a tag — indirect step)
// out of existence (normalize every signature, hand assign type)

/*
    A signature is (width, signedness)
    Preserves the calling convention. not the format annotation.
    Deriving it from _Generic
*/
typedef enum MotVar_Sig { SIG_I8, SIG_U8, SIG_I16, SIG_U16, SIG_I32, SIG_U32, _SIG_END } MotVar_Sig_T;



#if   defined(MOTOR_VAR_META_CHECK)   /* CI only - typos become errors */
#define MOTOR_VAR_META(type, instances, accessGroup, units, description)  , sizeof(type), units, accessGroup
#elif defined(PARSER_SIDE)            /* exporter */
#define MOTOR_VAR_META(type, instances, accessGroup, units, description)  , #type, #instances, #accessGroup, #units, description
#else                                 /* firmware - vanishes */
#define MOTOR_VAR_META(type, instances, accessGroup, units, description)
#endif


/******************************************************************************/
/*
    simplified representation
*/
/******************************************************************************/
/* This list each entry describes one variable */
static const VField_T  MOTOR_USER_OUT_VARS[] =
{
    [MOTOR_VAR_SPEED]       = { Motor_User_GetSpeed_Fract16, NULL, MOTOR_VAR_FIELD_META(Rpm, fract16_t, ) },
    [MOTOR_VAR_I_PHASE]     = { Motor_GetIPhase_Fract16, NULL, MOTOR_VAR_FIELD_META(Rpm, fract16_t) },
    [MOTOR_VAR_V_PHASE]     = { Motor_GetVPhase_Fract16, NULL, MOTOR_VAR_FIELD_META(Rpm, fract16_t) },
    [MOTOR_VAR_STATE]       = { Motor_GetStateId, NULL, MOTOR_VAR_FIELD_META(Rpm, Motor_StateId_T) },
    [MOTOR_VAR_SUB_STATE]   = { Motor_GetPathId, NULL, MOTOR_VAR_FIELD_META(Rpm, Motor_StateId_T) }
};


#define MOTOR_VAR_OBJ_META(...)

/* This list each entry describes the Object Groups or struct,  */
static const VarGroup_T MOTOR_VAR_GROUPS[] =
{
    //Motor_VarType_Base_T
    [MOTOR_VAR_TYPE_USER_OUT]       = {  &MOTOR_USER_OUT_VARS[0],           MOTOR_VAR_OBJ_META(Motor_Var_UserOut_T, Motor_T    ) },
    [MOTOR_VAR_TYPE_USER_OUT]       = { _Motor_Var_UserOut_Get,     NULL,   MOTOR_VAR_OBJ_META(Motor_Var_UserOut_T, Motor_T    ) },

    [MOTOR_VAR_TYPE_USER_CONTROL]   = { _Motor_Var_UserControl_Get, NULL,   MOTOR_VAR_OBJ_META(Motor_Var_UserControl_T, Motor_T) },
    [MOTOR_VAR_TYPE_USER_CONTROL]   = { &MOTOR_USER_CONTROL_VARS[0],        MOTOR_VAR_OBJ_META(Motor_Var_UserControl_T, Motor_T) },
    // MOTOR_VAR_TYPE_USER_SETPOINT, /* Setpoint Input only */
    // MOTOR_VAR_TYPE_STATE_CMD, /* Non polling Cmds */
};

/*  Config_T can use field offset. */

/******************************************************************************/
/*
    Per variable tags
*/
/******************************************************************************/
/*
    compiler assign index,
    static wrapper alias as name key.
        widened general format -> handwritten format annotation. no extra function sig enum, uniform signature fulfills interface.
            wrapper can double as name transcription.
        original format -> annotation export by dwarf file. materialize enum to encode calling convention, auto through _Generic
            pays through switch on sig enum
*/

static fract16_t Motor_User_GetSpeed(const Motor_Context_T * p_state) { return p_state->SensorState.Speed_Fract16; }
// static fract16_t MotorSpeed_Fract16(const Motor_Context_T * p_state) { return Motor_User_GetSpeed(p_state); }
static int MotorSpeed(const Motor_Context_T * p_state) { return Motor_User_GetSpeed(p_state); }

#define VAR_META_EXPORT(...)

static const VField_T  MOTOR_USER_OUT_VARS[] =
{
    { Motor_User_GetSpeed, NULL,    VAR_META_EXPORT(Rpm) }, /* VAR_META_EXPORT  MotVar_Sig_T from _Generic(typeof()) */
    { MotorSpeed, NULL,             VAR_META_EXPORT(Rpm, fract16_t, float) }, /* hand written format annotation */
    { MotorIPhase_Fract16, NULL, VAR_META_EXPORT(Rpm, fract16_t, float) },
    { MotorVPhase_Fract16, NULL, VAR_META_EXPORT(Rpm, fract16_t) },
    { MotorStateId, NULL, VAR_META_EXPORT(Rpm, Motor_StateId_T) },
    { MotorPathId, NULL, VAR_META_EXPORT(Rpm, Motor_StateId_T) }
};

int _Motor_Var_UserOut_Get(Motor_Context_T * p_state, Motor_Var_UserOut_T varId)
{
    return MOTOR_USER_OUT_VARS[varId].GET(p_state);
}

/*
    switch
    absorbs the calling convention

    requires explicitly defined enum index
    tag on enum, when there is no table.

    adding a id, requires updating both the enum and the switch statement.
*/
int _Motor_Var_UserOut_Get(const Motor_Context_T * p_state, Motor_Var_UserOut_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED:       value = Motor_User_GetSpeed_Fract16(p_state);           break;
        case MOTOR_VAR_I_PHASE:     value = Motor_GetIPhase_Fract16(p_state);               break;
        case MOTOR_VAR_V_PHASE:     value = Motor_GetVPhase_Fract16(p_state);               break;
        case MOTOR_VAR_STATE:       value = Motor_GetStateId(p_state);                      break;
        case MOTOR_VAR_SUB_STATE:   value = Motor_GetPathId(p_state);                       break;
        case MOTOR_VAR_FAULT_FLAGS: value = Motor_GetFaultFlags(p_state).Value;             break;
        default: break;
    }
    return value;
}

#define VAR_ID_TAG_EXPORT(name, ...)  name

typedef enum Motor_Var_UserOut
{
    VAR_ID_TAG_EXPORT(MOTOR_VAR_SPEED, RPM, fract16_t, float),    /* User Direction */
    MOTOR_VAR_I_PHASE,
    MOTOR_VAR_V_PHASE,
    MOTOR_VAR_STATE,
    MOTOR_VAR_SUB_STATE,
    MOTOR_VAR_FAULT_FLAGS,
}
Motor_Var_UserOut_T;



/*
    Un-materialize Meta struct pattern
    .def file style with individual def to avoid #undef

    same as the prior cases + optional enum def and table def to unify to 1 list.
*/
#define MOTOR_VAR_META_STRUCT(get, set, units, ...)  get, set, units, type, typeof(get)

#define MOTOR_VAR_SPEED_META    MOTOR_VAR_META_STRUCT(Motor_User_GetSpeed_Fract16, NULL, Rpm, )
// #define MOTOR_VAR_SPEED_META    MOTOR_VAR_META_STRUCT(MOTOR_VAR_SPEED, Motor_User_GetSpeed_Fract16, NULL, Rpm, )
#define MOTOR_VAR_I_META        MOTOR_VAR_META_STRUCT(Motor_GetIPhase_Fract16, NULL, Rpm, )
#define MOTOR_VAR_V_META        MOTOR_VAR_META_STRUCT(Motor_GetVPhase_Fract16, NULL, Rpm, )

#define MOTOR_USER_OUT_LIST(X) /* id, units, C type */ \
    X(MOTOR_VAR_SPEED_META)      \
    X(MOTOR_VAR_V_META)      \
    X(MOTOR_VAR_I_META)      \


#define _MOTOR_VAR_FN(a, b, ...)  { .GET = a, .SET = b }
// #define MOTOR_VAR_FN(args)  _MOTOR_VAR_FN(args)
#define MOTOR_VAR_FN(...)  _MOTOR_VAR_FN(__VA_ARGS__)

#define _MOTOR_VAR_FN_TYPE(get, set, units, type, fntype, ...) fntype
#define MOTOR_VAR_FN_TYPE(...) _MOTOR_VAR_FN_TYPE(__VA_ARGS__)

#define _MOTOR_VAR_GET(get, set, units, type, fntype, ...) get
#define MOTOR_VAR_GET(args) _MOTOR_VAR_GET(args)

// #define MOTOR_VAR_TYPED_GET(...)  _MOTOR_VAR_TYPED_GET(__VA_ARGS__)

/* table maps index */
static const VField_T  MOTOR_USER_TEST_VARS[] =
{
    MOTOR_VAR_FN(MOTOR_VAR_SPEED_META) , /* expand to fn only */
    MOTOR_VAR_FN(MOTOR_VAR_I_META) , /* expand to fn only */
    MOTOR_VAR_FN(MOTOR_VAR_V_META) , /* expand to fn only */
};

// #define CALL(fn, ...) ((typeof(fn)*)fn)(__VA_ARGS__)
#define CALL(fn, ...) (fn(__VA_ARGS__))

/* Table includes sig type.  */
int _Motor_Var_UserOut_Get(const Motor_Context_T *p_motor, int index)
{
   void (*get)(void) = MOTOR_USER_TEST_VARS[index].GET ;
   MotVar_Sig_T sig = MOTOR_USER_TEST_VARS[index].SIG_TYPE;
   // call get with sig type.
   // or MOTOR_VAR_META_STRUCT includes the wrapper
}

/* switch  manually map index   */
int _Motor_Var_UserOut_Get(const Motor_Context_T *p_motor, int index)
{
   switch (index)
   {
       case MOTOR_VAR_SPEED: return MOTOR_VAR_GET(MOTOR_VAR_SPEED_META)(p_motor);

    //    case MOTOR_VAR_ID(MOTOR_VAR_SPEED_META): return MOTOR_VAR_GET(MOTOR_VAR_SPEED_META)(p_motor); //enum from the same list
    // optionally macro produces the list with "case"
   }
}



/*
    .def file, def / def style
    one list
*/
#ifndef MOTOR_VAR_DEF
#define MOTOR_VAR_DEF(get, set, units, type, ...)
#endif

MOTOR_VAR_DEF(Motor_User_GetSpeed_Fract16, NULL, Rpm, fract16_t,)
MOTOR_VAR_DEF(Motor_GetIPhase_Fract16, NULL, Rpm, fract16_t) ,
MOTOR_VAR_DEF(Motor_GetVPhase_Fract16, NULL, Rpm, fract16_t) ,
MOTOR_VAR_DEF(Motor_GetStateId, NULL, Rpm, Motor_StateId_T) ,
MOTOR_VAR_DEF(Motor_GetPathId, NULL, Rpm, Motor_StateId_T)
#undef MOTOR_VAR_DEF

#define MOTOR_VAR_DEF
#define MOTOR_VAR_DEF(get, set,   ...)   { .GET = get, .SET = set }
#undef MOTOR_VAR_DEF




/*
    group sig
*/

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
static const VField_T VARS[] =
{
    VAR_FIELD(Motor_User_GetSpeed_Fract16, NULL, Rpm ),
    VAR_FIELD(Motor_GetIPhase_Fract16,     NULL, Amps ),
    VAR_FIELD(Motor_GetStateId,            NULL, None, Motor_StateId_T ),
    VAR_FIELD(Motor_GetPathId,             NULL, None ),
};
static_assert(sizeof(VARS)/sizeof(VARS[0]) == _MOTOR_VAR_USER_OUT_END, "count");

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

    [0] "Motor_User_GetSpeed_Fract16" , "Rpm" , "accum32_t" ,       <- dwarf file
    [1] "Motor_GetIPhase_Fract16" , "Amps" , "fract16_t" ,
    [2] "Motor_GetStateId", "None" , "Motor_StateId_T" ,
*/


/*
    .def file style e.g
*/
#define MOTOR_USER_OUT_IDS(X) /* id, units, C type */ \
    X(MOTOR_VAR_SPEED,     UNITS_RPM,  fract16_t)      \
    X(MOTOR_VAR_I_PHASE,   UNITS_AMPS, fract16_t)      \
    X(MOTOR_VAR_STATE,     UNITS_NONE, Motor_StateId_T)

#define _ID(id, units, type)  id,
typedef enum Motor_Var_UserOut { MOTOR_USER_OUT_IDS(_ID) _MOTOR_VAR_USER_OUT_END } Motor_Var_UserOut_T;


the exporter is the preprocessor.

$ arm-none-eabi-gcc -E -DEXPORT enum.h | grep @@
"MOTOR_VAR_SPEED" , "Rpm" , "fract16_t" ,
"MOTOR_VAR_STATE" , "None" , "Motor_StateId_T" ,


/* MotorUserOut.def — NO include guard: this file is meant to be included many times */
#ifndef MOTOR_VAR
#define MOTOR_VAR(id, fn, units, ctype)      /* default: consumers define only what they need */
#endif

/*          id                  accessor                     units        C type          */
MOTOR_VAR(MOTOR_VAR_SPEED,    Motor_User_GetSpeed_Fract16,  UNITS_RPM,   fract16_t)
MOTOR_VAR(MOTOR_VAR_I_PHASE,  Motor_GetIPhase_Fract16,      UNITS_AMPS,  fract16_t)
MOTOR_VAR(MOTOR_VAR_STATE,    Motor_GetStateId,             UNITS_NONE,  Motor_StateId_T)
MOTOR_VAR(MOTOR_VAR_IS_FAULT, Motor_IsFault,                UNITS_NONE,  bool)

#undef MOTOR_VAR                              /* self-cleaning */

```
/* 1. the id enum */
#define MOTOR_VAR(id, fn, units, ctype)   id,
typedef enum Motor_Var_UserOut {
#include "MotorUserOut.def"
    _MOTOR_VAR_USER_OUT_END } Motor_Var_UserOut_T;

/* 2. checks - emits nothing, proves the ctype column isn't lying */
#define MOTOR_VAR(id, fn, units, ctype)                                     \
    static_assert(sizeof(ctype) <= sizeof(int32_t), #id " wider than wire"); \
    static_assert(_Generic(fn, ctype (*)(const Motor_T *): 1, default: 0),   \
                  #id ": accessor return type disagrees with declared ctype");
#include "MotorUserOut.def"

/* 3. the dispatch */
#define MOTOR_VAR(id, fn, units, ctype)   case id: return (int32_t)fn(p_motor);
int32_t MotorUserOut_Get(const Motor_T * p_motor, int id)
{
    switch (id) {
#include "MotorUserOut.def"
    default: return 0; }
}

#define SIG_OF(ctype) _Generic((ctype){0}, fract16_t: SIG_I16, Motor_StateId_T: SIG_U8, bool: SIG_BOOL)

#define MOTOR_VAR(id, fn, units, ctype)   [id] = { (void(*)(void))fn, SIG_OF(ctype) },
static const VField_T USER_OUT_VARS[] = {
#include "MotorUserOut.def"
};

switch (e.KIND) {
    case SIG_I16: return (int32_t)((fract16_t(*)(const Motor_T *))e.FN)(p_motor);
    case SIG_U8:  return (int32_t)((Motor_StateId_T(*)(const Motor_T *))e.FN)(p_motor);
    ...
}
```
/* Thunk version */
#define _THUNK(id, get, set, units)                                            \
    static int32_t _get_##id(const Motor_Context_T * p) { return (int32_t)get(p); }
ROW_LIST(_THUNK)

typedef int32_t (*Get_T)(const Motor_Context_T *);
#define _TROW(id, get, set, units)  [id] = _get_##id,
static const Get_T GETTERS[_ROW_END] = { ROW_LIST(_TROW) };