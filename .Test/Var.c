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