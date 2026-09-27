
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






// static const VField_T  MOTOR_USER_TEST_VARS[] =
// {
//     MOTOR_VAR_FN(MOTOR_VAR_SPEED_META),

//     { MOTOR_VAR_FIELD_META(Motor_User_GetSpeed_Fract16, NULL, Rpm, fract16_t,) },
//     { MOTOR_VAR_FIELD_META(Motor_GetIPhase_Fract16, NULL, Rpm, fract16_t) },
//     { MOTOR_VAR_FIELD_META(Motor_GetVPhase_Fract16, NULL, Rpm, fract16_t) },
//     { MOTOR_VAR_FIELD_META(Motor_GetStateId, NULL, Rpm, Motor_StateId_T) },
//     { MOTOR_VAR_FIELD_META(Motor_GetPathId, NULL, Rpm, Motor_StateId_T) }
// };

int CallTableTest(const Motor_Context_T *p_motor, int index)
{
    void (*get)(void) = MOTOR_USER_OUT_VARS[index].GET;
    switch (index)
    {
        case INDEX_OF_FN(MOTOR_USER_TEST_VARS, Motor_User_GetSpeed_Fract16):
            {
                typeof(Motor_User_GetSpeed_Fract16) fn = (typeof(Motor_User_GetSpeed_Fract16))get;
                fn(p_motor);

                break;
            }
    }
}




// #define MOTOR_VAR_OBJ_META(...)
// /* This list each entry describes the Object Groups or struct,  */
// static const VarGroup_T MOTOR_VAR_GROUPS[] =
// {
//     //Motor_VarType_Base_T
//     [MOTOR_VAR_TYPE_USER_OUT]       = { _Motor_Var_UserOut_Get,     NULL, &MOTOR_USER_OUT_VARS[0], MOTOR_VAR_OBJ_META(Motor_Var_UserOut_T, Motor_T    ) },
//     [MOTOR_VAR_TYPE_USER_CONTROL]   = { _Motor_Var_UserControl_Get, NULL, &MOTOR_USER_CONTROL_VARS[0], MOTOR_VAR_OBJ_META(Motor_Var_UserControl_T, Motor_T) },
//     // MOTOR_VAR_TYPE_USER_SETPOINT, /* Setpoint Input only */
//     // MOTOR_VAR_TYPE_STATE_CMD, /* Non polling Cmds */
//     // MOTOR_VAR_TYPE_OPEN_LOOP_CMD,
//     // MOTOR_VAR_TYPE_CALIBRATION_CMD,
//     // MOTOR_VAR_TYPE_CMD_RESV,
//     // MOTOR_VAR_TYPE_CONFIG_CALIBRATION,
//     // MOTOR_VAR_TYPE_CONFIG_ACTUATION,
//     // MOTOR_VAR_TYPE_CONFIG_PID,
//     // MOTOR_VAR_TYPE_CONFIG_DEBUG,
//     // MOTOR_VAR_TYPE_CONFIG_RESV,
// };


