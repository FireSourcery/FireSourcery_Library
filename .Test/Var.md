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