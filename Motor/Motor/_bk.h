

/******************************************************************************/
/*
    Cw, Ccw use by Control loop
*/
/******************************************************************************/
// static void UpdateILimitsCcw(Motor_T * p_motor)
// // static void UpdateILimitsCcw(Motor_T * p_motor, fract16_t motoring, fract16_t generating)
// {
//     p_motor->ILimitCcw_Fract16 = p_motor->ILimitMotoring_Fract16  ;
//     p_motor->ILimitCw_Fract16 = (int32_t)0 - (p_motor->ILimitGenerating_Fract16  );
// }

// static void UpdateILimitsCw(Motor_T * p_motor)
// {
//     p_motor->ILimitCw_Fract16 = (int32_t)0 - (p_motor->ILimitMotoring_Fract16  );
//     p_motor->ILimitCcw_Fract16 = p_motor->ILimitGenerating_Fract16  ;
// }

// static void UpdateSpeedLimitsCcw(Motor_T * p_motor)
// {
//     int32_t ccw = (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16;
//     p_motor->SpeedLimitCcw_Fract16 = ccw  ;
//     p_motor->SpeedLimitCw_Fract16 = 0;
// }

// static void UpdateSpeedLimitsCw(Motor_T * p_motor)
// {
//     int32_t cw = (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitReverse_Fract16 : p_motor->SpeedLimitForward_Fract16;
//     p_motor->SpeedLimitCcw_Fract16 = 0;
//     p_motor->SpeedLimitCw_Fract16 = (int16_t)0 - (cw  );
// }





// alternatively derived values as base
// static inline uint16_t Motor_User_GetSpeedLimitForward(const Motor_T * p_motor) { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Fract16 : (0 - p_motor->SpeedLimitCw_Fract16); }
// static inline uint16_t Motor_User_GetSpeedLimitReverse(const Motor_T * p_motor) { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? (0 - p_motor->SpeedLimitCw_Fract16) : p_motor->SpeedLimitCcw_Fract16; }
// static inline uint16_t Motor_User_GetILimitMotoring(const Motor_T * p_motor)    { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->ILimitCcw_Fract16 : (0 - p_motor->ILimitCw_Fract16); }
// static inline uint16_t Motor_User_GetILimitGenerating(const Motor_T * p_motor)  { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? (0 - p_motor->ILimitCw_Fract16) : p_motor->ILimitCcw_Fract16; }

// static inline int16_t Motor_User_SpeedCmdLimitOf(const Motor_T * p_motor, int32_t speed_Fract16)
// {
//     return math_clamp(speed_Fract16, 0, Motor_User_GetSpeedLimit(p_motor));
// };

// static inline int16_t Motor_User_ICmdLimitOf(const Motor_T * p_motor, int16_t i_Fract16)
// {
//     return math_clamp(i_Fract16, (int32_t)0 - Motor_User_GetILimitGenerating(p_motor), Motor_User_GetILimitMotoring(p_motor));
// };


// static inline int16_t Motor_SpeedReqLimitOf(const Motor_T * p_motor, int16_t req)   { return math_clamp(req, p_motor->SpeedLimitCw_Fract16, p_motor->SpeedLimitCcw_Fract16); };
// static inline int16_t Motor_IReqLimitOf(const Motor_T * p_motor, int16_t req)       { return math_clamp(req, p_motor->ILimitCw_Fract16, p_motor->ILimitCcw_Fract16); };


/* proportional on input */
// static inline int16_t  feedback_scalar(uint16_t feedback, uint16_t limit, int16_t input)
// static inline int16_t  feedback_scalar(int16_t input, uint16_t limit, uint16_t feedback)
// {
//     return (feedback > limit) ? (int32_t)input * limit / feedback : input;
// }

// // static inline int16_t limit_feedback_signed(int16_t input, int16_t lower, int16_t upper, int16_t feedback)
// // {
// //     int16_t result;
// //     if      (feedback < lower) { result = (int32_t)input * lower / feedback; }
// //     else if (feedback > upper) { result = (int32_t)input * upper / feedback; }
// //     else                       { result = input; }
// //     return result;
// // }

// /*
//     Alternative to enabling PID
//     on output as scalar
//     Req I, or V
// */
// static inline int16_t Motor_TorqueReqSpeedLimit(const Motor_T * p_motor, int16_t req)
// {
//     return feedback_scalar(req, Motor_GetSpeedLimit(p_motor), math_abs(p_motor->Speed_Fract16));
//     /* fract16_div always return positive < 1 */
//     // if      (p_motor->Speed_Fract16 < p_motor->SpeedLimitCw_Fract16)  { scalar = fract16_div(p_motor->SpeedLimitCw_Fract16, p_motor->Speed_Fract16); } /* Speed is more negative */
//     // else if (p_motor->Speed_Fract16 > p_motor->SpeedLimitCcw_Fract16) { scalar = fract16_div(p_motor->SpeedLimitCcw_Fract16, p_motor->Speed_Fract16); }
//     // return fract16_mul(req, scalar);
// }



/******************************************************************************/
/* Reset unit conversion structs */
/* keep for extneral, physical  */
/******************************************************************************/
// void Motor_ResetSpeedFeedbackRef(Motor_T * p_motor)
// {
//     p_motor->SpeedFeedbackRef_Rpm = Motor_GetSpeedVRef_Rpm(p_motor);
// }

// void Motor_ResetUnitsIabc(Motor_T * p_motor)
// {
//     Motor_ResetUnitsIa(p_motor);
//     Motor_ResetUnitsIb(p_motor);
//     Motor_ResetUnitsIc(p_motor);
// }

// void Motor_ResetUnitsIa(Motor_T * p_motor)
// {
//     Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIa, p_motor->Config.IaZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
// #ifdef CONFIG_MOTOR_I_SENSORS_INVERT
//     Linear_ADC_SetInverted(&p_motor->UnitsIa);
// #endif
// }

// void Motor_ResetUnitsIb(Motor_T * p_motor)
// {
//     Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIb, p_motor->Config.IbZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
// #ifdef CONFIG_MOTOR_I_SENSORS_INVERT
//     Linear_ADC_SetInverted(&p_motor->UnitsIb);
// #endif
// }

// void Motor_ResetUnitsIc(Motor_T * p_motor)
// {
//     Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIc, p_motor->Config.IcZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
// #ifdef CONFIG_MOTOR_I_SENSORS_INVERT
//     Linear_ADC_SetInverted(&p_motor->UnitsIc);
// #endif
// }

// void Motor_ResetUnitsVabc(Motor_T * p_motor)
// {
// #if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
//     Linear_ADC_Init_Scalar(&p_motor->UnitsVabc, 0U, MotorStatic.VSourceRef_Adcu);
// #else
//     (void)p_motor;
// #endif
// }