

/******************************************************************************/
/*
    Cw, Ccw use by Control loop
*/
/******************************************************************************/
// static void UpdateILimitsCcw(Motor_State_T * p_motor)
// // static void UpdateILimitsCcw(Motor_State_T * p_motor, fract16_t motoring, fract16_t generating)
// {
//     p_motor->ILimitCcw_Fract16 = p_motor->ILimitMotoring_Fract16  ;
//     p_motor->ILimitCw_Fract16 = (int32_t)0 - (p_motor->ILimitGenerating_Fract16  );
// }

// static void UpdateILimitsCw(Motor_State_T * p_motor)
// {
//     p_motor->ILimitCw_Fract16 = (int32_t)0 - (p_motor->ILimitMotoring_Fract16  );
//     p_motor->ILimitCcw_Fract16 = p_motor->ILimitGenerating_Fract16  ;
// }

// static void UpdateSpeedLimitsCcw(Motor_State_T * p_motor)
// {
//     int32_t ccw = (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16;
//     p_motor->SpeedLimitCcw_Fract16 = ccw  ;
//     p_motor->SpeedLimitCw_Fract16 = 0;
// }

// static void UpdateSpeedLimitsCw(Motor_State_T * p_motor)
// {
//     int32_t cw = (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitReverse_Fract16 : p_motor->SpeedLimitForward_Fract16;
//     p_motor->SpeedLimitCcw_Fract16 = 0;
//     p_motor->SpeedLimitCw_Fract16 = (int16_t)0 - (cw  );
// }





// alternatively derived values as base
// static inline uint16_t Motor_User_GetSpeedLimitForward(const Motor_State_T * p_motor) { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Fract16 : (0 - p_motor->SpeedLimitCw_Fract16); }
// static inline uint16_t Motor_User_GetSpeedLimitReverse(const Motor_State_T * p_motor) { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? (0 - p_motor->SpeedLimitCw_Fract16) : p_motor->SpeedLimitCcw_Fract16; }
// static inline uint16_t Motor_User_GetILimitMotoring(const Motor_State_T * p_motor)    { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->ILimitCcw_Fract16 : (0 - p_motor->ILimitCw_Fract16); }
// static inline uint16_t Motor_User_GetILimitGenerating(const Motor_State_T * p_motor)  { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? (0 - p_motor->ILimitCw_Fract16) : p_motor->ILimitCcw_Fract16; }

// static inline int16_t Motor_User_SpeedCmdLimitOf(const Motor_State_T * p_motor, int32_t speed_Fract16)
// {
//     return math_clamp(speed_Fract16, 0, Motor_User_GetSpeedLimit(p_motor));
// };

// static inline int16_t Motor_User_ICmdLimitOf(const Motor_State_T * p_motor, int16_t i_Fract16)
// {
//     return math_clamp(i_Fract16, (int32_t)0 - Motor_User_GetILimitGenerating(p_motor), Motor_User_GetILimitMotoring(p_motor));
// };







// /*
//     Alternative to enabling PID
//     on output as scalar
//     Req I, or V
// */
// static inline int16_t Motor_State_TorqueReqSpeedLimit(const Motor_State_T * p_motor, int16_t req)
// {
//     return feedback_scalar(req, Motor_GetSpeedLimitSelected(p_motor), math_abs(p_motor->Speed_Fract16));
//     /* fract16_div always return positive < 1 */
//     // if      (p_motor->Speed_Fract16 < p_motor->SpeedLimitCw_Fract16)  { scalar = fract16_div(p_motor->SpeedLimitCw_Fract16, p_motor->Speed_Fract16); } /* Speed is more negative */
//     // else if (p_motor->Speed_Fract16 > p_motor->SpeedLimitCcw_Fract16) { scalar = fract16_div(p_motor->SpeedLimitCcw_Fract16, p_motor->Speed_Fract16); }
//     // return fract16_mul(req, scalar);
// }

// static inline int16_t Motor_FOC_VReqILimit(const Motor_State_T * p_motor, int16_t req)
// {
//     return math_feedback_scalar(req, Motor_FOC_GetILimit(p_motor), math_abs(FOC_GetIq(&p_motor->Foc)));
//     //     uint16_t scalar = FRACT16_1_OVERSAT;
//     //     /* fract16_div always return positive < 1 */
//     //     if      (p_motor->Foc.Iq < _Motor_GetILimitCw(p_motor)) { scalar = fract16_div(_Motor_GetILimitCw(p_motor), p_motor->Foc.Iq); }
//     //     else if (p_motor->Foc.Iq > _Motor_GetILimitCcw(p_motor)) { scalar = fract16_div(_Motor_GetILimitCcw(p_motor), p_motor->Foc.Iq); }
//     //     return scalar;
// }

/******************************************************************************/
/* Reset unit conversion structs */
/* keep for extneral, physical  */
/******************************************************************************/
// void Motor_ResetSpeedFeedbackRef(Motor_State_T * p_motor)
// {
//     p_motor->SpeedFeedbackRef_Rpm = Motor_GetSpeedVRef_Rpm(p_motor);
// }

// void Motor_ResetUnitsIabc(Motor_State_T * p_motor)
// {
//     Motor_ResetUnitsIa(p_motor);
//     Motor_ResetUnitsIb(p_motor);
//     Motor_ResetUnitsIc(p_motor);
// }

// void Motor_ResetUnitsIa(Motor_State_T * p_motor)
// {
//     Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIa, p_motor->Config.IaZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
// #ifdef CONFIG_MOTOR_I_SENSORS_INVERT
//     Linear_ADC_SetInverted(&p_motor->UnitsIa);
// #endif
// }

// void Motor_ResetUnitsIb(Motor_State_T * p_motor)
// {
//     Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIb, p_motor->Config.IbZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
// #ifdef CONFIG_MOTOR_I_SENSORS_INVERT
//     Linear_ADC_SetInverted(&p_motor->UnitsIb);
// #endif
// }

// void Motor_ResetUnitsIc(Motor_State_T * p_motor)
// {
//     Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIc, p_motor->Config.IcZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
// #ifdef CONFIG_MOTOR_I_SENSORS_INVERT
//     Linear_ADC_SetInverted(&p_motor->UnitsIc);
// #endif
// }

// void Motor_ResetUnitsVabc(Motor_State_T * p_motor)
// {
// #if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
//     Linear_Q16_Init(&p_motor->UnitsVabc, 0U, MotorStatic.VSupplyRef_Adcu);
// #else
//     (void)p_motor;
// #endif
// }






// /******************************************************************************/
// /*!
//     @brief State
//     alternatively make this passive state, substate freewheel, hold
//     Release Control,
// */
// /******************************************************************************/
// static void Freewheel_Entry(const Motor_T * p_motor)
// {
//     Phase_Float(&p_motor->PHASE);
//     Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
// }

// static void Freewheel_Proc(const Motor_T * p_motor)
// {
//     Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
//     // alternatively wait for input
//     // if (p_motor->P_ACTIVE->Speed_Fract16 == 0U) { _StateMachine_Transition(&p_motor->STATE_MACHINE, &MOTOR_STATE_STOP); }
// }

// // static State_T * Freewheel_Next(const Motor_T * p_motor)
// // {
// //     return (p_motor->P_ACTIVE->Speed_Fract16 == 0U) ? &MOTOR_STATE_STOP : NULL;
// // }

// /* Match Feedback to ProcAngleBemf on Resume */
// static State_T * Freewheel_InputControl(const Motor_T * p_motor, state_input_value_t phaseOutput)
// {
//     State_T * p_nextState = NULL;

//     switch ((Phase_Output_T)phaseOutput)
//     {
//         case PHASE_OUTPUT_FLOAT:  if (p_motor->P_ACTIVE->Speed_Fract16 == 0U) { p_nextState = &MOTOR_STATE_STOP; }    break;
//         case PHASE_OUTPUT_V0: if (p_motor->P_ACTIVE->Speed_Fract16 == 0U) { p_nextState = &MOTOR_STATE_STOP; }    break;
//         case PHASE_OUTPUT_VPWM:
//             if (Motor_IsClosedLoop(p_motor) == true) { p_nextState = &MOTOR_STATE_RUN; } /* If flags set */
//             /* OpenLoop does not resume */
//             break;
//     }

//     return p_nextState;
// }

// static State_T * Freewheel_InputFeedbackMode(const Motor_T * p_motor, state_input_value_t feedbackMode)
// {
//     Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
//     return NULL;
// }

// static const State_Input_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
// {
//     [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
//     [MSM_INPUT_CONTROL_STATE]   = (State_Input_T)Freewheel_InputControl,
//     [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Freewheel_InputFeedbackMode,
//     [MSM_INPUT_DIRECTION]       = NULL,
//     [MSM_INPUT_CALIBRATION]     = NULL,
// };

// const State_T MOTOR_STATE_FREEWHEEL =
// {
//     .ID                 = MSM_STATE_ID_FREEWHEEL,
//     .P_TRANSITION_TABLE = &FREEWHEEL_TRANSITION_TABLE[0U],
//     .ENTRY              = (State_Action_T)Freewheel_Entry,
//     .LOOP               = (State_Action_T)Freewheel_Proc,
// };

/*
    Effectively sync mailbox for async results
*/
// typedef union Motor_StateFlags
// {
//     struct
//     {
//         // uint16_t HeatWarning        : 1U;
//         // uint16_t ILimited           : 1U;
//         // uint16_t SpeedLimited       : 1U;
//         // uint16_t RampDisable        : 1U;
//         // uint16_t VarControlEnabled  : 1U; /*  */
//     };
//     uint16_t Value;
// }
// Motor_StateFlags_T;


// extern void Motor_SetSpeedLimit(Motor_State_T * p_motor, uint16_t speed_ufract16);
// extern void Motor_SetILimit(Motor_State_T * p_motor, uint16_t i_ufract16);

// extern void Motor_ResetUnitsVabc(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsIabc(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsIa(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsIb(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsIc(Motor_State_T * p_motor);


// extern void Motor_SetMechAngleFeedforward(Motor_State_T * p_motor, angle16_t angle);
// extern void Motor_SetElecAngleFeedforward(Motor_State_T * p_motor, angle16_t angle);

// extern fract16_t Motor_ProcOuterFeedback(const Motor_T * p_motor);
// extern fract16_t Motor_ProcTorqueRamp(Motor_State_T * p_motor);
// extern fract16_t Motor_ProcSpeedRamp(Motor_State_T * p_motor);
// extern fract16_t Motor_ProcTorqueRampOpenLoop(Motor_State_T * p_motor);
// extern void Motor_MatchSpeedTorqueState(Motor_State_T * p_motor, int32_t torqueReq);