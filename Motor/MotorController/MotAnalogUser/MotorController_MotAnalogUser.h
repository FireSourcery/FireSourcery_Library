// capture short handl + state

// static inline void _MotorController_ProcAnalogUser(const MotorController_T * p_context)
// {
//     MotorController_State_T * p_mc = p_context->P_MC_STATE;
//     MotAnalogUser_Cmd_T cmd;

//     MotAnalogUser_CaptureInput(&p_context->ANALOG_USER, MotAnalogUser_Conversion_GetThrottle(&p_context->ANALOG_USER_CONVERSIONS), MotAnalogUser_Conversion_GetBrake(&p_context->ANALOG_USER_CONVERSIONS));

//     // cmd = MotAnalogUser_PollCmd(&p_context->ANALOG_USER);
//     // switch (cmd)
//     // {
//     //     // case MOT_ANALOG_USER_CMD_SET_BRAKE:                 MotorController_User_SetCmdBrake(p_mc, MotAnalogUser_GetBrake(&p_context->ANALOG_USER));          break;
//     //     // case MOT_ANALOG_USER_CMD_SET_THROTTLE:              MotorController_User_SetCmdThrottle(p_mc, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));    break;
//     //     //                                                     // MotDrive_SetThrottleValue(&p_mc->MotDrive, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
//     //     // case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:         MotorController_User_SetCmdBrake(p_mc, 0U);                                                 break;
//     //     // case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:      MotorController_User_SetCmdThrottle(p_mc, 0U);                                              break;
//     //     // case MOT_ANALOG_USER_CMD_PROC_ZERO:                 MotorController_User_SetCmdDriveZero(p_mc);                                                 break;
//     //     // case MOT_ANALOG_USER_CMD_SET_NEUTRAL:               MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_NEUTRAL);                       break;
//     //     case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD:     //         // MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);    //         break;
//     //     case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE:    //         // MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);    //         break;
//     //     case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:  break;
//     //     default: break;
//     // }

//     if (TimerT_Counter_IsAligned(&p_context->MILLIS_TIMER, MOTOR_CONTROLLER_ANALOG_USER_DIVIDER) == true)
//     {
//         MotAnalogUser_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS);
//     }
// }

/******************************************************************************/
/*!
    @brief CalibrateAdc SubState
*/
/******************************************************************************/
// void StartCalibrateAdc(const MotorController_T * p_context)
// {
//     MotorController_State_T * p_mc = p_context->P_MC_STATE;
//     p_mc->StateCounter = 0U;
//     Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE);
//     Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.BRAKE);
//     Analog_Conversion_ClearResult(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE);
//     Analog_Conversion_ClearResult(&p_context->ANALOG_USER_CONVERSIONS.BRAKE);
//     Filter_Init(&p_mc->AvgBuffer0);
//     Filter_Init(&p_mc->AvgBuffer1);
//     // MotMotors_EnterCalibrateAdc(&p_context->MOTORS); /* Motor handles it own state */
// }

// /* Proc Per ms */
// void ProcCalibrateAdc(const MotorController_T * p_context)
// {
//     MotorController_State_T * p_mc = p_context->P_MC_STATE;

//     if (p_mc->StateCounter != 0U) /* skip first time */
//     {
//         Filter_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE));
//         Filter_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.BRAKE));
//         Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE);
//         Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.BRAKE);
//     }

//     p_mc->StateCounter++;
// }

// static State_T * EndCalibrateAdc(const MotorController_T * p_context)
// {
//     const uint32_t TIME = 2000U; /* > Motor calibrate adc time */

//     MotorController_State_T * p_mc = p_context->P_MC_STATE;
//     State_T * p_nextState = NULL;

//     if (p_mc->StateCounter > TIME)
//     {
//         // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false)
//         MotAnalogUser_SetThrottleZero(&p_context->ANALOG_USER, Filter_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE)));
//         MotAnalogUser_SetBrakeZero(&p_context->ANALOG_USER, Filter_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.BRAKE)));
//         p_mc->LockOpStatus = 0; /* success */

//         p_nextState = &MC_STATE_LOCK; /* return to lock state */
//     }

//     return p_nextState;
// }

// static const State_T MC_STATE_LOCK_CALIBRATE_ADC =
// {
//     .ID = MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC,
//     .P_TOP = &MC_STATE_LOCK,
//     .P_PARENT = &MC_STATE_LOCK,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)StartCalibrateAdc,
//     .LOOP = (State_Action_T)ProcCalibrateAdc,
//     .NEXT = (State_InputVoid_T)EndCalibrateAdc,
// };