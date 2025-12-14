/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Motor_Encoder.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor_Encoder.h"

#include "../../Motor_StateMachine.h"
#include "../../Motor.h"
#include "../../Motor_Var.h"
#include "../../Motor_FOC.h"
#include "../RotorSensor.h"

// /******************************************************************************/
// /*
//     Interface Table
// */
// /******************************************************************************/
// RotorSensor_VTable_T MOTOR_ENCODER =
// {
//     .Init = (RotorSensor_Proc_T)Encoder_ModeDT_Init_InterruptQuadrature,
//     // .VerifyCalibration = (RotorSensor_Test_T)Encoder_ModeDT_VerifyCalibration,
//     // .PollAngle = (RotorSensor_Angle_T)Encoder_ModeDT_GetAngle,
//     // .PollSpeed = (RotorSensor_Speed_T)Encoder_ModeDT_GetSpeed,
//     // .Zero = (RotorSensor_Proc_T)Encoder_ModeDT_SetInitial,
// };


// void Init(Encoder_T * p_encoder)
// {
//     Encoder_ModeDT_Init_InterruptQuadrature(p_encoder);
//     Encoder_EnableQuadratureMode(p_encoder);
//     // Motor_ResetUnitsSensor(p_motor);
// }

// // bool Motor_Init(const Motor_T * p_motor)
// // {
//     // Encoder_SetScalarSpeedRef(&p_motor->ENCODER, Motor_GetSpeedVRef_Rpm(p_motor));
// // }

// // bool Motor_VerifySensorCalibration(const Motor_T * p_motor)
// // {

// // }

// // angle16_t Motor_PollSensorAngle(const Motor_T * p_motor)
// // {
// //     Encoder_GetAngle_Scalar(&p_motor->ENCODER, p_motor->Config.PolePairs); /* ElectricalAngle => MechanicalAngle * PolePairs */
// //     // electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->ENCODER);
// // }

// // angle16_t Motor_GetMechanicalAngle(const const Motor_T * p_motor)
// // {
// //     Encoder_GetAngle(&p_motor->ENCODER);
// // }

// // /*!
// //     @return Fract16 Q1.15 unsaturated
// // */
// // int32_t Motor_PollSensorSpeed(const Motor_T * p_motor)
// // {
// //     Encoder_ModeDT_CaptureScalarVelocity(&p_motor->ENCODER) / 2;
// // }

// // void Motor_ZeroSensor(const Motor_T * p_motor)
// // {
// //     Encoder_ModeDT_SetInitial(&p_motor->ENCODER);
// // }

// // /* From Stop and after Align */
// // bool _Motor_IsSensorAvailable(const const Motor_T * p_motor)
// // {
// //     Encoder_IsAligned(&p_motor->ENCODER);
// // }

// // // inline bool Motor_IsClosedLoop(const const Motor_T * p_motor)
// // // {
// // //     return ((_Motor_IsSensorAvailable(p_motor) == true) && (_Motor_IsOpenLoop(p_motor) == false));
// // // }

// // Motor_Direction_T Motor_GetUserDirection(const Motor_T * p_motor)
// // {

// // }

// // /*
// //     Sensor Direction
// // */
// // void Motor_SetSensorCcw(const Motor_T * p_motor)
// // {

// // }

// // void Motor_SetSensorCw(const Motor_T * p_motor)
// // {

// // }

// // void Motor_ResetUnitsSensor(const Motor_T * p_motor)
// // {
// //     Motor_ResetUnitsEncoder(p_motor);
// // }

// angle resolution 65536/cpr
// el angle per tick = 65536/cpr * polepairs
// counts per electrical revolution = cpr/polepairs
// // /* Common, Set after PolePairs */
// // void Motor_ResetUnitsEncoder(const Motor_T * p_motor)
// // {
// //     if (Motor_GetSpeedVRef_Rpm(p_motor) != p_motor->ENCODER.Config.ScalarSpeedRef_Rpm)
// //     {
// //         Encoder_SetScalarSpeedRef(&p_motor->ENCODER, Motor_GetSpeedVRef_Rpm(p_motor));
// //     }
// //     if (p_motor->Config.PolePairs != p_motor->ENCODER.Config.PartitionsPerRevolution) /* Set for electrical cycle */
// //     {
// //         Encoder_SetPartitionsPerRevolution(&p_motor->ENCODER, p_motor->Config.PolePairs);
// //     }
// //     // if(p_motor->Config.GearRatioOutput != p_motor->ENCODER.Config.GearRatioOutput) ||
// //     // {
// //     //     Encoder_SetSurfaceRatio(&p_motor->ENCODER, p_motor->Config.GearRatio);
// //     // }
// // }

// /******************************************************************************/
// /*
//     States
// */
// /******************************************************************************/

// /*

// */
// static void StartHoming(const Motor_T * p_motor)
// {
//     Timer_StartPeriod_Millis(&p_motor->ControlTimer, 10); //~1rpm
//     Motor_FOC_ActivateOutputZero(p_motor);
//     Encoder_StartHoming(&p_motor->ENCODER);
//     p_motor->ElectricalAngle = 0U;
// }

// static void ProcHoming(const Motor_T * p_motor)
// {
//     uint16_t angle;

//     /* alternatively openloop speed/angle ramp */
//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         angle = Encoder_GetHomingAngle(&p_motor->ENCODER) * p_motor->Config.PolePairs;
//         Motor_FOC_ProcAngleFeedforwardV(p_motor, p_motor->ElectricalAngle + angle, Motor_GetVAlign_Duty( p_motor), 0);
//     }
// }

// static State_T * HomingTransition(const Motor_T * p_motor)
// {
//     State_T * p_nextState = NULL;

//     if (Encoder_PollHomingComplete(&p_motor->ENCODER) == true) /* todo error status */
//     {
//         // Encoder_CalibrateQuadratureDirection(&p_motor->ENCODER, p_motor->Direction == MOTOR_DIRECTION_CCW);
//         // /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
//         // StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_FLOAT);
//         p_nextState = &MOTOR_STATE_CALIBRATION;
//     }

//     return p_nextState;
// }

// //todo add align
// static const State_T STATE_ENCODER_HOMING =
// {
//     // .ID         = MSM_STATE_ID_CALIBRATION,
//     .P_TOP      = &MOTOR_STATE_CALIBRATION,
//     .P_PARENT   = &MOTOR_STATE_CALIBRATION,
//     .DEPTH      = 1U,
//     .ENTRY      = (State_Action_T)StartHoming,
//     .LOOP       = (State_Action_T)ProcHoming,
//     .NEXT       = (State_InputVoid_T)HomingTransition,
// };


// void Motor_Encoder_StartHoming(const Motor_T * p_motor)
// {
//     StateMachine_Branch_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&STATE_ENCODER_HOMING);
// }

// /*  */
// void Motor_Encoder_CalibrateHomeOffset(const Motor_T * p_motor)
// {
//     Encoder_CalibrateIndexZeroRef(&p_motor->ENCODER);
// }

// void Motor_Encoder_StartVirtualHome(const Motor_T * p_motor)
// {
//     Motor_Calibration_StartHome(p_motor);
// }

// /******************************************************************************/
// /*
//     Open Loop
// */
// /******************************************************************************/
// // aligning assuming phase a has not been found.
// // static void OpenLoop_EncoderAlignZero(const Motor_T * p_motor, state_value_t null)
// // {
// //     Motor_CommutationModeFn_Call(p_motor, Motor_FOC_StartStartUpAlign, NULL);
// // }

// // static void OpenLoop_ProcEncoderAlignZero(const Motor_T * p_motor)
// // {
// //     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
// //     {
// //         p_motor->ElectricalAngle = 0U;
// //         Encoder_CaptureAlignZero(&p_motor->ENCODER);
// //         /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
// //     }
// //     else
// //     {
// //         Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcStartUpAlign, NULL);

// //         if (Encoder_ModeDT_GetScalarSpeed(&p_motor->ENCODER) != 0U) /* reset the timer until speed is 0 */
// //         {
// //             Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
// //         }
// //     }
// // }

// static State_T *  AlignZeroNext(const Motor_T * p_motor)
// {
//     State_T * p_nextState = NULL;

//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         p_motor->ElectricalAngle = 0U;
//         Encoder_CaptureAlignZero(&p_motor->ENCODER);
//         /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
//     }
//     else
//     {
//         Motor_PollSensorAngle(p_motor);
//         Motor_PollCaptureSpeed(p_motor);
//         if (Encoder_ModeDT_GetScalarSpeed(&p_motor->ENCODER) != 0U) /* reset the timer until speed is 0 */
//         {
//             Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
//         }
//     }

//     return NULL;
// }


// static const State_T ALIGN =
// {
//     // .ID         = MSM_STATE_ID_OPEN_LOOP,
//     .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
//     .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH      = 1U,
//     .ENTRY      = (State_Action_T)Motor_FOC_StartStartUpAlign,
//     .LOOP       = (State_Action_T)Motor_FOC_ProcStartUpAlign,
//     .NEXT       = (State_InputVoid_T)AlignZeroNext,
// };


// /*

// */
// static void ValidateAlign(const Motor_T * p_motor)
// {
//     // if transitioning from align
//     // Motor_ZeroSensor(p_motor);
//     Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles * 2U);
//     Encoder_ModeDT_SetInitial(&p_motor->ENCODER);
//     FOC_SetReqD(&p_motor->Foc, 0);
//     Motor_FOC_MatchFeedbackState(p_motor);
//     Motor_FOC_StartOpenLoop(p_motor);
//     // p_motor->FeedbackMode.OpenLoop = 1U; /* input limited until openloop clears */
//     //p_motor->ElectricalAngle == 0U;
//     p_motor->MechanicalAngle = Encoder_GetAngle(&p_motor->ENCODER);
// }

// // static void OpenLoop_ProcEncoderValidateAlign(const Motor_T * p_motor)
// // {
//     //     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     //     {
//         //         if (Encoder_GetAngle(&p_motor->ENCODER) != p_motor->MechanicalAngle) { p_motor->FaultFlags.PositionSensor = 1U; }
//         //         // p_motor->FeedbackMode.OpenLoop = 0U;
//         //         // Encoder_CompleteAlignValidate(&p_motor->ENCODER);
//         //         /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
//         //     }
//         //     else
//         //     {
//             //         Motor_FOC_ProcOpenLoop(p_motor);
//             //         // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcAngleControl, NULL);  /* try closed loop */
//             //         // if ((p_motor->Speed_Fract16 ^ math_sign(p_motor->Foc.Vq)) < 0) { p_motor->FaultFlags.PositionSensor = 1U; }
//             //         // if (p_motor->FaultFlags.PositionSensor == 1U) { /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */ }
// //     }
// // }

// static State_T *  ValidateAlignNext(const Motor_T * p_motor)
// {
//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         if (Encoder_GetAngle(&p_motor->ENCODER) != p_motor->MechanicalAngle) { p_motor->FaultFlags.PositionSensor = 1U; }
//         // p_motor->FeedbackMode.OpenLoop = 0U;
//         // Encoder_CompleteAlignValidate(&p_motor->ENCODER);
//         /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
//     }

//     return NULL;
// }

// static const State_T VALIDATE_ALIGN =
// {
//     // .ID         = MSM_STATE_ID_OPEN_LOOP,
//     .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
//     .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH      = 1U,
//     .ENTRY      = (State_Action_T)ValidateAlign,
//     .LOOP       = (State_Action_T)Motor_FOC_ProcOpenLoop,
//     .NEXT       = (State_InputVoid_T)ValidateAlignNext,
// };


// // static void OpenLoop_ProcEncoderValidateClosedLoop(const Motor_T * p_motor)
// // {
// //     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
// //     {
// //         p_motor->FeedbackMode.OpenLoop = 0U;
// //         Encoder_CompleteAlignValidate(&p_motor->ENCODER);
// //         /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
// //     }
// //     else
// //     {
// //         Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcAngleControl, NULL);  /* try closed loop */
// //         // or encoder get speed
// //         if ((p_motor->Speed_Fract16 ^ math_sign(p_motor->Foc.Vq)) < 0) { p_motor->FaultFlags.PositionSensor = 1U; }
// //         if (p_motor->FaultFlags.PositionSensor == 1U) { /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */ }
// //     }
// // }

// /*  */
// static State_T * ValidateClosedLoopTransition(const Motor_T * p_motor)
// {
//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         p_motor->FeedbackMode.OpenLoop = 0U;
//         Encoder_CompleteAlignValidate(&p_motor->ENCODER);
//         /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
//     }
//     else
//     {
//         // or encoder get speed
//         if ((p_motor->Speed_Fract16 ^ math_sign(p_motor->Foc.Vq)) < 0)
//         {
//             p_motor->FaultFlags.PositionSensor = 1U;
//             /* _StateMachine_EndSubState(&p_motor->STATE_MACHINE); */
//         }
//     }

//     return NULL;
// }

// static const State_T VALIDATE_CLOSED_LOOP =
// {
//     // .ID         = MSM_STATE_ID_OPEN_LOOP,
//     .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
//     .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH      = 1U,
//     .ENTRY      = (State_Action_T)NULL,
//     .LOOP       = (State_Action_T)Motor_FOC_ProcAngleControl,
//     .NEXT       = (State_InputVoid_T)ValidateClosedLoopTransition,
//     // .P_TRANSITION_TABLE = NULL,
// };


// static State_T * Cmd_Align(const Motor_T * p_motor, state_value_t null)
// {
//     // Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
//     // Phase_WriteDuty_Fract16(&p_motor->PHASE, Motor_GetVAlign_Duty( p_motor), 0U, 0U);
//     return &ALIGN;
// }

// /* individual state test */
// void Motor_Encoder_StartAlignZero(const Motor_T * p_motor)
// {
//     static const StateMachine_TransitionInput_T CMD = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)Cmd_Align, };
//     StateMachine_Branch_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0);
// }

// static State_T * Cmd_ValidateAlign(const Motor_T * p_motor, state_value_t null)
// {
//     return &VALIDATE_ALIGN;
// }

// void Motor_Encoder_StartValidateAlign(const Motor_T * p_motor)
// {
//     static const StateMachine_TransitionInput_T CMD = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)Cmd_ValidateAlign, };
//     StateMachine_Branch_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0);
// }


// static State_T * Cmd_ValidateClosedLoop(const Motor_T * p_motor, state_value_t null)
// {
//     return &VALIDATE_CLOSED_LOOP;
// }

// void Motor_Encoder_StartValidateClosedLoop(const Motor_T * p_motor)
// {
//     static const StateMachine_TransitionInput_T CMD = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)Cmd_ValidateClosedLoop, };
//     StateMachine_Branch_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0);
// }


// /******************************************************************************/
// /*
//     Start Up Chain
// */
// /******************************************************************************/
// static State_T * StartUpTransition(const Motor_T * p_motor);
// static State_T * StartUpAlignTransition(const Motor_T * p_motor);
// static State_T * StartUpValidateAlignTransition(const Motor_T * p_motor);
// static State_T * StartUpValidateClosedLoopTransition(const Motor_T * p_motor);

// static const State_T START_UP =
// {
//     // .ID = MSM_STATE_ID_OPEN_LOOP,
//     .P_TOP  = &MOTOR_STATE_OPEN_LOOP,

//     .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH = 1U,

//     // .ENTRY = (State_Action_T) ,
//     // .LOOP = (State_Action_T) ,
//     .NEXT = (State_InputVoid_T)StartUpTransition,
// };

// static const State_T START_UP_ALIGN =
// {
//     // .ID = MSM_STATE_ID_OPEN_LOOP,
//     .P_TOP  = &MOTOR_STATE_OPEN_LOOP,
//     .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)Motor_FOC_StartStartUpAlign,
//     .LOOP = (State_Action_T)Motor_FOC_ProcStartUpAlign,
//     .NEXT = (State_InputVoid_T)StartUpAlignTransition,

//     // .P_TOP  = &MOTOR_STATE_OPEN_LOOP,
//     // .P_PARENT = &ALIGN,
//     // .DEPTH = 2U,
//     // .NEXT = (State_InputVoid_T)StartUpAlignTransition,
// };

// static const State_T START_UP_VALIDATE_ALIGN =
// {
//     // .ID = MSM_STATE_ID_OPEN_LOOP,
//     .P_TOP  = &MOTOR_STATE_OPEN_LOOP,
//     .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)ValidateAlign,
//     .LOOP = (State_Action_T)Motor_FOC_ProcOpenLoop,
//     .NEXT = (State_InputVoid_T)StartUpValidateAlignTransition,
// };

// static const State_T START_UP_VALIDATE_CLOSED_LOOP =
// {
//     // .ID = MSM_STATE_ID_OPEN_LOOP,
//     .P_TOP  = &MOTOR_STATE_OPEN_LOOP,
//     .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)NULL,
//     .LOOP = (State_Action_T)Motor_FOC_ProcAngleControl,
//     .NEXT = (State_InputVoid_T)StartUpValidateClosedLoopTransition,
// };

// static State_T * StartUpTransition(const Motor_T * p_motor)
// {
//     //if not aligned
//     return &START_UP_ALIGN;
// }

// static State_T * StartUpAlignTransition(const Motor_T * p_motor)
// {
//     return &START_UP_VALIDATE_ALIGN;
// }

// static State_T * StartUpValidateAlignTransition(const Motor_T * p_motor)
// {
//     return &START_UP_VALIDATE_CLOSED_LOOP;
// }

// static State_T * StartUpValidateClosedLoopTransition(const Motor_T * p_motor)
// {
//     return &MOTOR_STATE_RUN;
// }
// /*

// */

// State_T * Motor_Encoder_GetStartUpState(const Motor_T * p_motor)
// {
//     return &START_UP;
// }

// State_T * StartUpChain(const Motor_T * p_motor, state_value_t null)
// {
//     if (p_motor->Speed_Fract16 == 0U)    return &START_UP;
//     else return NULL;
// }

// void Motor_Encoder_StartUpChain(const Motor_T * p_motor)
// {
//     static const StateMachine_TransitionInput_T  START_UP_CHAIN = { .P_START = &MOTOR_STATE_PASSIVE, .TRANSITION = (State_Input_T)StartUpChain , };
//     StateMachine_Branch_InvokeTransition(&p_motor->STATE_MACHINE, &START_UP_CHAIN, 0);
// }


// /******************************************************************************/
// /* */
// /******************************************************************************/
// static inline void StartDirection(const Motor_T * p_motor)
// {
//     Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
//     Phase_WriteDuty_Fract16(&p_motor->PHASE, Motor_GetVAlign_Duty( p_motor), 0U, 0U);
// }

// static inline bool ProcDirection(const Motor_T * p_motor)
// {
//     bool isComplete = false;

//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         switch (p_motor->CalibrationStateIndex)
//         {
//             case 0U:
//                 Encoder_CaptureQuadratureReference(&p_motor->ENCODER);
//                 Phase_WriteDuty_Fract16(&p_motor->PHASE, 0U, Motor_GetVAlign_Duty( p_motor), 0U);
//                 p_motor->CalibrationStateIndex = 1U;
//                 break;

//             case 1U:
//                 Encoder_CalibrateQuadraturePositive(&p_motor->ENCODER);
//                 Phase_Float(&p_motor->PHASE);
//                 isComplete = true;
//                 break;
//             default: break;
//         }
//     }

//     return isComplete;
// }



/******************************************************************************/
/*
    Var Interface
*/
/******************************************************************************/

// #include "Transducer/Motor_Encoder/Motor_Encoder.h"
// #include "Transducer/Encoder/Encoder.h"

// int32_t Motor_Encoder_Config_Get(const Motor_T * p_motor, Encoder_ConfigId_T varId) { return _Encoder_ConfigId_Get(p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE, varId); }
// void Motor_Encoder_Config_Set(const Motor_T * p_motor, Encoder_ConfigId_T varId, int32_t varValue) { _Encoder_ConfigId_Set(p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE, varId, varValue); }



// int32_t _Motor_Var_ConfigEncoder_Get(const Motor_State_T * p_motor, Encoder_ConfigId_T varId) { return _Encoder_ConfigId_Get(&p_motor->ENCODER, varId); }

// void _Motor_Var_ConfigEncoder_Set(Motor_State_T * p_motor, Encoder_ConfigId_T varId, int32_t varValue) { _Encoder_ConfigId_Set(&p_motor->ENCODER, varId, varValue); }

// int Motor_Var_ConfigEncoder_Get(const Motor_State_T * p_motor, int varId) { return _Encoder_ConfigId_Get(&p_motor->ENCODER, varId); }
// void Motor_Var_ConfigEncoder_Set(Motor_State_T * p_motor, int varId, int varValue) { _Encoder_ConfigId_Set(&p_motor->ENCODER, varId, varValue); }

// const VarAccess_VTable_T MOTOR_VAR_CONFIG_ENCODER =
// {
//     .GET_AT = (get_field_t)Motor_Var_ConfigEncoder_Get,
//     .SET_AT = (set_field_t)Motor_Var_ConfigEncoder_Set,
//     .TEST_SET = (test_t)Motor_Config_IsConfigState,
// };
