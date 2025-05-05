/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Motor_Hall.h
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#include "Motor_Hall.h"

// MotorSensor_VTable_T Hall_VTable =
// {
//     .Init = (MotorSensor_Proc_T)Hall_Init,
//     .VerifyCalibration = (MotorSensor_Test_T)Hall_IsTableValid,
//     .PollStateAngle = (MotorSensor_Angle_T)Hall_GetAngle,
//     .CaptureAngle = (MotorSensor_Angle_T)Hall_CaptureAngle,
//     .CaptureSpeed = (MotorSensor_Speed_T)Hall_CaptureSpeed,
//     .SetInitial = (MotorSensor_Proc_T)Hall_SetInitial,
//     .GetElectricalAngle = (MotorSensor_Angle_T)Hall_GetElectricalAngle,
//     .GetMechanicalAngle = (MotorSensor_Angle_T)Hall_GetMechanicalAngle,
// };


/*!
    Hall sensors as speed encoder.

    MECH_R = ELECTRIC_R / N_POLE_PAIRS
    ELECTRIC_R = 6 STEPS / Hall Cycle
    CPR = PolePairs*6   => GetSpeed => mechanical speed
    CPR = PolePairs     => GetSpeed => electrical speed
*/
// void Motor_ResetUnitsHallEncoder(Motor_T * p_motor)
// {
//     if (p_motor->Config.PolePairs * 6U != p_motor->Encoder.Config.CountsPerRevolution)
//     {
//         Encoder_SetCountsPerRevolution(&p_motor->Encoder, p_motor->Config.PolePairs * 6U);
//     }
//     p_motor->Encoder.Config.IsQuadratureCaptureEnabled = false;
// }

// /* Common, Set after PolePairs */
// void Motor_ResetUnitsEncoder(Motor_T * p_motor)
// {
//     if (Motor_GetSpeedVRef_Rpm(p_motor) != p_motor->Encoder.Config.ScalarSpeedRef_Rpm)
//     {
//         Encoder_SetScalarSpeedRef(&p_motor->Encoder, Motor_GetSpeedVRef_Rpm(p_motor));
//     }
//     if (p_motor->Config.PolePairs != p_motor->Encoder.Config.PartitionsPerRevolution) /* Set for electrical cycle */
//     {
//         Encoder_SetPartitionsPerRevolution(&p_motor->Encoder, p_motor->Config.PolePairs);
//     }
//     // if(p_motor->Config.GearRatioOutput != p_motor->Encoder.Config.GearRatioOutput) ||
//     // {
//     //     Encoder_SetSurfaceRatio(&p_motor->Encoder, p_motor->Config.GearRatio);
//     // }
// }


/* Include [Phase] and [P_PARENT] State */

static void Calibration_Entry(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
    Phase_ActivateOutputV0(&p_motor->Phase);
    p_motor->CalibrationStateIndex = 0U;
}

static_assert(HALL_SENSORS_VIRTUAL_A == PHASE_ID_A);

static void Calibration_Proc(Motor_T * p_motor)
{
    const uint16_t duty = p_motor->Config.AlignPower_Fract16;

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        switch (p_motor->CalibrationStateIndex) /* Phase_ReadAlign() */
        {
            case 0U: Hall_StartCalibrate(&p_motor->Hall);                               Phase_Align(&p_motor->Phase, PHASE_ID_A, duty);        p_motor->CalibrationStateIndex = 1U;    break;
            case 1U: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_A);       Phase_Align(&p_motor->Phase, PHASE_ID_INV_C, duty);    p_motor->CalibrationStateIndex = 2U;    break;
            case 2U: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_INV_C);   Phase_Align(&p_motor->Phase, PHASE_ID_B, duty);        p_motor->CalibrationStateIndex = 3U;    break;
            case 3U: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_B);       Phase_Align(&p_motor->Phase, PHASE_ID_INV_A, duty);    p_motor->CalibrationStateIndex = 4U;    break;
            case 4U: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_INV_A);   Phase_Align(&p_motor->Phase, PHASE_ID_C, duty);        p_motor->CalibrationStateIndex = 5U;    break;
            case 5U: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_C);       Phase_Align(&p_motor->Phase, PHASE_ID_INV_B, duty);    p_motor->CalibrationStateIndex = 6U;    break;
            case 6U: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_INV_B);   Phase_Float(&p_motor->Phase);                          p_motor->CalibrationStateIndex = 7U;    break;
            default: break;
            // case PHASE_ID_A: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_A); Phase_AlignNext(&p_motor->Phase, duty); break;
        }
    }
}

static StateMachine_State_T * Calibration_End(Motor_T * p_motor)
{
    bool isComplete = (p_motor->CalibrationStateIndex >= 7U); /* (Phase_ReadAlign(&p_motor->Phase) == PHASE_ID_0) */
    if (isComplete == true) { p_motor->FaultFlags.PositionSensor = !Hall_IsTableValid(&p_motor->Hall); }
    return isComplete ? &MOTOR_STATE_CALIBRATION : NULL;
}

static const StateMachine_State_T CALIBRATION_STATE_HALL =
{
    .P_ROOT = &MOTOR_STATE_CALIBRATION,
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .DEPTH = 1U,
    .ENTRY = (StateMachine_Function_T)Calibration_Entry,
    .LOOP = (StateMachine_Function_T)Calibration_Proc,
    .NEXT = (StateMachine_InputVoid_T)Calibration_End,
};

void Motor_Hall_Calibrate(Motor_T * p_motor)
{
    StateMachine_ProcBranchInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_HALL);
    // StateMachine_InvokeBranchTransition(&p_motor->StateMachine, &(const StateMachine_TransitionInput_T) {.P_VALID = &MOTOR_STATE_CALIBRATION, .TRANSITION = Calibration_Start }, 0);
}