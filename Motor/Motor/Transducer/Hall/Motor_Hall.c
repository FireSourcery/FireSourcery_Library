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

static inline void Calibration_Entry(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
    p_motor->CalibrationStateIndex = 0U;
}

static inline void Calibration_Proc(Motor_T * p_motor)
{
    const uint16_t duty = p_motor->Config.AlignPower_UFract16;
    bool isComplete = false;

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        switch (p_motor->CalibrationStateIndex)
        {
            case 0U: Hall_StartCalibrate(&p_motor->Hall);       Phase_WriteDuty_Fract16(&p_motor->Phase, duty, 0U, 0U);      p_motor->CalibrationStateIndex = 1U;    break;
            case 1U: Hall_CalibratePhaseA(&p_motor->Hall);      Phase_WriteDuty_Fract16(&p_motor->Phase, duty, duty, 0U);    p_motor->CalibrationStateIndex = 2U;    break;
            case 2U: Hall_CalibratePhaseInvC(&p_motor->Hall);   Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, duty, 0U);      p_motor->CalibrationStateIndex = 3U;    break;
            case 3U: Hall_CalibratePhaseB(&p_motor->Hall);      Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, duty, duty);    p_motor->CalibrationStateIndex = 4U;    break;
            case 4U: Hall_CalibratePhaseInvA(&p_motor->Hall);   Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, 0U, duty);      p_motor->CalibrationStateIndex = 5U;    break;
            case 5U: Hall_CalibratePhaseC(&p_motor->Hall);      Phase_WriteDuty_Fract16(&p_motor->Phase, duty, 0U, duty);    p_motor->CalibrationStateIndex = 6U;    break;
            case 6U: Hall_CalibratePhaseInvB(&p_motor->Hall);   Phase_Float(&p_motor->Phase);                                p_motor->CalibrationStateIndex = 7U; isComplete = true;                      break;
            default: break;
        }
    }

}

static inline StateMachine_State_T * Calibration_End(Motor_T * p_motor)
{
    return (p_motor->CalibrationStateIndex == 7U) ? &MOTOR_STATE_STOP : NULL;
}

static const StateMachine_State_T CALIBRATION_STATE_HALL =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .DEPTH = 1U,
    .ENTRY = (StateMachine_Function_T)Calibration_Entry,
    .LOOP = (StateMachine_Function_T)Calibration_Proc,
    .NEXT = (StateMachine_Transition_T)Calibration_End,
};


void Motor_Hall_Calibrate(Motor_T * p_motor)
{
    // StateMachine_ProcSubStateInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_HALL);
    StateMachine_ProcBranchInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_HALL);
}
