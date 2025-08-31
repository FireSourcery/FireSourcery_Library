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
    @file   .h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "Motor_Analog.h"
#include "../Motor_StateMachine.h"



/******************************************************************************/
/*!
    @brief Calibration Routine via Motor_T StateMachine
*/
/******************************************************************************/
static void StartCalibration(const Motor_T * p_motor)
{
    Motor_State_T * const p_fields = p_motor->P_MOTOR_STATE;

    // Timer_StartPeriod_Millis(&p_fields->ControlTimer, 2000U); /* 2 Seconds */
    TimerT_Periodic_Set(&p_motor->CONTROL_TIMER, 40000U); /* 2 Seconds */

    Phase_WriteDuty_Fract16(&p_motor->PHASE, INT16_MAX / 2U, INT16_MAX / 2U, INT16_MAX / 2U);
    Phase_ActivateOutput(&p_motor->PHASE);

    Filter_Init(&p_fields->FilterA);
    Filter_Init(&p_fields->FilterB);
    Filter_Init(&p_fields->FilterC);
    p_fields->Config.IabcZeroRef_Adcu.A = 0U;
    p_fields->Config.IabcZeroRef_Adcu.B = 0U;
    p_fields->Config.IabcZeroRef_Adcu.C = 0U;
    Motor_Analog_MarkIabc(p_motor);
    p_fields->PhaseInput.IFlags.Id = PHASE_ID_0;
}

static void ProcCalibration(const Motor_T * p_motor)
{
    Motor_State_T * const p_fields = p_motor->P_MOTOR_STATE;

    if (p_fields->PhaseInput.IFlags.Id == PHASE_ID_ABC)
    {
        Filter_Avg(&p_fields->FilterA, Phase_Input_GetIa_Fract16(&p_fields->PhaseInput));
        Filter_Avg(&p_fields->FilterB, Phase_Input_GetIb_Fract16(&p_fields->PhaseInput));
        Filter_Avg(&p_fields->FilterC, Phase_Input_GetIc_Fract16(&p_fields->PhaseInput));
        Motor_Analog_MarkIabc(p_motor);
        p_fields->PhaseInput.IFlags.Id = PHASE_ID_0;
    }
}

static State_T * EndCalibration(const Motor_T * p_motor)
{
    Motor_State_T * const p_fields = p_motor->P_MOTOR_STATE;

    State_T * p_nextState = NULL;

    if (TimerT_IsElapsed(&p_motor->CONTROL_TIMER) == true)
    {
        p_fields->Config.IabcZeroRef_Adcu.A = Filter_Avg(&p_fields->FilterA, Phase_Input_GetIa_Fract16(&p_fields->PhaseInput));
        p_fields->Config.IabcZeroRef_Adcu.B = Filter_Avg(&p_fields->FilterB, Phase_Input_GetIb_Fract16(&p_fields->PhaseInput));
        p_fields->Config.IabcZeroRef_Adcu.C = Filter_Avg(&p_fields->FilterC, Phase_Input_GetIc_Fract16(&p_fields->PhaseInput));
        // Motor_ResetUnitsIabc(p_motor->p_Analog);
        Phase_Float(&p_motor->PHASE);
        p_nextState = &MOTOR_STATE_CALIBRATION; /* return to parent state, idle state */
    }

    return p_nextState;
}

/*  */
static const State_T CALIBRATION_STATE =
{
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)StartCalibration,
    .LOOP       = (State_Action_T)ProcCalibration,
    .NEXT       = (State_InputVoid_T)EndCalibration,
};

void Motor_Analog_Calibrate(const Motor_T * p_motor)
{
    StateMachine_ApplyBranchInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE);
}
