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
    @version V0
*/
/******************************************************************************/
#include "Motor_Analog.h"
#include "Motor_StateMachine.h"

/******************************************************************************/
/*!
    @brief
*/
/* Without checking for previous completion. Conversions must complete in within the analog cycle or feedback will never process  */
/******************************************************************************/
void Motor_Analog_MarkVabc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VA);
    Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VB);
    Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VC);
#else
    (void)p_motor;
#endif
}

void Motor_Analog_MarkIabc(Motor_T * p_motor)
{
    Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IA);
    Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
    Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
}


/******************************************************************************/
/*!
    @brief
    Call by StateMachine
*/
/******************************************************************************/
static void StartCalibration(Motor_T * p_motor)
{
    Timer_StartPeriod_Millis(&p_motor->ControlTimer, 2000U); /* 2 Seconds */

    Phase_WriteDuty_Fract16(&p_motor->Phase, INT16_MAX / 2U, INT16_MAX / 2U, INT16_MAX / 2U);
    Phase_ActivateOutput(&p_motor->Phase);

    Filter_Avg_Init(&p_motor->FilterA);
    Filter_Avg_Init(&p_motor->FilterB);
    Filter_Avg_Init(&p_motor->FilterC);
    // p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IA.P_STATE->Result = 0U;
    // p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IB.P_STATE->Result = 0U;
    // p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IC.P_STATE->Result = 0U;
    p_motor->IBatch.Value = PHASE_ID_0;
    p_motor->Config.IaZeroRef_Adcu = 0U;
    p_motor->Config.IbZeroRef_Adcu = 0U;
    p_motor->Config.IcZeroRef_Adcu = 0U;
    Motor_Analog_MarkIabc(p_motor);
}

static void ProcCalibration(Motor_T * p_motor)
{
    if (p_motor->IBatch.Value == PHASE_ID_ABC)
    {
        Filter_Avg(&p_motor->FilterA, Motor_Analog_GetIa(p_motor));
        Filter_Avg(&p_motor->FilterB, Motor_Analog_GetIb(p_motor));
        Filter_Avg(&p_motor->FilterC, Motor_Analog_GetIc(p_motor));
        Motor_Analog_MarkIabc(p_motor);
    }
}

static StateMachine_State_T * EndCalibration(Motor_T * p_motor)
{
    StateMachine_State_T * p_nextState = NULL;

    if (Timer_IsElapsed(&p_motor->ControlTimer) == true)
    {
        p_motor->Config.IaZeroRef_Adcu = Filter_Avg(&p_motor->FilterA, Motor_Analog_GetIa(p_motor));
        p_motor->Config.IbZeroRef_Adcu = Filter_Avg(&p_motor->FilterB, Motor_Analog_GetIb(p_motor));
        p_motor->Config.IcZeroRef_Adcu = Filter_Avg(&p_motor->FilterC, Motor_Analog_GetIc(p_motor));
        // Motor_ResetUnitsIabc(p_motor);
        Phase_Float(&p_motor->Phase);
    }

    return p_nextState;
}

static const StateMachine_State_T CALIBRATION_STATE =
{
    .P_ROOT     = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)StartCalibration,
    .LOOP       = (StateMachine_Function_T)ProcCalibration,
    .NEXT       = (StateMachine_InputVoid_T)EndCalibration,
};

void Motor_Analog_Calibrate(Motor_T * p_motor)
{
    StateMachine_ProcBranchInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE);
}
