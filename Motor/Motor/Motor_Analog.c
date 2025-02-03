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


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
void Motor_Analog_MarkVabc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    /* Without checking for previous completion. Conversions must complete in within the analog cycle or feedback will never process  */
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
void Motor_Analog_StartCalibration(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, MOTOR_STATIC.CONTROL_FREQ * 2U); /* 2 Seconds */
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ActivateOutputZero, 0U /*Phase_Ground(&p_motor->Phase)*/);

    Filter_Avg_Init(&p_motor->FilterA);
    Filter_Avg_Init(&p_motor->FilterB);
    Filter_Avg_Init(&p_motor->FilterC);
    p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IA.P_STATE->Result = 0U;
    p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IB.P_STATE->Result = 0U;
    p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IC.P_STATE->Result = 0U;
    Motor_Analog_MarkIabc(p_motor);
}

bool Motor_Analog_ProcCalibration(Motor_T * p_motor)
{
    const uint32_t DIVIDER = (MOTOR_STATIC.CONTROL_ANALOG_DIVIDER << 1U) & 1U; /* 2x normal sample time */

    bool isComplete = Timer_IsElapsed(&p_motor->ControlTimer);
    if (isComplete == true)
    {
        p_motor->Config.IaZeroRef_Adcu = Filter_Avg(&p_motor->FilterA, Motor_Analog_GetIa(p_motor));
        p_motor->Config.IbZeroRef_Adcu = Filter_Avg(&p_motor->FilterB, Motor_Analog_GetIb(p_motor));
        p_motor->Config.IcZeroRef_Adcu = Filter_Avg(&p_motor->FilterC, Motor_Analog_GetIc(p_motor));
        Motor_ResetUnitsIabc(p_motor);
        Phase_Float(&p_motor->Phase);
    }
    else
    {
        if (p_motor->ControlTimerBase != 0U) /* skip first time */
        {
            if ((p_motor->ControlTimerBase & DIVIDER) == 0U)
            {
                Filter_Avg(&p_motor->FilterA, Motor_Analog_GetIa(p_motor));
                Filter_Avg(&p_motor->FilterB, Motor_Analog_GetIb(p_motor));
                Filter_Avg(&p_motor->FilterC, Motor_Analog_GetIc(p_motor));
                Motor_Analog_MarkIabc(p_motor);
            }
        }
    }

    return isComplete;
}