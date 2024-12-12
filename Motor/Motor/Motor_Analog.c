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
#include "Motor_FOC.h"
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#include "Motor_SixStep.h"
#endif

// const Analog_ConversionBatch_T BATCH_V =
// {
//     .P_CONVERSIONS = (const Analog_Conversion_T * []) { &CONVERSION_VA, &CONVERSION_VB, &CONVERSION_VC },
//     .CONVERSION_COUNT = 3U,
//     .ON_COMPLETE = Motor_Analog_CaptureVa,
// }

/******************************************************************************/
/*!
    @brief  Callback functions
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief  Adc Capture
*/
/******************************************************************************/
void Motor_Analog_CaptureVa(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureVa, 0U /* Motor_SixStep_CaptureVBemfA */, adcu); }
void Motor_Analog_CaptureVb(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureVb, 0U /* Motor_SixStep_CaptureVBemfB */, adcu); }
void Motor_Analog_CaptureVc(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureVc, 0U /* Motor_SixStep_CaptureVBemfC */, adcu); }

void Motor_Analog_CaptureIa(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureIa, 0U /* Motor_SixStep_CaptureIa */, adcu); }
void Motor_Analog_CaptureIb(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureIb, 0U /* Motor_SixStep_CaptureIb */, adcu); }
void Motor_Analog_CaptureIc(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureIc, 0U /* Motor_SixStep_CaptureIc */, adcu); }

void Motor_Analog_CaptureHeat(Motor_T * p_motor, uint16_t adcu) { p_motor->AnalogResults.Heat_Adcu = adcu; } /* Temp for compatibility */


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
void Motor_Analog_MarkVabc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    if (Motor_IsAnalogCycle(p_motor) == true)
    {
        Analog_MarkConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VA);
        Analog_MarkConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VB);
        Analog_MarkConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VC);
    }
#else
    (void)p_motor;
#endif
}

void Motor_Analog_MarkIabc(Motor_T * p_motor)
{
    if (Motor_IsAnalogCycle(p_motor) == true)
    {
        Analog_MarkConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IA);
        Analog_MarkConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IB);
    #if defined(CONFIG_MOTOR_I_SENSORS_ABC)
        Analog_MarkConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IC);
    #endif
    }
}


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
void Motor_Analog_StartCalibration(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, MOTOR_STATIC.CONTROL_FREQ * 2U); /* 2 Seconds */
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ActivateOutput, 0U /*Phase_Ground(&p_motor->Phase)*/);

    Filter_Avg_Init(&p_motor->FilterA);
    Filter_Avg_Init(&p_motor->FilterB);
    Filter_Avg_Init(&p_motor->FilterC);
    p_motor->AnalogResults.Ia_Adcu = 0U; //todo
    p_motor->AnalogResults.Ib_Adcu = 0U;
    p_motor->AnalogResults.Ic_Adcu = 0U;
    // Motor_Analog_EnqueueIabc(p_motor);
    Motor_Analog_MarkIabc(p_motor);
}

bool Motor_Analog_ProcCalibration(Motor_T * p_motor)
{
    const uint32_t DIVIDER = (MOTOR_STATIC.CONTROL_ANALOG_DIVIDER << 1U) & 1U; /* 2x normal sample time */
    bool isComplete = Timer_Periodic_Poll(&p_motor->ControlTimer);
    if (isComplete == true)
    {
        p_motor->Config.IaZeroRef_Adcu = Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_Adcu);
        p_motor->Config.IbZeroRef_Adcu = Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_Adcu);
        p_motor->Config.IcZeroRef_Adcu = Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_Adcu);
        Motor_ResetUnitsIabc(p_motor);
        Phase_Float(&p_motor->Phase);
    }
    else
    {
        if((p_motor->ControlTimerBase & DIVIDER) == 0UL)
        {
            Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_Adcu);
            Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_Adcu);
            Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_Adcu);
            // Motor_Analog_EnqueueIabc(p_motor);
            Motor_Analog_MarkIabc(p_motor);
        }
    }

    return isComplete;
}