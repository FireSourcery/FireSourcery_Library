
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
    @file
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "MotorController_Analog.h"

#include <stdint.h>
#include <stdbool.h>

/*
    MotAnalog Callbacks go here
*/

/******************************************************************************/
/*!
    Calibrate
*/
/******************************************************************************/
void MotorController_Analog_StartCalibrate(MotorController_T * p_mc)
{
    p_mc->StateCounter = 0U;
    Analog_MarkConversion(&p_mc->CONST.CONVERSION_THROTTLE);
    Analog_MarkConversion(&p_mc->CONST.CONVERSION_BRAKE);
    p_mc->CONST.CONVERSION_THROTTLE.P_STATE->Result = 0U;
    p_mc->CONST.CONVERSION_BRAKE.P_STATE->Result = 0U;
    Filter_Avg_Init(&p_mc->AvgBuffer0);
    Filter_Avg_Init(&p_mc->AvgBuffer1);

    // void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_CalibrateAdc);
}

bool MotorController_Analog_ProcCalibrate(MotorController_T * p_mc)
{
    const uint32_t DIVIDER = (MOTOR_STATIC.CONTROL_ANALOG_DIVIDER << 1U) & 1U; /* 2x normal sample time */
    const uint32_t TIME = 2000U;

    // bool isLocalComplete = (p_mc->StateCounter == TIME); /* 2 seconds */

    if (p_mc->StateCounter == TIME)
    {
        MotAnalogUser_SetThrottleZero(&p_mc->AnalogUser, Filter_Avg(&p_mc->AvgBuffer0, p_mc->CONST.CONVERSION_THROTTLE.P_STATE->Result));
        MotAnalogUser_SetBrakeZero(&p_mc->AnalogUser, Filter_Avg(&p_mc->AvgBuffer1, p_mc->CONST.CONVERSION_BRAKE.P_STATE->Result));
    }
    else if (p_mc->StateCounter <= TIME)
    {
        if (p_mc->StateCounter != 0U) /* skip first time */
        {
            if ((p_mc->StateCounter & DIVIDER) == 0U)
            {
                Filter_Avg(&p_mc->AvgBuffer0, p_mc->CONST.CONVERSION_THROTTLE.P_STATE->Result);
                Filter_Avg(&p_mc->AvgBuffer1, p_mc->CONST.CONVERSION_BRAKE.P_STATE->Result);
                Analog_MarkConversion(&p_mc->CONST.CONVERSION_THROTTLE);
                Analog_MarkConversion(&p_mc->CONST.CONVERSION_BRAKE);
            }
        }
    }

    p_mc->StateCounter++;

    return (p_mc->StateCounter > TIME)  ;
}
