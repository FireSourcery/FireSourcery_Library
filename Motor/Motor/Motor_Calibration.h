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
    @file   Motor_Calibration_.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include "Motor.h"
#include "Motor_FOC.h"
#include "Motor_Analog.h"

/******************************************************************************/
/*
    Part of StateMachine - Mapped to StateMachine
    Calibration State Functions
*/
/******************************************************************************/

/******************************************************************************/
/*
    Sensors
*/
/******************************************************************************/
static inline void Motor_Calibration_StartHall(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
}

static inline bool Motor_Calibration_ProcHall(Motor_T * p_motor)
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
            case 6U: Hall_CalibratePhaseInvB(&p_motor->Hall);   Phase_Float(&p_motor->Phase);                                isComplete = true;                      break;
            default: break;
        }
    }

    return isComplete;
}




#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
static inline void Motor_Calibrate_StartSinCos(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_ControlCycles);
}

static inline bool Motor_Calibrate_SinCos(Motor_T * p_motor)
{
    bool isComplete = false;

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        switch (p_motor->CalibrationStateIndex)
        {
            case 0U:
                Phase_WriteDuty_Fract16(&p_motor->Phase, p_motor->Config.AlignPower_UFract16, 0U, 0U);
                p_motor->CalibrationStateIndex = 2U;
                /* wait 1s */
                break;

            // case 1U:
            //     //can repeat adc and filter results, or skip state use check in sensor routine
            //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
            //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
            //     p_motor->CalibrationStateIndex = 2U;
            //     /* wait 50us, 1s */
            //     break;

            case 2U:
                SinCos_CalibrateA(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
                Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, p_motor->Config.AlignPower_UFract16, 0U);
                p_motor->CalibrationStateIndex = 4U;
                /* wait 1s */
                break;

            // case 3U:
            //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
            //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
            //     p_motor->CalibrationStateIndex = 4U;
            //     break;

            case 4U:
                SinCos_CalibrateB(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
//                Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, 0U, p_motor->Config.AlignPower_UFract16);
                p_motor->CalibrationStateIndex = 5U;
                // isComplete = true;
                Phase_WriteDuty_Fract16(&p_motor->Phase, p_motor->Config.AlignPower_UFract16, 0U, 0U);
                break;

            case 5U:
                p_motor->SinCos.DebugAPostMech = SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
                p_motor->SinCos.DebugAPostElec = SinCos_GetElectricalAngle(&p_motor->SinCos);
                Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, p_motor->Config.AlignPower_UFract16, 0U);
                p_motor->CalibrationStateIndex = 6U;
                break;

            case 6U:
                p_motor->SinCos.DebugBPostMech = SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
                p_motor->SinCos.DebugBPostElec = SinCos_GetElectricalAngle(&p_motor->SinCos);
                p_motor->CalibrationStateIndex = 0U;
                isComplete = true;
                break;
            default: break;
        }
    }

    return isComplete;
}
#endif



#endif
