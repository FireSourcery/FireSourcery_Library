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
    @file   Motor_Thread.h
    @author FireSourcery
    @brief  Motor module functions must be placed into corresponding user app threads
            Outer most functions to call from main app
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include "Motor_StateMachine.h"
#include "Motor_Analog.h"
#include "Motor_User.h"
#include "Motor_Debug.h"
#include "Motor.h"

#include "Transducer/Encoder/Encoder_ISR.h"

/*
    Default 50us
    Calling function clears interrupt flag
*/
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
// #ifndef NDEBUG
    // Motor_Debug_CaptureRefTime(p_motor);
// #endif
    StateMachine_ProcState(&p_motor->StateMachine);
#ifdef CONFIG_MOTOR_PWM_INTERRUPT_CLEAR_PER_MOTOR
    Motor_ClearInterrupt(p_motor);
#endif

    p_motor->ControlTimerBase++;
}

/* Optionally mark before Start */
void Motor_MarkAnalog_Thread(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
    if (p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
    {
        Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
        Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
    }
#endif

    switch (StateMachine_GetActiveStateId(&p_motor->StateMachine))
    {
        case MSM_STATE_ID_STOP:         Motor_Analog_MarkVabc(p_motor);            break;
        case MSM_STATE_ID_RUN:          Motor_Analog_MarkIabc(p_motor);            break;
        case MSM_STATE_ID_FREEWHEEL:    Motor_Analog_MarkVabc(p_motor);            break;
        case MSM_STATE_ID_FAULT:        Motor_Analog_MarkVabc(p_motor);             break;
        // case MSM_STATE_ID_FAULT:        Motor_Analog_MarkVabc(p_motor); Motor_Analog_MarkIabc(p_motor); break;

        default:            break;
    }
    //    switch(p_motor->AnalogCmd)
    //    {
    //        case FOC_I_ABC :
    //            break;
    //        case FOC_VBEMF :
    //            break;
    //        default :
    //            break;
    //    }
}

static inline void Motor_Heat_Thread(Motor_T * p_motor)
{
    if (Thermistor_IsMonitorEnable(&p_motor->Thermistor) == true)
    {
        Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_HEAT);

        switch (Thermistor_PollMonitor(&p_motor->Thermistor, Motor_Analog_GetHeat(p_motor)))
        {
            case THERMISTOR_STATUS_OK:
                if (p_motor->StateFlags.HeatWarning == 1U) /* todo move to thermistor */
                {
                    p_motor->StateFlags.HeatWarning = 0U;
                    Motor_ClearILimitEntry(p_motor, MOTOR_I_LIMIT_ACTIVE_HEAT_THIS);
                }
                break;
            case THERMISTOR_STATUS_WARNING:     /* repeatedly checks if heat is a lower ILimit when another ILimit is active */
                p_motor->StateFlags.HeatWarning = 1U;
                Motor_SetILimitEntry(p_motor, MOTOR_I_LIMIT_ACTIVE_HEAT_THIS, Thermistor_GetHeatLimit_Percent16(&p_motor->Thermistor));
                break;
            case THERMISTOR_STATUS_FAULT:
                p_motor->FaultFlags.Overheat = 1U;
                Motor_StateMachine_EnterFault(p_motor);
                break;
            default: break;
        }
    }
}


/* Optionally use Hall ISR */
static inline void Motor_HallEncoderA_ISR(Motor_T * p_motor)
{
    Encoder_OnPhaseA_ISR(&p_motor->Encoder);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if(p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_motor->Hall); }
#endif
}

static inline void Motor_HallEncoderB_ISR(Motor_T * p_motor)
{
    Encoder_OnPhaseB_ISR(&p_motor->Encoder);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if(p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_motor->Hall); }
#endif
}

static inline void Motor_HallEncoderAB_ISR(Motor_T * p_motor)
{
    Encoder_OnPhaseAB_ISR(&p_motor->Encoder);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if(p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_motor->Hall); }
#endif
}

static inline void Motor_HallEncoderCZ_ISR(Motor_T * p_motor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_ENCODER:
            Encoder_OnIndex_ISR(&p_motor->Encoder);
            break;
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
        case MOTOR_SENSOR_MODE_HALL:
            Encoder_OnPhaseC_Hall_ISR(&p_motor->Encoder);
            Hall_CaptureAngle_ISR(&p_motor->Hall);
            break;
#endif
        default: break;
    }
}

#endif


