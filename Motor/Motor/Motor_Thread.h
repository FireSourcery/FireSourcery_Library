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
#include "Motor.h"

#include "Transducer/Encoder/Encoder_ISR.h"

/*
    Default 50us
    Calling function clears interrupt flag
*/
static inline void Motor_PWM_Thread(MotorPtr_T p_motor)
{
    // Motor_Debug_CaptureRefTime(p_motor);
    p_motor->ControlTimerBase++;
    // Motor_Analog_Proc(p_motor);
    StateMachine_Async_ProcState(&p_motor->StateMachine);
    // Motor_Debug_CaptureTime(p_motor, 5U);
}

static inline void Motor_Heat_Thread(MotorPtr_T p_motor)
{
    if(Thermistor_GetIsMonitorEnable(&p_motor->Thermistor) == true)
    {
        AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_HEAT);

        switch(Thermistor_PollMonitor(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu))
        {
            case THERMISTOR_STATUS_OK:
                p_motor->StatusFlags.HeatWarning = 0U;
                Motor_User_ClearILimitActive_Id(p_motor, MOTOR_I_LIMIT_ACTIVE_HEAT_THIS);
                break;
            case THERMISTOR_STATUS_WARNING:     /* repeatedly checks if heat is a lower ILimit when another ILimit is active */
                p_motor->StatusFlags.HeatWarning = 1U;
                Motor_User_SetILimitActive_Id(p_motor, Thermistor_GetHeatLimit_FracU16(&p_motor->Thermistor), MOTOR_I_LIMIT_ACTIVE_HEAT_THIS);
                break;
            case THERMISTOR_STATUS_FAULT:
                p_motor->FaultFlags.Overheat = 1U;
                Motor_User_SetFault(p_motor);
                break;
            default: break;
        }
    }
}


/* Optionally use Hall ISR */
static inline void Motor_HallEncoderA_ISR(MotorPtr_T p_motor)
{
    Encoder_OnPhaseA_ISR(&p_motor->Encoder);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureRotorAngle_ISR(&p_motor->Hall); }
#endif
}

static inline void Motor_HallEncoderB_ISR(MotorPtr_T p_motor)
{
    Encoder_OnPhaseB_ISR(&p_motor->Encoder);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureRotorAngle_ISR(&p_motor->Hall); }
#endif
}

static inline void Motor_HallEncoderAB_ISR(MotorPtr_T p_motor)
{
    Encoder_OnPhaseAB_ISR(&p_motor->Encoder);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureRotorAngle_ISR(&p_motor->Hall); }
#endif
}

static inline void Motor_HallEncoderCZ_ISR(MotorPtr_T p_motor)
{
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_ENCODER:
            Encoder_OnIndex_ISR(&p_motor->Encoder);
            break;
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
        case MOTOR_SENSOR_MODE_HALL:
            Encoder_OnPhaseC_Hall_ISR(&p_motor->Encoder);
            Hall_CaptureRotorAngle_ISR(&p_motor->Hall);
            break;
#endif
        default: break;
    }
}
#endif


