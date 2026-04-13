#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @brief  Motor module functions to be placed into corresponding user app threads
*/
/******************************************************************************/
#include "Motor_Debug.h"

#include "Analog/Motor_Analog.h"

#include "Motor_StateMachine.h"
#include "Motor.h"

#include "Transducer/Encoder/Encoder_ISR.h"

#include "Utility/StateMachine/StateMachine_Thread.h"


/******************************************************************************/
/*
    Motor_MarkAnalog_Thread
*/
/******************************************************************************/
static inline void Motor_MarkAnalog_Thread(const Motor_T * p_dev)
{
    // or caller handle offset
    if (Motor_IsAnalogCycle(p_dev) == true) { _Motor_Analog_Thread(p_dev); }
}

/******************************************************************************/
/*
    Motor_PWM_Thread
*/
/******************************************************************************/



/*
    Default 50us
    Calling function clears interrupt flag
*/
static inline void Motor_PWM_Thread(const Motor_T * p_dev)
{
    Motor_State_T * p_fields = p_dev->P_MOTOR_STATE;

    // p_fields->MicrosRef = SysTime_GetMicros();

    Motor_CaptureSensor(p_dev);

    StateMachine_Synchronous_RootFirst_Thread(&p_dev->STATE_MACHINE);

    /* Inline Phase Out, use common buffered values.. */
    /* Directly read register state */
    // if (!Phase_IsFloat(&p_dev->PHASE)) { Motor_FOC_WriteDuty(p_dev); } /* all substate must write to interface */
    // Phase_WriteDuty_Fract16_Thread(&p_dev->PHASE, FOC_DutyA(&p_fields->Foc), FOC_DutyB(&p_fields->Foc), FOC_DutyC(&p_fields->Foc));

     p_fields->ControlTimerBase++;
}

/* Controls StateMachine Proc. Local Critical */
// static inline void Motor_ClearInterrupt(const Motor_T * p_motor) { Phase_ClearInterrupt(&p_motor->PHASE); }
// static inline void Motor_DisableInterrupt(const Motor_T * p_motor) { Phase_DisableInterrupt(&p_motor->PHASE); }
// static inline void Motor_EnableInterrupt(const Motor_T * p_motor) { Phase_EnableInterrupt(&p_motor->PHASE); }

/******************************************************************************/
/*

*/
/******************************************************************************/
/* Caller handle I Limit HeatMonitor_GetScalarLimit_Percent16(&p_motor->Thermistor) */
static inline void Motor_Heat_Thread(const Motor_T * p_dev)
{
    switch (HeatMonitor_Poll(&p_dev->HEAT_MONITOR, Analog_Conversion_GetResult(&p_dev->HEAT_MONITOR_CONVERSION)))
    {
        case HEAT_MONITOR_STATUS_NORMAL:
            if (Monitor_IsStatusClearing(p_dev->HEAT_MONITOR.P_STATE) == true)
            {
                // Motor_ClearILimitMotoringEntry(p_dev, MOTOR_I_LIMIT_HEAT_THIS);
            }
            break;
        case HEAT_MONITOR_STATUS_WARNING_HIGH:  /* repeatedly checks if heat is a lower ILimit when another ILimit is active */
            // Motor_SetILimitMotoringEntry_Scalar(p_dev, MOTOR_I_LIMIT_HEAT_THIS, HeatMonitor_GetScalarLimit_Percent16(&p_dev->Thermistor));
            break;
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT:
            Motor_StateMachine_SetFault(p_dev, MOTOR_FAULT_OVERHEAT);
            break;
        default: break;
    }

    if (Monitor_IsEnabled(p_dev->HEAT_MONITOR.P_STATE) == true) { Analog_Conversion_Mark(&p_dev->HEAT_MONITOR_CONVERSION); }
}

/******************************************************************************/
/*
    Sensor ISR
*/
/******************************************************************************/
/* Optionally use Hall ISR */
static inline void Motor_HallEncoderA_ISR(const Motor_T * p_dev)
{
    Encoder_OnPhaseA_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR_STATE->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderB_ISR(const Motor_T * p_dev)
{
    Encoder_OnPhaseB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR_STATE->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderAB_ISR(const Motor_T * p_dev)
{
    Encoder_OnPhaseAB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR_STATE->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderCZ_ISR(const Motor_T * p_dev)
{
    switch (p_dev->P_MOTOR_STATE->Config.SensorMode)
    {
        case ROTOR_SENSOR_ID_ENCODER:
            Encoder_OnIndex_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
            break;
        #if defined(MOTOR_HALL_MODE_ISR)
        case ROTOR_SENSOR_ID_HALL:
            Encoder_OnPhaseC_Hall_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
            Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL);
            break;
        #endif
        default: break;
    }
}

