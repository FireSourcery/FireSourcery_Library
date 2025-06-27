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
#include "Analog/Motor_Analog.h"
#include "Motor_StateMachine.h"
// #include "Motor_User.h"
#include "Motor_Debug.h"
#include "Motor.h"

#include "Transducer/Encoder/Encoder_ISR.h"

#include "Utility/StateMachine/StateMachine_Thread.h"


static inline bool Motor_IsAnalogCycle(const Motor_T * p_context) { return MotorTimeRef_IsAnalogCycle(p_context->P_MOTOR_STATE->ControlTimerBase); }

/* Alternatively move singleton capture to this module */
// static inline void Motor_CaptureVSource(uint16_t vBus_adcu) { MotorAnalog_CaptureVSource_Adcu(vBus_adcu); }


/*
    Default 50us
    Calling function clears interrupt flag
*/
static inline void Motor_PWM_Thread(const Motor_T * p_context)
{
    Motor_State_T * p_fields = p_context->P_MOTOR_STATE;

    // Motor_Debug_CaptureRefTime(p_context);

    /* Alternatively capture */
    // if (Timer_Periodic_Poll(&p_fields->SpeedTimer) == true)
    // {
    //     MotorSensor_CaptureSpeed(p_fields->p_ActiveSensor);
    //     if (StateMachine_GetActiveStateId(p_context->STATE_MACHINE.P_MOTOR_STATE) == MSM_STATE_ID_RUN)
    //     {
    //         _Motor_ProcOuterFeedback(p_fields);
    //     }
    // }
    // MotorSensor_CaptureAngle(p_fields->p_ActiveSensor);

    // StateMachine_ProcState(&p_context->STATE_MACHINE);
    StateMachine_Synchronous_Thread(&p_context->STATE_MACHINE);

    /* phase out here can inline */
    /* Directly read register state */
    if (Phase_ReadOutputState(&p_context->PHASE) == PHASE_OUTPUT_VPWM) { Motor_FOC_ActivateVPhase(p_context); }
    // Phase_WriteDuty_Fract16_Thread(&p_context->PHASE, FOC_GetDutyA(&p_fields->Foc), FOC_GetDutyB(&p_fields->Foc), FOC_GetDutyC(&p_fields->Foc));
    // if (StateMachine_GetActiveStateId(&p_fields->StateMachine) == MSM_STATE_ID_RUN)
    // if (StateMachine_GetActiveStateId(p_context->STATE_MACHINE.P_MOTOR_STATE) == MSM_STATE_ID_RUN)
    // {
    //     // Motor_FOC_WriteDuty(&p_context->PHASE, &p_fields->Foc);
    //     Motor_FOC_ActivateVPhase(p_context);
    // }


    p_fields->ControlTimerBase++;

#ifdef CONFIG_MOTOR_PWM_INTERRUPT_CLEAR_PER_MOTOR
    Motor_ClearInterrupt(p_context);
#endif
}

/* Optionally mark on Start */
static inline void _Motor_MarkAnalog_Thread(const Motor_T * p_context)
{
    // MotorSensor_MarkAnalog(&p_context->Sensor);
    switch (StateMachine_GetActiveStateId(&p_context->P_MOTOR_STATE->StateMachine)) /* or set on state entry */
    {
        case MSM_STATE_ID_STOP:         Motor_Analog_MarkVabc(p_context);     break;
            // case MSM_STATE_ID_FREEWHEEL:    Motor_Analog_MarkVabc(p_context);     break;
        case MSM_STATE_ID_PASSIVE:      Motor_Analog_MarkVabc(p_context);     break;
        case MSM_STATE_ID_RUN:          Motor_Analog_MarkIabc(p_context);     break;
            // #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
        case MSM_STATE_ID_OPEN_LOOP:    Motor_Analog_MarkIabc(p_context);     break;
            // #endif
        case MSM_STATE_ID_FAULT:        Motor_Analog_MarkVabc(p_context);     break;
            // case MSM_STATE_ID_CALIBRATION:  Motor_Analog_MarkIabc(p_context);     break;
        case MSM_STATE_ID_CALIBRATION:      break;
        case MSM_STATE_ID_INIT:             break;
            // case MSM_STATE_ID_FAULT:     Motor_Analog_MarkVabc(p_context); Motor_Analog_MarkIabc(p_context); break;
        default:            break;
    }
}



static inline void Motor_MarkAnalog_Thread(const Motor_T * p_context)
{
    // or called handle offset
    if (Motor_IsAnalogCycle(p_context) == true) { _Motor_MarkAnalog_Thread(p_context); }
}


/* Caller handle I Limit HeatMonitor_GetScalarLimit_Percent16(&p_motor->Thermistor) */
static inline void Motor_Heat_Thread(const Motor_T * p_context)
{
    switch (HeatMonitor_Poll(&p_context->HEAT_MONITOR_CONTEXT))
    {
        case HEAT_MONITOR_STATUS_NORMAL:
            if (Monitor_IsStatusClearing(p_context->HEAT_MONITOR_CONTEXT.P_STATE) == true)
            {
                // Motor_ClearILimitMotoringEntry(p_context, MOTOR_I_LIMIT_HEAT_THIS);
            }
            break;
        case HEAT_MONITOR_STATUS_WARNING_HIGH:  /* repeatedly checks if heat is a lower ILimit when another ILimit is active */
            // Motor_SetILimitMotoringEntry_Scalar(p_context, MOTOR_I_LIMIT_HEAT_THIS, HeatMonitor_GetScalarLimit_Percent16(&p_context->Thermistor));
            break;
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT:
            p_context->P_MOTOR_STATE->FaultFlags.Overheat = 1U;
            Motor_StateMachine_EnterFault(p_context); // Motor_StateMachine_SetFault(p_context, MOTOR_FAULT_FLAG_OVERHEAT);
            break;
        default: break;
    }

    if (Monitor_IsEnabled(p_context->HEAT_MONITOR_CONTEXT.P_STATE) == true) { HeatMonitor_MarkConversion(&p_context->HEAT_MONITOR_CONTEXT); }
}

/******************************************************************************/
/*
    Interrupts
*/
/******************************************************************************/
/* Controls StateMachine Proc. Local Critical */
// static inline void Motor_ClearInterrupt(const Motor_T * p_motor) { Phase_ClearInterrupt(&p_motor->PHASE); }
// static inline void Motor_DisableInterrupt(const Motor_T * p_motor) { Phase_DisableInterrupt(&p_motor->PHASE); }
// static inline void Motor_EnableInterrupt(const Motor_T * p_motor) { Phase_EnableInterrupt(&p_motor->PHASE); }


/* Optionally use Hall ISR */
static inline void Motor_HallEncoderA_ISR(const Motor_T * p_context)
{
    Encoder_OnPhaseA_ISR(&p_context->SENSOR_TABLE.ENCODER.ENCODER);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if (p_context->P_MOTOR_STATE->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_context->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderB_ISR(const Motor_T * p_context)
{
    Encoder_OnPhaseB_ISR(&p_context->SENSOR_TABLE.ENCODER.ENCODER);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if (p_context->P_MOTOR_STATE->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_context->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderAB_ISR(const Motor_T * p_context)
{
    Encoder_OnPhaseAB_ISR(&p_context->SENSOR_TABLE.ENCODER.ENCODER);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
    if (p_context->P_MOTOR_STATE->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_context->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderCZ_ISR(const Motor_T * p_context)
{
    switch (p_context->P_MOTOR_STATE->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_ENCODER:
            Encoder_OnIndex_ISR(&p_context->SENSOR_TABLE.ENCODER.ENCODER);
            break;
        #if defined(CONFIG_MOTOR_HALL_MODE_ISR)
        case MOTOR_SENSOR_MODE_HALL:
            Encoder_OnPhaseC_Hall_ISR(&p_context->SENSOR_TABLE.ENCODER.ENCODER);
            Hall_CaptureAngle_ISR(&p_context->SENSOR_TABLE.HALL.HALL);
            break;
        #endif
        default: break;
    }
}

