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


static inline bool Motor_IsAnalogCycle(const Motor_T * p_context) { return MotorTimeRef_IsAnalogCycle(p_context->P_ACTIVE->ControlTimerBase); }


/*
    Default 50us
    Calling function clears interrupt flag
*/
static inline void Motor_PWM_Thread(const Motor_T * p_context)
{
    Motor_State_T * p_fields = p_context->P_ACTIVE;

    // Motor_Debug_CaptureRefTime(p_context);
    if (Timer_Periodic_Poll(&p_fields->SpeedTimer) == true) { MotorSensor_CaptureSpeed(p_fields->p_ActiveSensor); }
    MotorSensor_CaptureAngle(p_fields->p_ActiveSensor);

    // StateMachine_ProcState(&p_context->STATE_MACHINE); /* todo inline */
    StateMachine_Synchronous_Thread(&p_context->STATE_MACHINE);

    /* phase out here can inline */
    // switch ((Phase_Output_T)phaseOutput)
    // {
    //     case PHASE_OUTPUT_FLOAT: break;
    //     case PHASE_OUTPUT_V0:    break;
    //     case PHASE_OUTPUT_VPWM: Motor_FOC_WriteDuty(&p_context->PHASE, &p_fields->Foc); break;
    //     default: break;
    // }
    // if (StateMachine_GetActiveStateId(&p_fields->StateMachine) == MSM_STATE_ID_RUN)
    if (StateMachine_GetActiveStateId(p_context->STATE_MACHINE.P_ACTIVE) == MSM_STATE_ID_RUN)
    {
        Motor_FOC_WriteDuty(&p_context->PHASE, &p_fields->Foc);
    }

    p_fields->ControlTimerBase++;

#ifdef CONFIG_MOTOR_PWM_INTERRUPT_CLEAR_PER_MOTOR
    Motor_ClearInterrupt(p_context);
#endif
}

/* Optionally mark on Start */
static inline void _Motor_MarkAnalog_Thread(const Motor_T * p_context)
{
    // MotorSensor_MarkAnalog(&p_context->Sensor);
    switch (StateMachine_GetActiveStateId(&p_context->P_ACTIVE->StateMachine)) /* or set on state entry */
    {
        case MSM_STATE_ID_STOP:         Motor_Analog_MarkVabc(p_context);     break;
            // case MSM_STATE_ID_FREEWHEEL:    Motor_Analog_MarkVabc(p_context);     break;
        case MSM_STATE_ID_PASSIVE:      Motor_Analog_MarkVabc(p_context);     break;
        case MSM_STATE_ID_RUN:          Motor_Analog_MarkIabc(p_context);     break;
            // #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
        case MSM_STATE_ID_OPEN_LOOP:    Motor_Analog_MarkIabc(p_context);     break;
            // #endif
        case MSM_STATE_ID_FAULT:        Motor_Analog_MarkVabc(p_context);     break;
            // case MSM_STATE_ID_CALIBRATION:  Motor_Analog_MarkIabc(p_context);     break;
        case MSM_STATE_ID_CALIBRATION:      break;
        case MSM_STATE_ID_INIT:             break;
            // case MSM_STATE_ID_FAULT:     Motor_Analog_MarkVabc(p_context); Motor_Analog_MarkIabc(p_context); break;
        default:            break;
    }
    //    switch(p_context->AnalogCmd)
    //    {
    //        case FOC_I: break;
    //        case FOC_V: break;
    //        default: break;
    //    }
}

static inline void Motor_MarkAnalog_Thread(const Motor_T * p_context)
{
    // todo change timer
    if (Motor_IsAnalogCycle(p_context) == true) { _Motor_MarkAnalog_Thread(p_context); }
}

/* Alternatively move singleton capture to this module */
// static inline void Motor_CaptureVSource(uint16_t vBus)
// {
//     MotorAnalog_CaptureVSource_Adcu(vBus);
// }

/* Caller handle I Limit HeatMonitor_GetScalarLimit_Percent16(&p_motor->Thermistor) */
static inline void Motor_Heat_Thread(const Motor_T * p_context)
{
    // if (HeatMonitor_IsEnabled(&p_context->P_ACTIVE->Thermistor) == true)
    // {
    //     // if (HeatMonitor_PollMonitor_Edge(&p_motor->Thermistor, Motor_Analog_GetHeat(p_motor)))
    //     // {
    //     //     if (HeatMonitor_IsFault(&p_motor->Thermistor) == true)
    //     //     {
    //     //         p_motor->FaultFlags.Overheat = 1U;
    //     //         Motor_StateMachine_EnterFault(p_motor);
    //     //     }
    //     // }

    //     // switch (HeatMonitor_PollMonitor(&p_motor->Thermistor, Motor_Analog_GetHeat(p_motor)))
    //     // {
    //     //     case HEAT_MONITOR_STATUS_OK:
    //     //         if (p_motor->StateFlags.HeatWarning == 1U) /* todo move to thermistor */
    //     //         {
    //     //             p_motor->StateFlags.HeatWarning = 0U;
    //     //             // Motor_ClearILimitMotoringEntry(p_motor, MOTOR_I_LIMIT_HEAT_THIS);
    //     //         }
    //     //         break;
    //     //     case HEAT_MONITOR_STATUS_WARNING_THRESHOLD:
    //     //     case HEAT_MONITOR_STATUS_WARNING:     /* repeatedly checks if heat is a lower ILimit when another ILimit is active */
    //     //         p_motor->StateFlags.HeatWarning = 1U;
    //     //         // Motor_SetILimitMotoringEntry_Scalar(p_motor, MOTOR_I_LIMIT_HEAT_THIS, HeatMonitor_GetScalarLimit_Percent16(&p_motor->Thermistor));
    //     //         break;
    //     //     case HEAT_MONITOR_STATUS_FAULT_THRESHOLD:
    //     //     case HEAT_MONITOR_STATUS_FAULT:
    //     //         p_motor->FaultFlags.Overheat = 1U;
    //     //         Motor_StateMachine_EnterFault(p_motor);
    //     //         break;
    //     //     default: break;
    //     // }

    //     // Analog_MarkConversion(&p_context->ANALOG.CONVERSION_HEAT);
    // }
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
// static inline void Motor_HallEncoderA_ISR(Motor_State_T * p_motor)
// {
//     Encoder_OnPhaseA_ISR(&p_motor->Encoder);
// #if defined(CONFIG_MOTOR_HALL_MODE_ISR)
//     if(p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_motor->Hall); }
// #endif
// }

// static inline void Motor_HallEncoderB_ISR(Motor_State_T * p_motor)
// {
//     Encoder_OnPhaseB_ISR(&p_motor->Encoder);
// #if defined(CONFIG_MOTOR_HALL_MODE_ISR)
//     if(p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_motor->Hall); }
// #endif
// }

// static inline void Motor_HallEncoderAB_ISR(Motor_State_T * p_motor)
// {
//     Encoder_OnPhaseAB_ISR(&p_motor->Encoder);
// #if defined(CONFIG_MOTOR_HALL_MODE_ISR)
//     if(p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_HALL) { Hall_CaptureAngle_ISR(&p_motor->Hall); }
// #endif
// }

// static inline void Motor_HallEncoderCZ_ISR(Motor_State_T * p_motor)
// {
//     switch(p_motor->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_ENCODER:
//             Encoder_OnIndex_ISR(&p_motor->Encoder);
//             break;
// #if defined(CONFIG_MOTOR_HALL_MODE_ISR)
//         case MOTOR_SENSOR_MODE_HALL:
//             Encoder_OnPhaseC_Hall_ISR(&p_motor->Encoder);
//             Hall_CaptureAngle_ISR(&p_motor->Hall);
//             break;
// #endif
//         default: break;
//     }
// }

