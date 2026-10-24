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
// #include "Motor_Debug.h"

#include "Analog/Motor_Analog.h"
#include "Motor_StateMachine.h"
#include "Motor.h"

#include "Framework/StateMachine/StateMachine_Thread.h"
#include "Phase/Phase_Types.h"



/******************************************************************************/
/*
    Motor_PWM_Thread
*/
/******************************************************************************/
/*
    Capture State
*/
static inline void Motor_CaptureSensor(Motor_T * p_dev)
{
    RotorSensor_CaptureAngle(p_dev->P_MOTOR->p_ActiveSensor);
    if (TimerT_Periodic_Poll(&p_dev->SPEED_TIMER) == true)
    {
        RotorSensor_CaptureSpeed(p_dev->P_MOTOR->p_ActiveSensor);
        _StateMachine_ActionInput(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev, MOTOR_STATE_INPUT_ON_SPEED); /* No critical section. Same thread as ISR */
    }
}


/*
    ~50us
    Calling function clears interrupt flag
*/
/*
    Caller's compilation unit sees Motor_T as static const. further optimizes loads from [Motor_T].
    HAL calls inside StateMachine still optimizes register accesses.
*/
static inline void Motor_PWM_Thread(Motor_T * p_dev)
{
    Motor_CaptureSensor(p_dev);

    _Motor_StateMachine_Thread(&p_dev->STATE_MACHINE);
    // _Motor_StateMachine(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev);
    // StateMachine_Synchronous_RootFirst_Thread(&p_dev->STATE_MACHINE);
    // Motor_FOC_WriteDuty_Thread(p_dev);

    p_dev->P_MOTOR->ControlTimerBase++;
}

/******************************************************************************/
/*
    Motor_MarkAnalog_Thread
*/
/******************************************************************************/
static inline void Motor_MarkAnalog_Thread(Motor_T * p_dev)
{
    if (Motor_IsAnalogCycle(p_dev) == true) { _Motor_Analog_Thread(p_dev); }
}

static inline void Motor_ADC_OnIabc_Thread(Motor_T * p_motor, adc_result_t a, adc_result_t b, adc_result_t c)
{
    Motor_Context_T * p_state = p_motor->P_MOTOR;
    Motor_Analog_CaptureIabc(p_state, a, b, c);
    _StateMachine_ActionInput(p_motor->STATE_MACHINE.P_ACTIVE, (void *)p_motor, MOTOR_STATE_INPUT_ON_PHASE);
}

static inline void Motor_OnIabc_Thread(Motor_T * p_motor, int32_t a, int32_t b, int32_t c)
{
    Motor_Context_T * p_state = p_motor->P_MOTOR;
    p_state->PhaseInput.I = (Phase_Data_T) { .Values = { .A = a, .B = b, .C = c }, .Flags.Bits =  PHASE_ID_ABC, };
    _StateMachine_ActionInput(p_motor->STATE_MACHINE.P_ACTIVE, (void *)p_motor, MOTOR_STATE_INPUT_ON_PHASE);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Per-motor winding thermal derate.
*/
static inline void Motor_Heat_Thread(Motor_T * p_dev)
{
    switch (HeatMonitor_Poll(&p_dev->HEAT_MONITOR, ADC_Conversion_GetResult(&p_dev->HEAT_MONITOR_CONVERSION)))
    {
        // case HEAT_MONITOR_STATUS_NORMAL:
        // case HEAT_MONITOR_STATUS_WARNING_HIGH:
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT: Motor_StateMachine_SetFault(p_dev, MOTOR_FAULT_OVERHEAT); break;
        default: break;
    }

    if (Monitor_IsEnabled(p_dev->HEAT_MONITOR.P_STATE) == true) { ADC_Conversion_Mark(&p_dev->HEAT_MONITOR_CONVERSION); }
}


/******************************************************************************/
/*
*/
/******************************************************************************/


/* Controls StateMachine Proc. Local Critical */
// static inline void Motor_ClearInterrupt(Motor_T * p_motor) { Phase_ClearInterrupt(&p_motor->PHASE); }
// static inline void Motor_DisableInterrupt(Motor_T * p_motor) { Phase_DisableInterrupt(&p_motor->PHASE); }
// static inline void Motor_EnableInterrupt(Motor_T * p_motor) { Phase_EnableInterrupt(&p_motor->PHASE); }

