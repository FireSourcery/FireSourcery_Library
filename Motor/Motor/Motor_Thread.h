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
static inline void Motor_PollSpeed(Motor_T * p_dev)
{
    if (TimerT_Periodic_Poll(&p_dev->SPEED_TIMER) == true)
    {
        RotorSensor_CaptureSpeed(p_dev->P_MOTOR->p_ActiveSensor);
        _StateMachine_ActionInput(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev, MOTOR_STATE_INPUT_ON_SPEED); /* No critical section. Same thread as ISR */
        // _StateMachine_ActionInputRootState(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev, MOTOR_STATE_INPUT_ON_SPEED);
    }
}

static inline void Motor_CaptureSensor(Motor_T * p_dev)
{
    RotorSensor_CaptureAngle(p_dev->P_MOTOR->p_ActiveSensor);
    // if (Motor_IsAnalogCycle(p_dev) == true) { Motor_PollSpeed(p_dev); } /* With the requests. The 1 ms poll lands on a fixed cycle, delayed at most 1 */
}

/*
    After the state machine: the sensor's take of the stator, from the motor's Foc [FOC_T], which the sensor holds. Every control tick, in every state.
    The sensorless observer corrects here; a sensor reading the rotor directly does nothing [RotorSensor_CaptureStatorNone].
    Constraints on the caller, valid only when they hold:
        Once per control tick, after the state's write [Motor_FOC_WriteDuty]: Foc holds this tick's take, Ialpha/Ibeta, Id/Iq, and the command just written, Valpha/Vbeta.
        The take Parked at the published θ̂: the observer reads Id/Iq in its own frame.
        PhaseInput.I is a set sampled while driven [Motor_ADC_MarkPhases].
    Not held, the observer corrects on what Foc holds:
        PASSIVE floating: Id/Iq 0 after the feedback clear [Motor_FOC_ClearFeedbackState], Valpha/Vbeta the measured terminal voltage [FOC_CaptureVBemf]: i and v as they are.
        PASSIVE shorted, FAULT, DEACTIVATED, INIT: no V capture, no write. Valpha/Vbeta the last measured or written, Id/Iq 0 after the feedback clear.
        A state writing the duty other than through the FOC leaves Foc's command stale.
*/
static inline void Motor_CaptureStator(Motor_T * p_dev)
{
    RotorSensor_CaptureStator(p_dev->P_MOTOR->p_ActiveSensor);
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

    _Motor_StateMachine_Thread(&p_dev->STATE_MACHINE); // _Motor_StateMachine(p_dev->STATE_MACHINE.P_ACTIVE, p_dev);
    // Motor_FOC_WriteDuty_Thread(p_dev); // okay if statemachine transitions for Duty0,0,0 state

    Motor_CaptureStator(p_dev);

    Motor_PollSpeed(p_dev);
    p_dev->P_MOTOR->ControlTimerBase++;
}

/******************************************************************************/
/*

*/
/******************************************************************************/
// static inline void Motor_MarkAnalog_Thread(Motor_T * p_dev)
// {
//     if (Motor_IsAnalogCycle(p_dev) == true) { Motor_ADC_MarkPhases(p_dev); }
// }

/*
    The I set's completion, the set already captured, e.g. by per-channel pushes. From the ADC or DMA ISR.
    Run at the PWM ISR's priority, so the active state is not mid-transition. ON_PHASE reaches the active leaf state only.
    The set stays raw: the state's take Parks it, or its ON_PHASE action does [Motor_FOC_CaptureI].
*/
// static inline void _Motor_OnIabc_Thread(Motor_T * p_motor)
// {
//     _StateMachine_ActionInput(p_motor->STATE_MACHINE.P_ACTIVE, (void *)p_motor, MOTOR_STATE_INPUT_ON_PHASE);
// }

// /* Low-side sensing's I set of ADCU [PHASE_ANALOG_I_SENSOR_LOW_SIDE]: [phases] the phases converted, the third solved [Phase_Analog_CaptureIBatch] */
// static inline void Motor_ADC_OnIBatch_Thread(Motor_T * p_motor, int32_t a_adcu, int32_t b_adcu, int32_t c_adcu, Phase_Id_T phases)
// {
//     Motor_Analog_CaptureIBatch(p_motor->P_MOTOR, a_adcu, b_adcu, c_adcu, phases);
//     _Motor_OnIabc_Thread(p_motor);
// }

// static inline void Motor_OnIabc_Thread(Motor_T * p_motor, int32_t a, int32_t b, int32_t c)
// {
//     p_motor->P_MOTOR->PhaseInput.I = (Phase_Data_T) { .Values = { .A = a, .B = b, .C = c }, .Flags.Bits = PHASE_ID_ABC, };
//     _Motor_OnIabc_Thread(p_motor);
// }

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

