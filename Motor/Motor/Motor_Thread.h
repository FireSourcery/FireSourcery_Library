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
    if (Motor_IsAnalogCycle(p_dev) == true) { _Motor_Analog_Thread(p_dev); }
}

/******************************************************************************/
/*
    Motor_PWM_Thread
*/
/******************************************************************************/
/*
    ~50us
    Calling function clears interrupt flag
*/
static inline void Motor_PWM_Thread(const Motor_T * p_dev)
{
    Motor_State_T * p_fields = p_dev->P_MOTOR;
    Motor_CaptureSensor(p_dev);

    _Motor_StateMachine_Thread(&p_dev->STATE_MACHINE);
    // StateMachine_Synchronous_RootFirst_Thread(&p_dev->STATE_MACHINE);

    /* Inline Phase Out, use common buffered values.. */
    /* Directly read register state */
    // if (!Phase_IsFloat(&p_dev->PHASE)) { Motor_FOC_WriteDuty(p_dev); } /* all substate must write to interface */
    // Phase_WriteDuty_Thread(&p_dev->PHASE, motor->dutytriplet);

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
/*
    Per-motor winding thermal derate. Writes to motor-local arrays so one motor's
    heat does not broadcast to peers; composition into the effective cap happens
    inside Motor_Table_Apply* at the controller tick.
*/
static inline void Motor_Heat_Thread(const Motor_T * p_dev)
{
    switch (HeatMonitor_Poll(&p_dev->HEAT_MONITOR, Analog_Conversion_GetResult(&p_dev->HEAT_MONITOR_CONVERSION)))
    {
        case HEAT_MONITOR_STATUS_NORMAL:
            if (Monitor_IsStatusClearing(p_dev->HEAT_MONITOR.P_STATE) == true)
            {
                LimitArray_TestClearEntry(&p_dev->I_LIMITS_LOCAL, MOTOR_I_LIMIT_HEAT_WINDING);
                LimitArray_TestClearEntry(&p_dev->I_GEN_LIMITS_LOCAL, MOTOR_I_GEN_LIMIT_HEAT_WINDING);
            }
            break;
        case HEAT_MONITOR_STATUS_WARNING_HIGH:
            {
                uint16_t i_limit = fract16_mul(HeatMonitor_GetScalarLimit_Percent16(&p_dev->HEAT_MONITOR) / 2, Phase_Calibration_GetIRatedPeak_Fract16());
                LimitArray_TestSetEntry(&p_dev->I_LIMITS_LOCAL, MOTOR_I_LIMIT_HEAT_WINDING, i_limit);
                LimitArray_TestSetEntry(&p_dev->I_GEN_LIMITS_LOCAL, MOTOR_I_GEN_LIMIT_HEAT_WINDING, i_limit);
                break;
            }
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT:
            Motor_StateMachine_SetFault(p_dev, MOTOR_FAULT_OVERHEAT);
            break;
        default: break;
    }

    if (Monitor_IsEnabled(p_dev->HEAT_MONITOR.P_STATE) == true) { Analog_Conversion_Mark(&p_dev->HEAT_MONITOR_CONVERSION); }
}

