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

#include "StateMachine/Motor_StateMachine.h"
#include "Motor.h"

#include "Transducer/Encoder/Encoder_ISR.h"

#include "Framework/StateMachine/StateMachine_Thread.h"


/******************************************************************************/
/*
    Motor_MarkAnalog_Thread
*/
/******************************************************************************/
static inline void Motor_MarkAnalog_Thread(Motor_T * p_dev)
{
    if (Motor_IsAnalogCycle(p_dev) == true) { _Motor_Analog_Thread(p_dev); }
}



// static inline void Phase_WriteDuty_Thread(Phase_VOut_T * p_phase, uint16_t pwmA, uint16_t pwmB, uint16_t pwmC)
// {
//     Phase_Bitmask_T state = _Phase_ReadGates(p_phase);

//     if (state.A == 1U) { PWM_WriteDuty(&p_phase->PWM_A, pwmA); }
//     if (state.B == 1U) { PWM_WriteDuty(&p_phase->PWM_B, pwmB); }
//     if (state.C == 1U) { PWM_WriteDuty(&p_phase->PWM_C, pwmC); }
//     if (state.Bits != PHASE_ID_0) { _Phase_SyncPwmDuty(p_phase, state.Bits); }
// }

/******************************************************************************/
/*
    Motor_PWM_Thread
*/
/******************************************************************************/
/*
    ~50us
    Calling function clears interrupt flag
*/
static inline void Motor_PWM_Thread(Motor_T * p_dev)
{
    Motor_State_T * p_fields = p_dev->P_MOTOR;
    Motor_CaptureSensor(p_dev);

    _Motor_StateMachine_Thread(&p_dev->STATE_MACHINE);
    // StateMachine_Synchronous_RootFirst_Thread(&p_dev->STATE_MACHINE);

    /* Inline Phase Out, use common buffered values.. */
    /* Directly read register state */
    // if (!Phase_IsFloat(&p_dev->PHASE)) { Motor_FOC_WriteDuty(p_dev); } /* all substate must write to interface */
    /* state machine handles write gate state */
    // Motor_FOC_WriteDuty_Thread(p_dev);
    /* alternatively state machine calls functions mapped to compiled in this layer */

    p_fields->ControlTimerBase++;
}




/* Controls StateMachine Proc. Local Critical */
// static inline void Motor_ClearInterrupt(Motor_T * p_motor) { Phase_ClearInterrupt(&p_motor->PHASE); }
// static inline void Motor_DisableInterrupt(Motor_T * p_motor) { Phase_DisableInterrupt(&p_motor->PHASE); }
// static inline void Motor_EnableInterrupt(Motor_T * p_motor) { Phase_EnableInterrupt(&p_motor->PHASE); }

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Per-motor winding thermal derate.
*/
static inline void Motor_Heat_Thread(Motor_T * p_dev)
{
    switch (HeatMonitor_Poll(&p_dev->HEAT_MONITOR, Analog_Conversion_GetResult(&p_dev->HEAT_MONITOR_CONVERSION)))
    {
        // case HEAT_MONITOR_STATUS_NORMAL:
        // case HEAT_MONITOR_STATUS_WARNING_HIGH:
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT: Motor_StateMachine_SetFault(p_dev, MOTOR_FAULT_OVERHEAT); break;
        default: break;
    }

    if (Monitor_IsEnabled(p_dev->HEAT_MONITOR.P_STATE) == true) { Analog_Conversion_Mark(&p_dev->HEAT_MONITOR_CONVERSION); }
}

