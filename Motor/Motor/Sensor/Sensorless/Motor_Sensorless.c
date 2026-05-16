#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Motor_Sensorless.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Sensorless_Sensor.h"
#include "../../StateMachine/Motor_StateMachine.h"
#include "../../StateMachine/Motor_OpenLoop.h"
#include "../RotorSensor.h"

// #include "../Math/FOC_Sensorless.h"

static void RunSensorless_Proc(Motor_T * p_motor)
{
    Motor_Context_T * p_state = p_motor->P_MOTOR;
    if (FOC_CaptureIabc(&p_state->Foc, &p_state->PhaseInput.I) == true)
    {
        FOC_Sensorless_Step(&p_state->FocSensorless, p_state->Foc.Id, p_state->Foc.Iq);
    }
    FOC_ProcInvClarkePark(&p_state->Foc);
    FOC_Sensorless_CaptureVoltage(&p_state->FocSensorless, p_state->Foc.Vd, p_state->Foc.Vq);
}

State_T * RunSensorless_Next(Motor_T * p_motor)
{
    // return (RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) == true) ? &MOTOR_STATE_RUN : NULL;
    return NULL; // debug
}

static State_T SENSORLESS_START_UP =
{
    // .ID         = MOTOR_STATE_ID_OPEN_LOOP,
    .P_PARENT = &OPEN_LOOP_STATE_RUN,
    .DEPTH = 2U,
    // .ENTRY      = (State_Action_T)Run_Entry,
    .LOOP = (State_Action_T)RunSensorless_Proc,
    .NEXT = (State_Input0_T)RunSensorless_Next,
};



State_T * SensorlessAlign_Next(Motor_T * p_motor)
{
    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        return &SENSORLESS_START_UP;
    }
    return NULL;
}


static State_T SENSORLESS_ALIGN =
{
    // .ID         = MOTOR_STATE_ID_OPEN_LOOP,
    .P_PARENT = &OPEN_LOOP_STATE_START_UP_ALIGN,
    .DEPTH = 2U,
    // .ENTRY      = (State_Action_T)Run_Entry,
    // .LOOP = (State_Action_T)RunSensorless_Proc,
    .NEXT = (State_Input0_T)SensorlessAlign_Next,
};