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
    @brief  Sensorless start-up substate tree under MOTOR_STATE_OPEN_LOOP.

    Parent OPEN_LOOP states own the actuation (align, ramp) and the observer
    drive. These substates only override the lifecycle decisions:
      SENSORLESS_ALIGN     — on align-timer expiry → SENSORLESS_START_UP
                             (parent would otherwise go to OPEN_LOOP_STATE_RUN).
      SENSORLESS_START_UP  — on observer lock     → MOTOR_STATE_RUN.
*/
/******************************************************************************/
#include "Sensorless_Sensor.h"
#include "../../StateMachine/Motor_StateMachine.h"
#include "../../StateMachine/Motor_OpenLoop.h"
#include "../RotorSensor.h"


/* Hand off to closed-loop once the observer reports lock. Parent
   OPEN_LOOP_STATE_RUN.LOOP (Motor_OpenLoop::Run_Proc) drives the observer. */
static State_T * RunSensorless_Next(Motor_T * p_motor)
{
    return RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) ? &MOTOR_STATE_RUN : NULL;
}

static State_T SENSORLESS_START_UP =
{
    .P_TOP    = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &OPEN_LOOP_STATE_RUN,
    .DEPTH    = 2U,
    .NEXT     = (State_Input0_T)RunSensorless_Next,
};


/* Override parent StartUpAlign_Next so timer expiry routes into the
   sensorless start-up substate instead of plain OPEN_LOOP_STATE_RUN. */
static State_T * SensorlessAlign_Next(Motor_T * p_motor)
{
    return (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true) ? &SENSORLESS_START_UP : NULL;
}

static State_T SENSORLESS_ALIGN =
{
    .P_TOP    = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &OPEN_LOOP_STATE_START_UP_ALIGN,
    .DEPTH    = 2U,
    .NEXT     = (State_Input0_T)SensorlessAlign_Next,
};


/******************************************************************************/
/*
    Entry points
*/
/******************************************************************************/
/* Start the sensorless align → run-up → closed-loop chain. */
void Motor_Sensorless_StartRunChain(Motor_T * p_motor)
{
    if (p_motor->P_MOTOR->Direction == MOTOR_DIRECTION_NULL) { return; }
    Phase_ActivateV0(&p_motor->PHASE);
    Motor_OpenLoop_EnterBranch(p_motor, &SENSORLESS_ALIGN);
}
