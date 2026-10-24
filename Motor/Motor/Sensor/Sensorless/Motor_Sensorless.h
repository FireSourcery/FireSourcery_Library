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
    @file   Motor_Sensorless.h
    @author FireSourcery
*/
/******************************************************************************/
#include "../../Motor.h"
#include "Math/Fixed/fract16.h"
#include "Sensorless_Sensor.h"

#include "../RotorSensor.h"
#include "../../Motor_StateMachine.h"
#include "../../StateMachine/Motor_OpenLoop.h"
#include "../../Motor_FOC.h"

#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
/*
    Start the sensorless align → ramp → closed-loop handoff chain.
    Enters MOTOR_STATE_OPEN_LOOP at the SENSORLESS_ALIGN substate; the substate
    chain transitions to MOTOR_STATE_RUN once the observer reports lock.
    No-op unless Direction is set and sensorless is the selected sensor.
*/
extern void Motor_Sensorless_StartRunChain(Motor_T * p_motor);
extern State_T * Motor_Sensorless_StartUpState(Motor_T * p_motor);

/* The observer before the FOC proc. Takes the device as a state action does, for an ON_PHASE action table */
extern void Motor_Sensorless_CaptureStator(Motor_T * p_motor);
extern void Motor_Sensorless_InitWinding(Motor_T * p_motor);
#endif

/******************************************************************************/
/*
    Feedback lost: a NEXT for the states that drive on the sensor's angle. RUN, INTERVENTION, or a sensor's substate of them.
    Hall and SinCos never report it, Encoder holds its alignment, the observer releases its lock [AnglePll_FilterTuning_T].
*/
/******************************************************************************/
static State_T * _Motor_TransitionFeedbackLost(Motor_T * p_motor)
{
    if (RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) == true) { return NULL; }
    if (Motor_IsSpeedFreewheelLimitRange(p_motor) == true) { return &MOTOR_STATE_PASSIVE; }
    p_motor->P_MOTOR->FaultFlags.PositionSensor = 1U;
    return &MOTOR_STATE_FAULT;
}

