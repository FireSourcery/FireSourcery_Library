/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotorStateMachine.h
    @author FireSourcery
    @brief  MotorStateMachine
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

#include "Motor.h"
#include "Utility/StateMachine/StateMachine.h"

#define MSM_TRANSITION_TABLE_LENGTH     (7U)

/*
    Motor State Machine Inputs
*/
typedef enum Motor_StateMachine_Input
{
    MSM_INPUT_FAULT,            /* Toggle Fault */
    MSM_INPUT_CONTROL,          /* Spin - Activate Control / Feedback Mode - Transition to Run State (Active Control), and FeedbackMode update */
    MSM_INPUT_RELEASE,          /* Freewheel - Release Control */
    MSM_INPUT_HOLD,             /* Stop */
    MSM_INPUT_DIRECTION,
    MSM_INPUT_CALIBRATION,
}
Motor_StateMachine_Input_T;

/* INIT and OPEN_LOOP as passthrough. each other state has directly corresponding input */
typedef enum Motor_StateMachine_StateId
{
    MSM_STATE_ID_INIT,
    MSM_STATE_ID_STOP,
    MSM_STATE_ID_RUN,
    MSM_STATE_ID_FREEWHEEL,
    MSM_STATE_ID_OPEN_LOOP,
    MSM_STATE_ID_CALIBRATION,
    MSM_STATE_ID_FAULT,
}
Motor_StateMachine_StateId_T;

extern const StateMachine_Machine_T MSM_MACHINE;

#define MOTOR_STATE_MACHINE_INIT(p_Motor) STATE_MACHINE_INIT(&MSM_MACHINE, p_Motor, false)

extern void Motor_ActivateControl(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_ActivateControl_Cast(Motor_T * p_motor, uint8_t modeWord);

extern bool Motor_StateMachine_IsFault(const Motor_T * p_motor);
extern bool Motor_StateMachine_ExitFault(Motor_T * p_motor);
extern void Motor_StateMachine_EnterFault(Motor_T * p_motor);
// extern void Motor_PollAdcFaultFlags(Motor_T * p_motor);

#endif
