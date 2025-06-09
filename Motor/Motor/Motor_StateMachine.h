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
*/
/******************************************************************************/
#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

#include "Motor_Commutation.h"
#include "Motor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/StateMachine/_StateMachine.h"

#include "System/Critical/Critical.h"
#include "System/SysTime/SysTime.h"

#ifndef MOTOR_STATE_MACHINE_INIT_WAIT
#define MOTOR_STATE_MACHINE_INIT_WAIT (1500U) /* For 1S polling to run twice */
#endif

/*
    Motor State Machine Inputs
*/
typedef enum Motor_State_Input
{
    MSM_INPUT_FAULT,             /* Toggle Fault */
    MSM_INPUT_CONTROL_STATE,     /* Active/Release/Hold */
    MSM_INPUT_FEEDBACK_MODE,     /* [FeedbackMode] flags */
    // MSM_INPUT_FEEDBACK_CONTROL,
    MSM_INPUT_DIRECTION,
    MSM_INPUT_CALIBRATION,
    MSM_INPUT_OPEN_LOOP,
    // MSM_INPUT_USER_BUFFER,
}
Motor_State_Input_T;

typedef enum Motor_StateId
{
    MSM_STATE_ID_INIT,
    MSM_STATE_ID_STOP,
    MSM_STATE_ID_PASSIVE,
    MSM_STATE_ID_RUN,
    // MSM_STATE_ID_FREEWHEEL,
    MSM_STATE_ID_OPEN_LOOP,
    MSM_STATE_ID_CALIBRATION,
    MSM_STATE_ID_FAULT,
}
Motor_StateId_T;

/* extern for extension */
extern const State_T MOTOR_STATE_INIT;
extern const State_T MOTOR_STATE_STOP;
extern const State_T MOTOR_STATE_PASSIVE;
extern const State_T MOTOR_STATE_RUN;
extern const State_T MOTOR_STATE_OPEN_LOOP;
extern const State_T MOTOR_STATE_CALIBRATION;
extern const State_T MOTOR_STATE_FAULT;

extern const StateMachine_Machine_T MSM_MACHINE;

/* state machine context on Motor const handler */
#define MOTOR_STATE_MACHINE_INIT(p_MotorConst, p_MotorActive) STATE_MACHINE_INIT((p_MotorConst), &MSM_MACHINE, &((p_MotorActive)->StateMachine))

/* Wrap for interface */
// static inline bool _Motor_StateMachine_IsState(const Motor_State_T * p_motor, Motor_StateId_T stateId) { return (StateMachine_IsActiveStateId(&p_motor->StateMachine, (Motor_StateId_T)stateId)); }
// static inline bool Motor_StateMachine_IsState(const Motor_State_T * p_motor, int stateId) { return _Motor_StateMachine_IsState(p_motor, (Motor_StateId_T)stateId); }
static inline bool Motor_StateMachine_IsState(const Motor_T * p_motor, Motor_StateId_T stateId) { return (StateMachine_IsActiveStateId(p_motor->STATE_MACHINE.P_ACTIVE, (Motor_StateId_T)stateId)); }

// bool _Motor_StateMachine_IsFault(const Motor_State_T * p_fields) { return (StateMachine_GetActiveStateId(&p_fields->StateMachine) == MSM_STATE_ID_FAULT); }
static inline bool Motor_StateMachine_IsFault(const Motor_T * p_motor) { return Motor_StateMachine_IsState(p_motor, MSM_STATE_ID_FAULT); }

/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Motor_StateMachine_EnterFault(const Motor_T * p_motor);
extern bool Motor_StateMachine_ExitFault(const Motor_T * p_motor);
extern void Motor_StateMachine_SetFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags);
extern void Motor_StateMachine_ClearFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags);

#endif


// static inline void Motor_StateMachine_Input(const Motor_T * p_motor, Motor_State_Input_T input, uintptr_t value) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, input, value); }