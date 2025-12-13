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
    @file   Motor_StateMachine.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor_Commutation.h"
#include "Motor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/StateMachine/_StateMachine.h" /* Include the private header to allocate within Motor_State_T */

#include "System/Critical/Critical.h"
#include "System/SysTime/SysTime.h"


/******************************************************************************/
/*
    Motor State Machine
*/
/******************************************************************************/
#define MSM_TRANSITION_TABLE_LENGTH (7U)

#ifndef MOTOR_STATE_MACHINE_INIT_WAIT
#define MOTOR_STATE_MACHINE_INIT_WAIT (1500U) /* For 1S polling to run twice */
#endif

/******************************************************************************/
/*
    Motor State Machine Inputs
*/
/******************************************************************************/
typedef enum Motor_State_Input
{
    MSM_INPUT_FAULT,            /* Toggle Fault */
    MSM_INPUT_PHASE_OUTPUT,     /* [Phase_Output_T] Active/Release/Hold */
    MSM_INPUT_FEEDBACK_MODE,    /* [FeedbackMode_T] flags */
    MSM_INPUT_DIRECTION,        /* [Motor_Direction_T] Ccw/Cw Start/Stop */
    MSM_INPUT_OPEN_LOOP,        /* OpenLoop Cmd */
    MSM_INPUT_CALIBRATION,      /* Calibration Cmd */
    // MSM_INPUT_USER_BUFFER,
    // MSM_INPUT_CAPTURE_ADC,
}
Motor_State_Input_T;

/******************************************************************************/
/*!
    @brief  Motor State IDs
*/
/******************************************************************************/
typedef enum Motor_StateId
{
    MSM_STATE_ID_INIT,
    MSM_STATE_ID_STOP,      /* 0 speed. */ //  Feedback Off + Ouput VFloat / V0 / 0 speed
    MSM_STATE_ID_PASSIVE,   /* without feedback */ // Feedback Off + Ouput VFloat / V0 - optionally
    // MSM_STATE_ID_FREEWHEEL,
    MSM_STATE_ID_RUN,       /* Feedback Loop */ // Feedback On + Ouput VPWM
    MSM_STATE_ID_OPEN_LOOP, /* Torque Loop On/Off + Float/V0/VPWM */
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

/*!
    @param p_MotorContext [const Motor_T *]
    @param MotorActive [Motor_State_T]
*/
#define MOTOR_STATE_MACHINE_INIT(p_MotorContext, MotorState) STATE_MACHINE_INIT((p_MotorContext), &MSM_MACHINE, &((MotorState).StateMachine))

// static inline bool _Motor_StateMachine_IsState(const StateMachine_T * p_stateMachine, Motor_StateId_T stateId) { return (StateMachine_IsActiveStateId(p_stateMachine->P_ACTIVE, stateId)); }


/* Wrap for interface */
/* Does not include substates */
static inline bool _Motor_StateMachine_IsState(const Motor_State_T * p_fields, Motor_StateId_T stateId) { return (StateMachine_IsActiveStateId(&p_fields->StateMachine, stateId)); }
static inline bool Motor_StateMachine_IsState(const Motor_T * p_motor, Motor_StateId_T stateId) { return (StateMachine_IsActiveStateId(p_motor->STATE_MACHINE.P_ACTIVE, stateId)); }

// static inline bool _Motor_StateMachine_IsFault(const Motor_State_T * p_fields) { return (_Motor_StateMachine_IsState(p_fields, MSM_STATE_ID_FAULT)); }
static inline bool Motor_StateMachine_IsFault(const Motor_T * p_motor) { return Motor_StateMachine_IsState(p_motor, MSM_STATE_ID_FAULT); }

// static inline bool _Motor_StateMachine_IsOpenLoop(const Motor_State_T * p_fields) { return _Motor_StateMachine_IsState(p_fields, MSM_STATE_ID_OPEN_LOOP); }

static inline bool Motor_StateMachine_IsConfig(const Motor_T * p_motor)
{
    return (StateMachine_GetActiveStateId(p_motor->STATE_MACHINE.P_ACTIVE) == MSM_STATE_ID_STOP) ||
           (StateMachine_GetActiveStateId(p_motor->STATE_MACHINE.P_ACTIVE) == MSM_STATE_ID_CALIBRATION) ||
           (StateMachine_GetActiveStateId(p_motor->STATE_MACHINE.P_ACTIVE) == MSM_STATE_ID_FAULT);
}




/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Motor_StateMachine_EnterFault(const Motor_T * p_motor);
extern bool Motor_StateMachine_ExitFault(const Motor_T * p_motor);
extern void Motor_StateMachine_SetFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags);
extern void Motor_StateMachine_ClearFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags);

// static inline void Motor_StateMachine_Input(const Motor_T * p_motor, Motor_State_Input_T input, uintptr_t value) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, input, value); }