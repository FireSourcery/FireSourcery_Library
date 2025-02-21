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

#include "Motor_Calibration.h"
#include "Motor_FOC.h"
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#include "Motor_SixStep.h"
#endif
#include "Motor.h"

#include "Utility/StateMachine/StateMachine.h"

#include "System/Critical/Critical.h"
#include "System/SysTime/SysTime.h"


#define MSM_TRANSITION_TABLE_LENGTH (7U)

/*
    Motor State Machine Inputs
*/
typedef enum Motor_StateMachine_Input
{
    MSM_INPUT_FAULT,            /* Toggle Fault */
    MSM_INPUT_CONTROL_MODE,     /* Active/Release/Hold */
    MSM_INPUT_FEEDBACK_MODE,    /* [FeedbackMode] flags */
    MSM_INPUT_DIRECTION,
    MSM_INPUT_CALIBRATION,
    MSM_INPUT_OPEN_LOOP,
}
Motor_StateMachine_Input_T;

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

/*
    Open Loop SubState
*/
typedef enum Motor_OpenLoopState
{
    MOTOR_OPEN_LOOP_STATE_ENTER,
    MOTOR_OPEN_LOOP_STATE_PASSIVE,
    // MOTOR_OPEN_LOOP_STATE_CMD,
    MOTOR_OPEN_LOOP_STATE_ALIGN,
    MOTOR_OPEN_LOOP_STATE_RUN,
    // MOTOR_OPEN_LOOP_STATE_VALIDATE_ALIGN,
    MOTOR_OPEN_LOOP_STATE_START_UP_ALIGN,
    MOTOR_OPEN_LOOP_STATE_START_UP_RUN,
}
Motor_OpenLoopState_T;

// typedef enum Motor_OpenLoopCmd
// {
//     MOTOR_OPEN_LOOP_CMD_PHASE_CONTROL,
//     MOTOR_OPEN_LOOP_CMD_PHASE_ALIGN,
//     MOTOR_OPEN_LOOP_CMD_ANGLE,
//     MOTOR_OPEN_LOOP_CMD_ALIGN,
//     // MOTOR_OPEN_LOOP_CMD_STARTUP,
// }
// Motor_OpenLoopCmd_T;

/*
    Calibration SubState
*/
typedef enum Motor_CalibrationState
{
    MOTOR_CALIBRATION_STATE_DISABLE,
    MOTOR_CALIBRATION_STATE_ADC,
    MOTOR_CALIBRATION_STATE_HALL,
    MOTOR_CALIBRATION_STATE_ENCODER,
    MOTOR_CALIBRATION_STATE_SIN_COS,
    MOTOR_CALIBRATION_STATE_POSITION_SENSOR,
    MOTOR_CALIBRATION_STATE_IDLE,
}
Motor_CalibrationState_T;

/* extern for extension */
extern const StateMachine_State_T MOTOR_STATE_INIT;
extern const StateMachine_State_T MOTOR_STATE_STOP;
extern const StateMachine_State_T MOTOR_STATE_RUN;
extern const StateMachine_State_T MOTOR_STATE_FREEWHEEL;
extern const StateMachine_State_T MOTOR_STATE_CALIBRATION;
extern const StateMachine_State_T MOTOR_STATE_FAULT;
// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern const StateMachine_State_T MOTOR_STATE_OPEN_LOOP;
// #endif

// extern const StateMachine_State_T OPEN_LOOP_STATE_ALIGN;
// extern const StateMachine_State_T OPEN_LOOP_STATE_RUN;
// extern const StateMachine_State_T OPEN_LOOP_STATE_START_UP_ALIGN;


extern const StateMachine_Machine_T MSM_MACHINE;

#define MOTOR_STATE_MACHINE_INIT(p_Motor) STATE_MACHINE_INIT(&MSM_MACHINE, p_Motor, false)

/* uint8_t Wrap for array */
static inline bool Motor_StateMachine_IsState(const Motor_T * p_motor, uint8_t stateId) { return (StateMachine_IsActiveStateId(&p_motor->StateMachine, (Motor_StateMachine_StateId_T)stateId)); }

static inline bool Motor_Calibration_IsComplete(const Motor_T * p_motor) { return (p_motor->CalibrationState == MOTOR_CALIBRATION_STATE_DISABLE); }
// void Motor_OpenLoop_Enter(Motor_T * p_motor, state_machine_value_t state)
// {
//     StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_OPEN_LOOP, MOTOR_OPEN_LOOP_STATE_ENTER);
// }

extern void Motor_OpenLoop_SetPhaseState(Motor_T * p_motor, Phase_State_T state);
extern void Motor_OpenLoop_SetPhaseVAlign(Motor_T * p_motor, Phase_Align_T align);
extern void Motor_OpenLoop_SetAngleAlign(Motor_T * p_motor, angle16_t angle);
extern void Motor_OpenLoop_SetJog(Motor_T * p_motor, int8_t direction);
extern void Motor_OpenLoop_StartRunChain(Motor_T * p_motor);
extern void Motor_Calibration_StartHome(Motor_T * p_motor);

// extern void Motor_Calibration_SetIdle(Motor_T * p_motor);

extern bool Motor_StateMachine_IsFault(const Motor_T * p_motor);
extern bool Motor_StateMachine_ExitFault(Motor_T * p_motor);
extern void Motor_StateMachine_EnterFault(Motor_T * p_motor);

#endif

