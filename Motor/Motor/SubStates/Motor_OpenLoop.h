
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
    @file   Motor_OpenLoop.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Motor_StateMachine.h"

/*

*/
static inline void Motor_OpenLoop_Enter(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_OPEN_LOOP, (uintptr_t)&MOTOR_STATE_OPEN_LOOP); }

// static inline void Motor_OpenLoop_Exit(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_OPEN_LOOP, 0); }

/* alternatively use State_CmdInput_T with addition   entry */
// static inline void Motor_OpenLoop_EnterBranch(const Motor_T * p_motor, State_T * p_subState)
// {
//     StateMachine_ProcBranchInput(&p_motor->STATE_MACHINE, MSM_INPUT_OPEN_LOOP, (uintptr_t)p_subState);
// }

/*
    Extern
*/
/* Activate */
extern void Motor_OpenLoop_SetPhaseOutput(const Motor_T * p_motor, Phase_Output_T state);
extern void Motor_OpenLoop_SetPhaseAlign(const Motor_T * p_motor, Phase_Id_T align);
extern void Motor_OpenLoop_SetAngleAlign(const Motor_T * p_motor, angle16_t angle);
extern void Motor_OpenLoop_SetJog(const Motor_T * p_motor, int8_t direction);
extern void Motor_OpenLoop_StartRunChain(const Motor_T * p_motor);


/*
    Open Loop SubState
*/
// typedef enum Motor_OpenLoopState
// {
//     MOTOR_OPEN_LOOP_STATE_ENTER,
//     MOTOR_OPEN_LOOP_STATE_PASSIVE,
//     // MOTOR_OPEN_LOOP_STATE_CMD,
//     MOTOR_OPEN_LOOP_STATE_ALIGN,
//     MOTOR_OPEN_LOOP_STATE_RUN,
//     // MOTOR_OPEN_LOOP_STATE_VALIDATE_ALIGN,
//     MOTOR_OPEN_LOOP_STATE_START_UP_ALIGN,
//     MOTOR_OPEN_LOOP_STATE_START_UP_RUN,
// }
// Motor_OpenLoopState_T;


/*
    Id directly correponding to VarId
*/
typedef enum Motor_OpenLoop_VarCmd
{
    MOTOR_OPEN_LOOP_CMD_PHASE_CONTROL,
    MOTOR_OPEN_LOOP_CMD_PHASE_ALIGN,
    MOTOR_OPEN_LOOP_CMD_ANGLE,
    MOTOR_OPEN_LOOP_CMD_ALIGN,
    // MOTOR_OPEN_LOOP_CMD_STARTUP,

    // MOTOR_VAR_OPEN_LOOP_CONTROL,        /* Enter State. optional pass sub statecmd */
    // MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT,
    // MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN,
    // MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN,
    // MOTOR_VAR_OPEN_LOOP_JOG,
    // MOTOR_VAR_OPEN_LOOP_RUN,
}
Motor_OpenLoop_VarCmd_T;

