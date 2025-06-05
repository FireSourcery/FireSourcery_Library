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
    @file   Motor_Calibration.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Motor_StateMachine.h"

/* Exit the substate and return to the parent state on complete */
static inline bool Motor_Calibration_IsComplete(const Motor_State_T * p_motor) { return StateMachine_IsActiveSubState(&p_motor->StateMachine, &MOTOR_STATE_CALIBRATION); }
// static inline bool Motor_Calibration_IsComplete(const const Motor_T * p_motor) { return (p_motor->CalibrationState == MOTOR_CALIBRATION_STATE_DISABLE); }


/* Proc, to allow validate immediately */
/* StateMachine_ProcInput cannot transitions top level only */
static inline void Motor_Calibration_Enter(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&MOTOR_STATE_CALIBRATION); }
static inline void Motor_Calibration_Exit(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&MOTOR_STATE_STOP); }
// static inline void Motor_Calibration_Exit(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, MOTOR_DIRECTION_NULL); }

static inline void Motor_Calibration_EnterBranch(const Motor_T * p_motor, State_T * p_subState) { StateMachine_ProcBranchInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)p_subState); }

/*

*/
extern void Motor_Calibration_StartHome(const Motor_T * p_motor);