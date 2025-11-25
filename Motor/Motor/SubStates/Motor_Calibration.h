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

extern const State_T CALIBRATION_STATE_TUNNING;

/* Exit the substate and return to the parent state on complete */
// static inline bool _Motor_Calibration_IsComplete(const Motor_State_T * p_motor) { return StateMachine_IsActiveSubState(&p_motor->StateMachine, &MOTOR_STATE_CALIBRATION); }
static inline bool Motor_Calibration_IsComplete(const Motor_T * p_motor) { return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &MOTOR_STATE_CALIBRATION); }

/* IsComplete SubState = TopState =  MSM_STATE_ID_CALIBRATION */

/* return MSM_STATE_ID_CALIBRATION when parent state is MSM_STATE_ID_CALIBRATION. return 0xff otherwise including state == MSM_STATE_ID_CALIBRATION */
// static inline state_t Motor_Calibration_GetSubStateId(const Motor_State_T * p_motor) { return StateMachine_GetActiveSubStateId(&p_motor->StateMachine, &MOTOR_STATE_CALIBRATION); }

/* Proc, to allow validate immediately */
/* StateMachine_ApplyInput cannot transitions top level only */
static inline void Motor_Calibration_Enter(const Motor_T * p_motor) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&MOTOR_STATE_CALIBRATION); }
// static inline void Motor_Calibration_Exit(const Motor_T * p_motor) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&MOTOR_STATE_STOP); }
// static inline void Motor_Calibration_Exit(const Motor_T * p_motor) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, MOTOR_DIRECTION_NULL); }

/* if every cmd is also a substate */
static inline void Motor_Calibration_EnterBranch(const Motor_T * p_motor, State_T * p_subState) { StateMachine_Branch_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)p_subState); }

/*

*/
extern void Motor_Calibration_StartHome(const Motor_T * p_motor);


/*
    Calibration SubState
*/
// typedef enum Motor_CalibrationState
// {
//     MOTOR_CALIBRATION_STATE_DISABLE,
//     MOTOR_CALIBRATION_STATE_ADC,
//     MOTOR_CALIBRATION_STATE_HALL,
//     MOTOR_CALIBRATION_STATE_ENCODER,
//     MOTOR_CALIBRATION_STATE_SIN_COS,
//     MOTOR_CALIBRATION_STATE_POSITION_SENSOR,
//     MOTOR_CALIBRATION_STATE_IDLE,
// }
// Motor_CalibrationState_T;

extern void Motor_Calibration_EnterTuning(const Motor_T * p_motor);
