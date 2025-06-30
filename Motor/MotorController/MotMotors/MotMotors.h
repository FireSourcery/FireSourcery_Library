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
    @file   MotMotors.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"
#include "Type/Array/void_array.h"


/* Shorthand Wrappers */
/* Define with motor runtime state allocated in continous array */
typedef const struct MotMotors
{
    Motor_State_T * const P_STATES;
    const Motor_T * const P_CONTEXTS;
    const size_t LENGTH;
}
MotMotors_T;

static inline Motor_State_T * MotMotors_StateAt(const MotMotors_T * p_ctx, uint8_t motorIndex) { return &(p_ctx->P_STATES[motorIndex]); }
static inline Motor_T * MotMotors_ContextAt(const MotMotors_T * p_ctx, uint8_t motorIndex) { return &(p_ctx->P_CONTEXTS[motorIndex]); }

// typedef void (*Motor_ContextAction_T)(const Motor_T * p_context, Motor_State_T * p_state);

// static inline void _MotMotors_Context_ForEach(Motor_State_T * p_motors, int count, Motor_Proc_T function) { void_array_foreach(p_motors, sizeof(Motor_State_T), count, (proc_t)function); }

static inline void MotMotors_ForEach(const MotMotors_T * p_ctx, Motor_Proc_T function) { void_array_foreach(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (proc_t)function); }
static inline bool MotMotors_ForEvery(const MotMotors_T * p_ctx, Motor_State_TryProc_T function) { return void_array_for_every(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (try_proc_t)function); }

static inline void MotMotors_ForEachSet(const MotMotors_T * p_ctx, Motor_Set_T function, motor_value_t value) { void_array_foreach_set(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (set_t)function, value); }

static inline bool MotMotors_IsEverySet(const MotMotors_T * p_ctx, Motor_State_TrySet_T test, motor_value_t value) { return void_array_for_every_set(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (try_set_t)test, value); }
static inline bool MotMotors_IsAnySet(const MotMotors_T * p_ctx, Motor_State_TrySet_T test, motor_value_t value) { return void_array_for_any_set(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (try_set_t)test, value); }

/* Const State  */
static inline bool MotMotors_IsEvery(const MotMotors_T * p_ctx, Motor_State_Test_T test) { return void_array_is_every(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (test_t)test); }
static inline bool MotMotors_IsAny(const MotMotors_T * p_ctx, Motor_State_Test_T test) { return void_array_is_any(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (test_t)test); }

/* Test value only, cast function pointer for now */
static inline bool MotMotors_IsEveryValue(const MotMotors_T * p_ctx, Motor_State_TryValue_T test, motor_value_t value) { return void_array_for_every_set(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (try_set_t)test, value); }
static inline bool MotMotors_IsAnyValue(const MotMotors_T * p_ctx, Motor_State_TryValue_T test, motor_value_t value) { return void_array_for_any_set(p_ctx->P_STATES, sizeof(Motor_State_T), p_ctx->LENGTH, (try_set_t)test, value); }


/******************************************************************************/
/*
    Specialized
*/
/******************************************************************************/
/*
    no void_array functions
*/
static inline void MotMotors_ApplySpeedLimit(const MotMotors_T * p_ctx, LimitArray_T * p_limit)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_SetSpeedLimitWith(&p_ctx->P_STATES[iMotor], p_limit); } }

static inline void MotMotors_ApplyILimit(const MotMotors_T * p_ctx, LimitArray_T * p_limit)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_SetILimitWith(&p_ctx->P_STATES[iMotor], p_limit); } }

/*
    Feedback Mode Set first
*/
typedef void (*Motor_User_SetCmdValue_T)(Motor_State_T * p_motor, int16_t userCmd); /* alternatively as cmd struct */

/* selected mode using function */
static inline void MotMotors_SetCmdWith(const MotMotors_T * p_ctx, Motor_User_SetCmdValue_T function, int16_t value)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { function(&p_ctx->P_STATES[iMotor], value); } }

/*  */
// static inline void MotMotors_SetCmdValue(const MotMotors_T * p_ctx, int16_t userCmd)
//     { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_User_SetActiveCmdValue_Scalar(&p_ctx->P_STATES[iMotor], userCmd); } }



/******************************************************************************/
/*
    On Full Context
*/
/******************************************************************************/
static inline void MotMotors_SetFeedbackMode(const MotMotors_T * p_ctx, Motor_FeedbackMode_T mode)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_User_SetFeedbackMode(&p_ctx->P_CONTEXTS[iMotor], mode); } }

static inline void MotMotors_ActivateControlState(const MotMotors_T * p_ctx, Phase_Output_T state)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_User_ActivatePhaseOutput(&p_ctx->P_CONTEXTS[iMotor], state); } }

static inline void MotMotors_ApplyUserDirection(const MotMotors_T * p_ctx, int sign)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_User_ApplyDirectionSign(&p_ctx->P_CONTEXTS[iMotor], sign); } }

/* Exits Calibration and OpenLoop States */
static inline void MotMotors_StopAll(const MotMotors_T * p_ctx)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_User_Stop(&p_ctx->P_CONTEXTS[iMotor]); } }

static inline void MotMotors_ForceDisableControl(const MotMotors_T * p_ctx)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_User_ForceDisableControl(&p_ctx->P_CONTEXTS[iMotor]); } }

// static inline bool MotMotors_IsEveryUserDirection(const MotMotors_T * p_ctx, int sign)
// {
//     bool isEvery = true;
//     for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++)
//     {
//         if (Motor_GetUserDirection(&p_ctx->P_CONTEXTS[iMotor]) != sign) { isEvery = false; break; }
//     }
//     return isEvery;
// }

// static inline void MotMotors_InputStateMachine(const MotMotors_T * p_ctx, Motor_State_Input_T input, state_input_value_t value)
//     { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_StateMachine_Input(&p_ctx->P_CONTEXTS[iMotor], input, value); } }

static inline void MotMotors_EnterCalibration(const MotMotors_T * p_ctx)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_Calibration_Enter(&p_ctx->P_CONTEXTS[iMotor]); } }

static inline void MotMotors_EnterCalibrateAdc(const MotMotors_T * p_ctx)
    { for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { Motor_Analog_Calibrate(&p_ctx->P_CONTEXTS[iMotor]); } }

/* IsEveryMachineState */
static inline bool MotMotors_IsEveryState(const MotMotors_T * p_ctx, Motor_StateId_T stateId)
{
    bool isEvery = true;
    for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { if (Motor_StateMachine_IsState(&p_ctx->P_CONTEXTS[iMotor], stateId) == false) { isEvery = false; break; } }
    return isEvery;
}

static inline bool MotMotors_IsAnyState(const MotMotors_T * p_ctx, Motor_StateId_T stateId)
{
    bool isAny = false;
    for (uint8_t iMotor = 0U; iMotor < p_ctx->LENGTH; iMotor++) { if (Motor_StateMachine_IsState(&p_ctx->P_CONTEXTS[iMotor], stateId) == true) { isAny = true; break; } }
    return isAny;
}


