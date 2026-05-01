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
    @file   Motor_Table.h
    @author FireSourcery
    @brief  Motor Array/Collection Interface
*/
/******************************************************************************/
#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Analog/Motor_Analog.h"
#include "Type/Array/void_array.h"


/* Shorthand Wrappers */
/* Define with motor runtime state allocated in continuous array */
typedef const struct Motor_Table
{
    Motor_State_T * const P_STATES; /* optionally */
    const Motor_T * const P_DEVS;
    const size_t LENGTH;
}
Motor_Table_T;

static inline Motor_State_T * Motor_Table_StateAt(Motor_Table_T * p_table, uint8_t motorIndex) { return &(p_table->P_STATES[motorIndex]); }
static inline Motor_T * Motor_Table_At(Motor_Table_T * p_table, uint8_t motorIndex) { return &(p_table->P_DEVS[motorIndex]); }


static inline void Motor_Table_ForEach(Motor_Table_T * p_table, Motor_Proc_T function) { void_array_foreach(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (proc_t)function); }

static inline void Motor_Table_ForEachSet(Motor_Table_T * p_table, Motor_Set_T function, motor_value_t value) { void_array_foreach_set(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (set_t)function, value); }
static inline bool Motor_Table_ForEvery(Motor_Table_T * p_table, Motor_State_TryProc_T function) { return void_array_for_every(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (try_proc_t)function); }

static inline bool Motor_Table_IsEverySet(Motor_Table_T * p_table, Motor_State_TrySet_T test, motor_value_t value) { return void_array_for_every_set(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (try_set_t)test, value); }
static inline bool Motor_Table_IsAnySet(Motor_Table_T * p_table, Motor_State_TrySet_T test, motor_value_t value) { return void_array_for_any_set(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (try_set_t)test, value); }

/* Const State  */
static inline bool Motor_Table_IsEvery(Motor_Table_T * p_table, Motor_State_Test_T test) { return void_array_is_every(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (test_t)test); }
static inline bool Motor_Table_IsAny(Motor_Table_T * p_table, Motor_State_Test_T test) { return void_array_is_any(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (test_t)test); }

/* Test value only, cast function pointer for now */
static inline bool Motor_Table_IsEveryValue(Motor_Table_T * p_table, Motor_State_TryValue_T test, motor_value_t value) { return void_array_is_every_value(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (test_value_t)test, value); }
static inline bool Motor_Table_IsAnyValue(Motor_Table_T * p_table, Motor_State_TryValue_T test, motor_value_t value) { return void_array_is_any_value(sizeof(Motor_State_T), p_table->P_STATES, p_table->LENGTH, (test_value_t)test, value); }


/******************************************************************************/
/*`
    Specialized
*/
/******************************************************************************/
// #define Motor_CastSetter(p_motor, setter) \
//     _Generic((setter), \
//         Motor_Set_T:            (Motor_Set_T)(setter), \
//         Motor_SetCmdValue_T:    (Motor_SetCmdValue_T)(setter ),  \
//         Motor_Proc_T:           (Motor_Proc_T)(setter )   \
//     )(p_motor, )

// #define Motor_SetWith(p_motor, setter, value, ...) ((Motor_CastSetter(p_motor, focSet))(p_motor, value __VA_OPT__(,) __VA_ARGS__))

/// todo create a set on ArraySpan
/*
    Feedback Mode Set first
*/
typedef void (*Motor_SetCmdValue_T)(Motor_State_T * p_motor, int16_t userCmd); /* alternatively as cmd struct */

/* selected mode using function */
static inline void Motor_Table_SetCmdWith(Motor_Table_T * p_table, Motor_SetCmdValue_T function, int16_t value) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { function(&p_table->P_STATES[iMotor], value); } }

// static inline void Motor_Table_ApplyInputs(Motor_Table_T * p_table, Motor_Input_T * p_input) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ProcSyncInput(&p_table->P_MONITORS[iMotor], p_input); } }

/*
    Two-scope arbitration: per motor, effective limit = min(system_upper, local_upper).
    Both scopes default to LIMIT_ARRAY_MAX when inactive, so the composition falls
    through to Motor_Set* which clamps to Config cap — equivalent to "reset".
*/
static inline void Motor_Table_ApplySpeedLimit(Motor_Table_T * p_table, LimitArray_T * p_system) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_SetSpeedLimitWith(&p_table->P_STATES[iMotor], &p_table->P_DEVS[iMotor].SPEED_LIMITS_LOCAL, p_system); } }
static inline void Motor_Table_ApplyILimit(Motor_Table_T * p_table, LimitArray_T * p_system) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_SetILimitMotoringWith(&p_table->P_STATES[iMotor], &p_table->P_DEVS[iMotor].I_LIMITS_LOCAL, p_system); } }
static inline void Motor_Table_ApplyIGenLimit(Motor_Table_T * p_table, LimitArray_T * p_system) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_SetILimitGeneratingWith(&p_table->P_STATES[iMotor], &p_table->P_DEVS[iMotor].I_GEN_LIMITS_LOCAL, p_system); } }


/******************************************************************************/
/*
    On Full Context
*/
/******************************************************************************/
// typedef void (*Motor_ContextAction_T)(const Motor_T * p_dev, Motor_State_T * p_state);
// static inline void _Motor_Table_Context_ForEach(Motor_State_T * p_motors, int count, Motor_Proc_T function) { void_array_foreach(sizeof(Motor_State_T), p_motors, count, (proc_t)function); }

static inline void Motor_Table_ApplyFeedbackMode(Motor_Table_T * p_table, Motor_FeedbackMode_T mode) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ApplyFeedbackMode(&p_table->P_DEVS[iMotor], mode); } }

static inline void Motor_Table_ApplyControl(Motor_Table_T * p_table, Phase_Output_T state) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ApplyControlState(&p_table->P_DEVS[iMotor], state); } }

static inline void Motor_Table_ApplyUserDirection(Motor_Table_T * p_table, int sign) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ApplyUserDirection(&p_table->P_DEVS[iMotor], sign); } }

typedef void(*Motor_ContextProc_T)(Motor_T * p_motor);
/* Motor_Table_ForEachProc */
static inline void Motor_Table_ForEachApply(Motor_Table_T * p_table, Motor_ContextProc_T function) { void_array_foreach(sizeof(Motor_T), (void *)p_table->P_DEVS, p_table->LENGTH, (proc_t)function); }
// static inline void Motor_Table_EnableAll(Motor_Table_T * p_table) { Motor_Table_ForEachApply(p_table, (Motor_ContextProc_T)Motor_Enable); }

static inline void Motor_Table_DisableAll(Motor_Table_T * p_table) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_Disable(&p_table->P_DEVS[iMotor]); } }
static inline void Motor_Table_EnableAll(Motor_Table_T * p_table) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_Enable(&p_table->P_DEVS[iMotor]); } }
static inline void Motor_Table_ForceDisableControl(Motor_Table_T * p_table) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ForceDisableControl(&p_table->P_DEVS[iMotor]); } }
static inline void Motor_Table_EnterCalibration(Motor_Table_T * p_table) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_Calibration_Enter(&p_table->P_DEVS[iMotor]); } }
static inline void Motor_Table_EnterCalibrateAdc(Motor_Table_T * p_table) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_Analog_Calibrate(&p_table->P_DEVS[iMotor]); } }

static inline bool Motor_Table_IsCalibrationComplete(Motor_Table_T * p_table) { return void_array_is_every(sizeof(Motor_T), p_table->P_DEVS, p_table->LENGTH, (test_t)Motor_Calibration_IsComplete); }

/* IsEveryMachineState */
static inline bool Motor_Table_IsEveryState(Motor_Table_T * p_table, State_T * p_state)
{
    bool isEvery = true;
    for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { if (Motor_IsState(&p_table->P_DEVS[iMotor], p_state) == false) { isEvery = false; break; } }
    return isEvery;
}

static inline bool Motor_Table_IsAnyState(Motor_Table_T * p_table, State_T * p_state)
{
    bool isAny = false;
    for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { if (Motor_IsState(&p_table->P_DEVS[iMotor], p_state) == true) { isAny = true; break; } }
    return isAny;
}

static inline bool Motor_Table_IsEveryUserDirection(Motor_Table_T * p_table, int sign)
{
    bool isEvery = true;
    for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { if (Motor_GetUserDirection(p_table->P_DEVS[iMotor].P_MOTOR) != sign) { isEvery = false; break; } }
    return isEvery;
}

/*  */
static int _Motor_Table_GetDirectionAll(Motor_Table_T * p_table)
{
    volatile int direction;
    if (Motor_Table_IsEvery(p_table, Motor_IsDirectionForward) == true) { direction = 1; }
    else if (Motor_Table_IsEvery(p_table, Motor_IsDirectionReverse) == true) { direction = -1; }
    else { direction = 0; } /* overload stop and Error */
    return direction;
}

// static inline void Motor_Table_InputStateMachine(Motor_Table_T * p_table, Motor_State_Input_T input, state_value_t value)  { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_StateMachine_Input(&p_table->P_MONITORS[iMotor], input, value); } }
