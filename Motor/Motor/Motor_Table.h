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

// handle per instance extensions not required by core logic
// typedef const struct Motor_Entity
// {
//     struct
//     {
//          Motor_T DEV;
//          HeatMonitor_T HEAT_MONITOR;
//          Analog_Conversion_T HEAT_MONITOR_CONVERSION;
//          uint8_t * const P_ADAPTER_BUFFER;
//     } ;
// }
// Motor_Entity_T;

/* Shorthand Wrappers */
/* Define with motor runtime state allocated in continuous array */
// split as 2 array spans
typedef const struct Motor_Table
{
    // Motor_Context_T * const P_STATES; /* optionally */
    Motor_T * const P_DEVS;
    const size_t LENGTH;
}
Motor_Table_T;


/******************************************************************************/
/*
    Specialized
*/
/******************************************************************************/

/*
    Feedback Mode Set first
*/
typedef void (*Motor_SetCmdValue_T)(Motor_Context_T * p_motor, int16_t userCmd); /* alternatively as cmd struct */


/*
*/
static inline void Motor_Table_ApplyFeedbackMode(Motor_Table_T * p_table, Motor_FeedbackMode_T mode) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ApplyFeedbackMode(&p_table->P_DEVS[iMotor], mode); } }
static inline void Motor_Table_ApplyControl(Motor_Table_T * p_table, Phase_VOutMode_T state) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ApplyControlState(&p_table->P_DEVS[iMotor], state); } }
static inline void Motor_Table_ApplyUserDirection(Motor_Table_T * p_table, Motor_Direction_T sign) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ApplyUserDirection(&p_table->P_DEVS[iMotor], sign); } }

/* on states */
/* selected mode using function */
// static inline void Motor_Table_SetCmdWith(Motor_Table_T * p_table, Motor_SetCmdValue_T function, int16_t value) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { function(&p_table->P_STATES[iMotor], value); } }
static inline void Motor_Table_SetCmdWith(Motor_Table_T * p_table, Motor_SetCmdValue_T function, int16_t value) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { function(p_table->P_DEVS[iMotor].P_MOTOR, value); } }

// static inline void Motor_Table_ApplyInputs(Motor_Table_T * p_table, Motor_Input_T * p_input) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ProcSyncInput(&p_table->P_MONITORS[iMotor], p_input); } }

static inline bool Motor_Table_IsEveryUserDirection(Motor_Table_T * p_table, Motor_Direction_T sign)
{
    bool isEvery = true;
    for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { if (Motor_GetUserDirection(p_table->P_DEVS[iMotor].P_MOTOR) != sign) { isEvery = false; break; } }
    return isEvery;
}

// update to dev
// static inline bool Motor_Table_IsEverySpeedZero(Motor_Table_T * p_table) { return Motor_Table_IsEvery(p_table, Motor_IsSpeedZero); }
static inline bool Motor_Table_IsEverySpeedZero(Motor_Table_T * p_table)
{
    bool isEvery = true;
    for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { if (Motor_IsSpeedZero(p_table->P_DEVS[iMotor].P_MOTOR) == false) { isEvery = false; break; } }
    return isEvery;
}

/*  */
static Motor_Direction_T _Motor_Table_GetDirectionAll(Motor_Table_T * p_table)
{
    volatile Motor_Direction_T direction;
    if (Motor_Table_IsEveryUserDirection(p_table, MOTOR_DIRECTION_FORWARD) == true) { direction = MOTOR_DIRECTION_FORWARD; }
    else if (Motor_Table_IsEveryUserDirection(p_table, MOTOR_DIRECTION_REVERSE) == true) { direction = MOTOR_DIRECTION_REVERSE; }
    else { direction = MOTOR_DIRECTION_NULL; } /* overload stop and Error */
    return direction;
}

/******************************************************************************/
/*
    On Full Context
*/
/******************************************************************************/
typedef void(*Motor_ArrayProc_T)(Motor_T * p_motor);

typedef int(*Motor_ArrayGet_T)(Motor_T * p_motor);
typedef void(*Motor_ArraySet_T)(Motor_T * p_motor, int value);

typedef bool(*Motor_Test_T)(Motor_T * p_motor);
typedef bool(*Motor_TryProc_T)(Motor_T * p_motor);
typedef bool(*Motor_TrySet_T)(Motor_T * p_motor, int value);

// #define Motor_CastSetter(p_motor, setter)
//     _Generic((setter),
//         Motor_Set_T:            (Motor_Set_T)(setter),
//         Motor_SetCmdValue_T:    (Motor_SetCmdValue_T)(setter ),
//         Motor_Proc_T:           (Motor_Proc_T)(setter )
//     )(p_motor, )

// #define Motor_SetWith(p_motor, setter, value, ...) ((Motor_CastSetter(p_motor, focSet))(p_motor, value __VA_OPT__(,) __VA_ARGS__))

static inline void Motor_Table_ForEach(Motor_Table_T * p_table, Motor_ArrayProc_T function) { void_array_foreach(sizeof(Motor_T), (void *)p_table->P_DEVS, p_table->LENGTH, (proc_t)function); }
static inline void Motor_Table_ForEachSet(Motor_Table_T * p_table, Motor_ArraySet_T function, int value) { void_array_foreach_set(sizeof(Motor_T), (void *)p_table->P_DEVS, p_table->LENGTH, (set_t)function, value); }
static inline bool Motor_Table_ForEvery(Motor_Table_T * p_table, Motor_TryProc_T function) { return void_array_for_every(sizeof(Motor_T), (void *)p_table->P_DEVS, p_table->LENGTH, (try_proc_t)function); }

static inline bool Motor_Table_ForEverySet(Motor_Table_T * p_table, Motor_TrySet_T test, int value) { return void_array_for_every_set(sizeof(Motor_T), (void *)p_table->P_DEVS, p_table->LENGTH, (try_set_t)test, value); }
static inline bool Motor_Table_ForAnySet(Motor_Table_T * p_table, Motor_TrySet_T test, int value) { return void_array_for_any_set(sizeof(Motor_T), (void *)p_table->P_DEVS, p_table->LENGTH, (try_set_t)test, value); }

static inline bool Motor_Table_IsEvery(Motor_Table_T * p_table, Motor_Test_T test) { return void_array_is_every(sizeof(Motor_T), p_table->P_DEVS, p_table->LENGTH, (test_t)test); }
static inline bool Motor_Table_IsAny(Motor_Table_T * p_table, Motor_Test_T test) { return void_array_is_any(sizeof(Motor_T), p_table->P_DEVS, p_table->LENGTH, (test_t)test); }

static inline void Motor_Table_DisableAll(Motor_Table_T * p_table) { Motor_Table_ForEach(p_table, Motor_Disable); }
static inline void Motor_Table_EnableAll(Motor_Table_T * p_table) { Motor_Table_ForEach(p_table, Motor_Enable); }
static inline void Motor_Table_ForceDisableControl(Motor_Table_T * p_table) { Motor_Table_ForEach(p_table, Motor_ForceDisableControl); }


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

/*
    Limit broadcast. Derate: shared system arbitration result. Value: common physical cap written to each motor's channel.
*/
static inline void Motor_Table_SetSpeedLimitDerate(Motor_Table_T * p_table, uint16_t scalar_ufract16) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_SetSpeedLimitDerate(p_table->P_DEVS[iMotor].P_MOTOR, scalar_ufract16); } }
static inline void Motor_Table_SetILimitDerate(Motor_Table_T * p_table, uint16_t scalar_ufract16) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_SetILimitDerate(p_table->P_DEVS[iMotor].P_MOTOR, scalar_ufract16); } }
static inline void Motor_Table_SetSpeedLimit(Motor_Table_T * p_table, uint16_t speed_ufract16) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_SetSpeedLimit(p_table->P_DEVS[iMotor].P_MOTOR, speed_ufract16); } }
static inline void Motor_Table_SetILimit(Motor_Table_T * p_table, uint16_t i_ufract16) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_SetILimit(p_table->P_DEVS[iMotor].P_MOTOR, i_ufract16); } }
static inline void Motor_Table_ResetSpeedLimit(Motor_Table_T * p_table) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ResetSpeedLimit(p_table->P_DEVS[iMotor].P_MOTOR); } }
static inline void Motor_Table_ResetILimit(Motor_Table_T * p_table) { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_ResetILimit(p_table->P_DEVS[iMotor].P_MOTOR); } }

// static inline void Motor_Table_InputStateMachine(Motor_Table_T * p_table, Motor_State_Input_T input, state_value_t value)  { for (uint8_t iMotor = 0U; iMotor < p_table->LENGTH; iMotor++) { Motor_StateMachine_Input(&p_table->P_MONITORS[iMotor], input, value); } }
