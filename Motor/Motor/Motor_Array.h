
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
    @file   Motor_Array.h
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#ifndef MOTOR_ARRAY_H
#define MOTOR_ARRAY_H

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"
#include "Type/Array/void_array.h"


/* Shorthand Wrappers */

typedef const struct Motor_Array
{
    Motor_T * const P_ARRAY; const size_t LENGTH;
}
Motor_Array_T;

static inline void Motor_Array_ForEach(Motor_Array_T * p_this, Motor_Proc_T function) { void_array_foreach(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (proc_t)function); }
static inline bool Motor_Array_ForEvery(Motor_Array_T * p_this, Motor_TryProc_T function) { return void_array_for_every(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (try_proc_t)function); }
static inline bool Motor_Array_IsEvery(const Motor_Array_T * p_this, Motor_Test_T test) { return void_array_is_every(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (test_t)test); }
static inline bool Motor_Array_IsAny(const Motor_Array_T * p_this, Motor_Test_T test) { return void_array_is_any(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (test_t)test); }

static inline void Motor_Array_ForEachSet(Motor_Array_T * p_this, Motor_Set_T function, motor_value_t value) { void_array_foreach_set(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (set_t)function, value); }
static inline bool Motor_Array_IsEverySet(const Motor_Array_T * p_this, Motor_TrySet_T test, motor_value_t value) { return void_array_for_every_set(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (try_set_t)test, value); }
static inline bool Motor_Array_IsAnySet(const Motor_Array_T * p_this, Motor_TrySet_T test, motor_value_t value) { return void_array_for_any_set(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (try_set_t)test, value); }

static inline bool Motor_Array_IsEveryValue(const Motor_Array_T * p_this, Motor_TryValue_T test, motor_value_t value) { return void_array_for_every_set(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (try_set_t)test, value); }
static inline bool Motor_Array_IsAnyValue(const Motor_Array_T * p_this, Motor_TryValue_T test, motor_value_t value) { return void_array_for_any_set(p_this->P_ARRAY, sizeof(Motor_T), p_this->LENGTH, (try_set_t)test, value); }

/* temp */
static inline void Motor_Array_SetCmdValue(const Motor_Array_T * p_this, int16_t userCmd)
{
    for (uint8_t iMotor = 0U; iMotor < p_this->LENGTH; iMotor++) { Motor_User_SetActiveCmdValue_Scalar(&p_this->P_ARRAY[iMotor], userCmd); }
}

/*
    no void_array functions
*/
static inline void Motor_Array_ApplySpeedLimit(const Motor_Array_T * p_this, LimitArray_T * p_limit)
{
    for (uint8_t iMotor = 0U; iMotor < p_this->LENGTH; iMotor++) { Motor_SetSpeedLimitWith(&p_this->P_ARRAY[iMotor], p_limit); }
}

static inline void Motor_Array_ApplyILimit(const Motor_Array_T * p_this, LimitArray_T * p_limit)
{
    for (uint8_t iMotor = 0U; iMotor < p_this->LENGTH; iMotor++) { Motor_SetILimitWith(&p_this->P_ARRAY[iMotor], p_limit); }
}

#endif