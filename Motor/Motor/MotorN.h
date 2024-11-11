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
    @file   MotorN.h
    @author FireSourcery
    @brief  Array functions
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_N_H
#define MOTOR_N_H

#include "Motor_User.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    N Motor Array functions
*/
/******************************************************************************/
typedef void (*Motor_ProcVoid_T)(MotorPtr_T p_motor);
typedef bool (*Motor_PollStatus_T)(MotorPtr_T p_motor);

typedef void (*Motor_SetInt16_T)(MotorPtr_T p_motor, int16_t cmd);
typedef void (*Motor_SetUint16_T)(MotorPtr_T p_motor, uint16_t cmd);
typedef bool (*Motor_SetInt16WithStatus_T)(MotorPtr_T p_motor, int16_t cmd);
typedef void (*Motor_SetId_T)(MotorPtr_T p_motor, uint32_t enumValue); // user provide cast function
typedef bool (*Motor_Test_T)(const MotorPtr_T p_motor);

typedef bool (*Motor_SetEntryUint16_T)(MotorPtr_T p_motor, uint8_t id, uint16_t value);
typedef bool (*Motor_SetEntryInt16_T)(MotorPtr_T p_motor, uint8_t id, int16_t value);
typedef bool (*Motor_ClearEntry_T)(MotorPtr_T p_motor, uint8_t id);


typedef void (*Motor_User_SetFeedbackMode_T)(MotorPtr_T p_motor, Motor_FeedbackMode_T feedbackMode);

// #define MOTOR_PROC(function, p_motor) _Generic((function), \
//     Motor_ProcVoid_T:           function,   \
//     Motor_PollStatus_T:         function)(p_motor, value)

// Use function type to select itself. Enforces type safety, checks value type against function type.
#define MOTOR_SET(function, p_motor, value) _Generic((function), \
    Motor_SetId_T:                  function,   \
    Motor_SetUint16_T:              function,   \
    Motor_SetInt16WithStatus_T:     function,   \
    default:                        function)(p_motor, value)

/* 2 parameters */
#define MOTOR_SET_ENTRY(function, p_motor, key, value) _Generic((function), \
    Motor_SetEntryInt16_T:      function, \
    Motor_SetEntryUint16_T:     function)(p_motor, key, value)


// #define EACH_MOTOR(motorCount) (uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
// #define MOTOR_N_FOR_EACH(p_motorArray, motorCount, function)                    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { MOTOR_PROC(&p_motorArray[iMotor]); }
#define MOTOR_N_SET_EACH(p_motorArray, motorCount, function, value)             for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { MOTOR_SET(function, &p_motorArray[iMotor], value); }
#define MOTOR_N_SET_EACH_ENTRY(p_motorArray, motorCount, function, key, value)  for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { MOTOR_SET_ENTRY( function, &p_motorArray[iMotor], key, value); }

#define MOTOR_N_POLL_EACH_ENTRY(p_motorArray, motorCount, function, key, value) \
{   \
    bool isTrue = false; \
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(MOTOR_SET_ENTRY(&p_motorArray[iMotor], key, value) == true) { isTrue = true; }; } \
    return isTrue; \
}

#define MOTOR_N_POLL_EACH_WITH(p_motorArray, motorCount, function, value) \
{   \
    bool isTrue = false; \
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(MOTOR_SET(function, &p_motorArray[iMotor], value) == true) { isTrue = true; }; } \
    return isTrue; \
}



// static inline void MotorN_User_test(MotorPtr_T p_motorArray, uint8_t motorCount )
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { MOTOR_SET(Motor_SetCmd, &p_motorArray[iMotor], 0); }

//     MOTOR_N_SET_EACH(p_motorArray, motorCount, Motor_SetCmd, 0);
// }


/*
    Motor_User_ForceDisableControl
    Motor_User_TryRelease
    Motor_User_TryHold
*/
static inline void MotorN_Proc(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_ProcVoid_T function)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor]); }
}

/*
    Motor_User_TryDirectionForward
    Motor_User_TryDirectionReverse
    MotorN_User_ClearFault
*/
static inline bool MotorN_Poll(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_PollStatus_T function)
{
    bool isTrue = true;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motorArray[iMotor]) == false) { isTrue = false; } }
    return isTrue;
}

static inline bool MotorN_PollSetInt16(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_SetInt16WithStatus_T function, int32_t value) MOTOR_N_POLL_EACH_WITH(p_motorArray, motorCount, function, value);



/*
    AND logic
    e.g. MotorN_User_CheckStop
*/
static inline bool MotorN_IsEvery(const MotorPtr_T p_motorArray, uint8_t motorCount, Motor_Test_T function)
{
    bool isTrue = true;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motorArray[iMotor]) == false) { isTrue = false; break; } }
    return isTrue;
}

/*
    OR logic
    e.g. MotorN_User_CheckFault
*/
static inline bool MotorN_IsAny(const MotorPtr_T p_motorArray, uint8_t motorCount, Motor_Test_T function)
{
    bool isTrue = false;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motorArray[iMotor]) == true) { isTrue = true; break; } }
    return isTrue;
}


// static inline void MotorN_User_SetCmd(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_SetInt16_T function, int16_t cmdValue)
// {
//     // for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { MotorN_User_Set(function, &p_motorArray[iMotor], cmdValue); }
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor], cmdValue); }
// }

static inline void MotorN_User_SetScalar16(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_SetUint16_T function, uint16_t scalar16)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor], scalar16); }
}

// static inline void MotorN_User_SetId(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_SetId_T function, uint32_t cmdValue)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor], cmdValue); }
// }

// static inline void MotorN_User_SetFeedbackMode(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_FeedbackMode_T feedbackMode)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { Motor_ActivateControl(&p_motorArray[iMotor], feedbackMode); }
// }
static inline void MotorN_User_ActivateControl(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_FeedbackMode_T feedbackMode)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { Motor_ActivateControl(&p_motorArray[iMotor], feedbackMode); }
}


/*!
    Motor_SetILimitEntry
    Motor_User_ClearLimit
    OR logic
    @return true if at least one is set
*/
static inline bool MotorN_User_SetLimit(MotorPtr_T p_motor, uint8_t motorCount, Motor_SetEntryUint16_T function, uint16_t limit_scalar16, uint8_t id)
{
    bool isSet = false;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motor[iMotor], id, limit_scalar16) == true) { isSet = true; }; }
    return isSet;
}

/*!
    OR logic
    @return true if at least one is cleared
*/
static inline bool MotorN_User_ClearLimit(MotorPtr_T p_motor, uint8_t motorCount, Motor_ClearEntry_T function, uint8_t id)
{
    bool isSet = false;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motor[iMotor], id) == true) { isSet = true; }; }
    return isSet;
}



#endif
