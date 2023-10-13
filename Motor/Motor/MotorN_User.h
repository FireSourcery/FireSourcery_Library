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
    @file   MotorN_User.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_N_USER_H
#define MOTOR_N_USER_H

#include "Motor_User.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    N Motor Array functions
*/
/******************************************************************************/
typedef void (*Motor_User_ProcVoid_T)(MotorPtr_T p_motor);
typedef void (*Motor_User_SetCmd_T)(MotorPtr_T p_motor, int16_t cmd);
typedef void (*Motor_User_SetScalar16_T)(MotorPtr_T p_motor, uint16_t cmd);
typedef void (*Motor_User_SetId_T)(MotorPtr_T p_motor, uint32_t enumValue);
typedef void (*Motor_User_SetEnum32_T)(MotorPtr_T p_motor, uint32_t enumValue);
typedef void (*Motor_User_SetFeedbackMode_T)(MotorPtr_T p_motor, Motor_FeedbackMode_T feedbackMode);
typedef bool (*Motor_User_ProcStatus_T)(MotorPtr_T p_motor);
typedef bool (*Motor_User_CheckStatus_T)(const MotorPtr_T p_motor);

typedef bool (*Motor_User_SetLimit_T)(MotorPtr_T p_motor, uint16_t scalar16, uint8_t id);
typedef bool (*Motor_User_ClearLimit_T)(MotorPtr_T p_motor, uint8_t id);

/*
    Motor_User_ReleaseControl
    Motor_User_DisableControl
    Motor_User_TryHold
*/
static inline void MotorN_User_ProcFunction(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_ProcVoid_T function)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor]); }
}

static inline void MotorN_User_SetCmd(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_SetCmd_T function, int16_t cmdValue)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor], cmdValue); }
}

static inline void MotorN_User_SetScalar16(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_SetScalar16_T function, uint16_t scalar16)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor], scalar16); }
}

static inline void MotorN_User_SetId(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_SetId_T function, uint32_t cmdValue)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor], cmdValue); }
}

//todo remove 1 arg
// static inline void MotorN_User_SetFeedbackMode(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_SetFeedbackMode_T function, Motor_FeedbackMode_T feedbackMode)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_motorArray[iMotor], feedbackMode); }
// }

static inline void MotorN_User_SetFeedbackMode(MotorPtr_T p_motorArray, uint8_t motorCount,  Motor_FeedbackMode_T feedbackMode)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { Motor_User_ActivateFeedbackMode(&p_motorArray[iMotor], feedbackMode); }
}

/*
    Motor_User_TryDirectionForward
    Motor_User_TryDirectionReverse
    MotorN_User_ClearFault
*/
static inline bool MotorN_User_ProcStatusAnd(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_ProcStatus_T function)
{
    bool isSet = true;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motorArray[iMotor]) == false) { isSet = false; } }
    return isSet;
}

/*
*/
static inline bool MotorN_User_ProcStatusOr(MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_ProcStatus_T function)
{
    bool isSet = false;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motorArray[iMotor]) == true) { isSet = true; } }
    return isSet;
}

/*
    MotorN_User_CheckFault
*/
static inline bool MotorN_User_CheckStatusOr(const MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_CheckStatus_T function)
{
    bool isSet = false;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motorArray[iMotor]) == true) { isSet = true; break; } }
    return isSet;
}

/*
    MotorN_User_CheckStop
*/
static inline bool MotorN_User_CheckStatusAnd(const MotorPtr_T p_motorArray, uint8_t motorCount, Motor_User_CheckStatus_T function)
{
    bool isSet = true;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motorArray[iMotor]) == false) { isSet = false; break; } }
    return isSet;
}

/*
    Motor_User_SetILimitActive_Id
    Motor_User_ClearLimit
*/
/*! @return true if at least one is set */
static inline bool MotorN_User_SetLimit(MotorPtr_T p_motor, uint8_t motorCount, Motor_User_SetLimit_T function, uint16_t limit_scalar16, uint8_t id)
{
    bool isSet = false;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motor[iMotor], limit_scalar16, id) == true) { isSet = true; }; }
    return isSet;
}

/*! @return true if at least one is cleared */
static inline bool MotorN_User_ClearLimit(MotorPtr_T p_motor, uint8_t motorCount, Motor_User_ClearLimit_T function, uint8_t id)
{
    bool isSet = false;
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_motor[iMotor], id) == true) { isSet = true; }; }
    return isSet;
}



#endif
