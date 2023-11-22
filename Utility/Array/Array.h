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
// /******************************************************************************/
// /*!
//     @file   Array.h
//     @author FireSourcery
//     @brief
//     @version V0
// */
// /******************************************************************************/
// #ifndef MOTOR_N_USER_H
// #define MOTOR_N_USER_H

// #include "Array.h"
// #include <stdint.h>
// #include <stdbool.h>

// // #define FOREACH(item, array) for (int i = 0; i < sizeof(array) / sizeof(array[0]); i++) \
// // for (item = &array[i]; &array[i] != &array[sizeof(array) / sizeof(array[0])]; i++, item = &array[i])

// #define FOREACH(item, array) for (int i = 0; i < sizeof(array) / sizeof(array[0]); i++)

// typedef void(*Array_ProcVoid_T)(void * p_item);
// typedef void(*Array_SetCmd_T)(void * p_item, int16_t cmd);
// typedef void(*Array_SetScalar16_T)(void * p_item, uint16_t cmd);
// typedef void(*Array_SetId_T)(void * p_item, uint32_t enumValue);
// // typedef void(*Array_SetFeedbackMode_T)(void * p_item, struct feedbackMode);
// typedef bool(*Array_ProcStatus_T)(void * p_item);

// /*
//     Array_ReleaseControl
//     Array_DisableControl
//     Array_Hold
// */
// static inline void Array_ProcFunction(void ** pp_array, uint8_t motorCount, Array_ProcVoid_T function)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(pp_array[iMotor]); }

//     FOREACH(item, pp_array) function(item);
// }

// static inline void Array_SetCmd(void * p_array, uint8_t motorCount, Array_SetCmd_T function, int16_t cmdValue)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_array[iMotor], cmdValue); }
// }

// static inline void Array_SetScalar16(void * p_array, uint8_t motorCount, Array_SetScalar16_T function, uint16_t scalar16)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_array[iMotor], scalar16); }
// }

// static inline void Array_SetId(void * p_array, uint8_t motorCount, Array_SetId_T function, uint32_t cmdValue)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_array[iMotor], cmdValue); }
// }

// static inline void Array_SetFeedbackMode(void * p_array, uint8_t motorCount, Array_SetFeedbackMode_T function, Motor_FeedbackMode_T feedbackMode)
// {
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { function(&p_array[iMotor], feedbackMode); }
// }

// /*
//     Array_SetDirectionForward
//     Array_SetDirectionReverse
//     Array_ClearFault
// */
// static inline bool Array_ProcStatusAnd(void * p_array, uint8_t motorCount, Array_ProcStatus_T function)
// {
//     bool isSet = true;
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_array[iMotor]) == false) { isSet = false; } }
//     return isSet;
// }

// /*
// */
// static inline bool Array_ProcStatusOr(void * p_array, uint8_t motorCount, Array_ProcStatus_T function)
// {
//     bool isSet = false;
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_array[iMotor]) == true) { isSet = true; } }
//     return isSet;
// }

// /*
//     Array_CheckFault
// */
// static inline bool Array_CheckStatusOr(void * p_array, uint8_t motorCount, Array_ProcStatus_T function)
// {
//     bool isSet = false;
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_array[iMotor]) == true) { isSet = true; break; } }
//     return isSet;
// }

// /*
//     Array_CheckStop
// */
// static inline bool Array_CheckStatusAnd(void * p_array, uint8_t motorCount, Array_ProcStatus_T function)
// {
//     bool isSet = true;
//     for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { if(function(&p_array[iMotor]) == false) { isSet = false; break; } }
//     return isSet;
// }


// #endif
