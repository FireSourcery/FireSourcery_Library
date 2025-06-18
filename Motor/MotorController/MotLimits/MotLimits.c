
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
    @file   .h
    @author FireSourcery

    @brief  Limits Input
*/
/******************************************************************************/
#include "MotLimits.h"

/******************************************************************************/
/*

*/
/******************************************************************************/


// /* Null sets reverse, lower value */
// static uint16_t SpeedLimitSentinelOf(const Motor_State_T * p_motor, uint16_t speed_ufract16)
// {
//     return (p_motor->Direction == p_motor->Config.DirectionForward) ?
//         math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16) : math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
// }

// static uint16_t ILimitMotoringSentinelOf(const Motor_State_T * p_motor, uint16_t i_Fract16)
// {
//     return math_limit_upper(i_Fract16, p_motor->Config.ILimitMotoring_Fract16);
// }

// static uint16_t ILimitMotoringSentinelOf_Scalar(const Motor_State_T * p_motor, uint16_t scalar_ufract16)
// {
//     return fract16_mul(p_motor->Config.ILimitMotoring_Fract16, scalar_ufract16);
// }

// /******************************************************************************/
// /*
//     Conditional -
//     Each source use unique entry id
//     returns true if the selected id becomes the active id
// */
// /******************************************************************************/
// bool Motor_SetSpeedLimitEntry(Motor_State_T * p_motor, uint8_t id, uint16_t speed_ufract16)
// {
//     uint16_t speedLimit = SpeedLimitSentinelOf(p_motor, speed_ufract16);
//     bool isActiveLimit = LimitArray_SetEntry(&p_motor->SpeedLimit, id, speedLimit);
//     if (isActiveLimit == true) { Motor_SetSpeedLimit(p_motor, speedLimit); }
//     return isActiveLimit;
// }

// // bool Motor_SetSpeedLimitEntry_Scalar(Motor_State_T * p_motor, uint8_t id, uint16_t scalar_ufract16)
// // {
// //     // int32_t speed_ufract16 = fract16_mul(Motor_GetSpeedLimitActive(p_motor), scalar_ufract16); // of base
// //     // bool isActiveLimit = LimitArray_SetEntry(&p_motor->SpeedLimit, id, speed_ufract16);
// //     // if (isActiveLimit == true) { Motor_SetSpeedLimitArray_Scalar(p_motor, scalar_ufract16); }
// //     // return isActiveLimit;
// // }

// bool Motor_ClearSpeedLimitEntry(Motor_State_T * p_motor, uint8_t id)
// {
//     bool isActiveLimit = LimitArray_ClearEntry(&p_motor->SpeedLimit, id);
//     if(isActiveLimit == true)
//     {
//         if (LimitArray_IsUpperActive(&p_motor->SpeedLimit) == true)
//             { Motor_SetSpeedLimit(p_motor, LimitArray_GetUpper(&p_motor->SpeedLimit)); }
//         else
//             { Motor_ClearSpeedLimit(p_motor); }
//     }
//     return isActiveLimit;
// }

/*
*/
/*! @return true if set */
// bool Motor_SetILimitMotoringEntry(Motor_State_T * p_motor, uint8_t id, uint16_t i_ufract16)
// {
//     int32_t iLimit = ILimitMotoringSentinelOf(p_motor, i_ufract16);
//     bool isActiveLimit = LimitArray_SetEntry(&p_motor->ILimit, id, iLimit);
//     if (isActiveLimit == true) { Motor_SetILimit(p_motor, iLimit); } /* alteratively maintain scalar comparison satisfy both sides */
//     return isActiveLimit;
// }

// bool Motor_SetILimitMotoringEntry_Scalar(Motor_State_T * p_motor, uint8_t id, uint16_t scalar_ufract16)
// {
//     int32_t iLimit = ILimitMotoringSentinelOf_Scalar(p_motor, scalar_ufract16);
//     bool isActiveLimit = LimitArray_SetEntry(&p_motor->ILimit, id, iLimit);
//     if (isActiveLimit == true) { Motor_SetILimitArray_Scalar(p_motor, iLimit); }
//     return isActiveLimit;
// }

// /*!
//     Restores previous limit
//     @param[in] id Motor_ILimitId_T
//     @return true if cleared. ILimit of input id
// */
// bool Motor_ClearILimitMotoringEntry(Motor_State_T * p_motor, uint8_t id)
// {
//     bool isActiveLimit = LimitArray_ClearEntry(&p_motor->ILimit, id);
//     if (isActiveLimit == true)
//     {
//         if (LimitArray_IsUpperActive(&p_motor->ILimit) == true)
//             { Motor_SetILimit(p_motor, LimitArray_GetUpper(&p_motor->ILimit)); }
//         else
//             { Motor_ClearILimit(p_motor); }
//     }
//     return isActiveLimit;
// }


// static inline bool Motor_User_TrySpeedLimit(Motor_State_T * p_motor, motor_value_t speed_fract16) { return Motor_SetSpeedLimitEntry(p_motor, MOTOR_SPEED_LIMIT_USER, (uint16_t)speed_fract16); }
// static inline bool Motor_User_ClearSpeedLimit(Motor_State_T * p_motor)                            { return Motor_ClearSpeedLimitEntry(p_motor, MOTOR_SPEED_LIMIT_USER); }
// static inline bool Motor_User_TryILimit(Motor_State_T * p_motor, motor_value_t i_fract16)         { return Motor_SetILimitMotoringEntry(p_motor, MOTOR_I_LIMIT_USER, (uint16_t)i_fract16); }
// static inline bool Motor_User_ClearILimit(Motor_State_T * p_motor)                                { return Motor_ClearILimitMotoringEntry(p_motor, MOTOR_I_LIMIT_USER); }
