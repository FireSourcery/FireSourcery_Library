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
    @file   Config.h
    @author FireSourcery
    @brief  Motor module preprocessor conditional compilation options and defaults

*/
/******************************************************************************/
#ifndef CONFIG_MOTOR_H
#define CONFIG_MOTOR_H

// #if     defined(CONFIG_MOTOR_DEBUG)
// #elif   defined(CONFIG_MOTOR_DEBUG_ENABLE)
//     #define CONFIG_MOTOR_OPEN_LOOP_ENABLE
//     #define CONFIG_MOTOR_DEBUG
// #elif   defined(CONFIG_MOTOR_DEBUG_DISABLE)
//   #define CONFIG_MOTOR_DEBUG 0U
// #else
    // #define CONFIG_MOTOR_DEBUG_DISABLE
    #define CONFIG_MOTOR_DEBUG (!NDEBUG)
// #endif

//depreciate
// #if     defined(CONFIG_MOTOR_I_SENSORS_NONINVERT)
// #elif   defined(CONFIG_MOTOR_I_SENSORS_INVERT)
// #else
//     #define CONFIG_MOTOR_I_SENSORS_INVERT
// #endif

#if     defined(CONFIG_MOTOR_I_SENSORS_AB)
#elif   defined(CONFIG_MOTOR_I_SENSORS_ABC)
#else
    #define CONFIG_MOTOR_I_SENSORS_ABC
#endif

#if     defined(CONFIG_MOTOR_V_SENSORS_ISOLATED)
#elif   defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
#else
    #define CONFIG_MOTOR_V_SENSORS_ANALOG
#endif

#if     defined(CONFIG_MOTOR_HALL_MODE_POLLING)
#elif   defined(CONFIG_MOTOR_HALL_MODE_ISR)
#else
    #define CONFIG_MOTOR_HALL_MODE_POLLING
#endif

#if     defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
#elif   defined(CONFIG_MOTOR_SENSOR_SIN_COS_DISABLE)
#else
    #define CONFIG_MOTOR_SENSOR_SIN_COS_DISABLE
#endif

#if     defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
#elif   defined(CONFIG_MOTOR_SENSOR_SENSORLESS_DISABLE)
#else
    #define CONFIG_MOTOR_SENSOR_SENSORLESS_DISABLE
#endif

#if     defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)
#elif   defined(CONFIG_MOTOR_OPEN_LOOP_DISABLE)
#else
    #define CONFIG_MOTOR_OPEN_LOOP_ENABLE
#endif

#if     defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#elif   defined(CONFIG_MOTOR_SIX_STEP_DISABLE)
#else
    #define CONFIG_MOTOR_SIX_STEP_DISABLE
#endif

#if     defined(CONFIG_MOTOR_FOC_ENABLE)
#elif   defined(CONFIG_MOTOR_FOC_DISABLE)
#else
    #define CONFIG_MOTOR_FOC_ENABLE
#endif

/* reboot for params to take effect when disabled */
#if     defined(CONFIG_MOTOR_CONFIG_PROPAGATE_SET_DISABLE)
#elif   defined(CONFIG_MOTOR_CONFIG_PROPAGATE_SET_ENABLE)
#else
        #define CONFIG_MOTOR_CONFIG_PROPAGATE_SET_ENABLE
#endif

#if     defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
    #if defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
    #endif
#elif   defined(CONFIG_MOTOR_UNIT_CONVERSION_HOST)
#else
    #define CONFIG_MOTOR_UNIT_CONVERSION_HOST
#endif

#if     defined(CONFIG_MOTOR_EXTERN_CONTROL_ENABLE)
#elif   defined(CONFIG_MOTOR_EXTERN_CONTROL_DISABLE)
#else
    #define CONFIG_MOTOR_EXTERN_CONTROL_DISABLE
#endif

#endif



