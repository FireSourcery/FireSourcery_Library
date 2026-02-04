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
#ifndef MOTOR_PREPROCESSOR_H
#define MOTOR_PREPROCESSOR_H

#define MOTOR_DEBUG (!NDEBUG)

// #if     defined(MOTOR_I_SENSORS_AB)
// #elif   defined(MOTOR_I_SENSORS_ABC)
// #else
//     #define MOTOR_I_SENSORS_ABC
// #endif

// #if     defined(MOTOR_V_SENSORS_ISOLATED)
// #elif   defined(MOTOR_V_SENSORS_ANALOG)
// #else
//     #define MOTOR_V_SENSORS_ANALOG
// #endif


#if     defined(MOTOR_OPEN_LOOP_ENABLE)
#elif   defined(MOTOR_OPEN_LOOP_DISABLE)
#else
    #define MOTOR_OPEN_LOOP_ENABLE
#endif

#if     defined(MOTOR_SIX_STEP_ENABLE)
#elif   defined(MOTOR_SIX_STEP_DISABLE)
#else
    #define MOTOR_SIX_STEP_DISABLE
#endif

#if     defined(MOTOR_FOC_ENABLE)
#elif   defined(MOTOR_FOC_DISABLE)
#else
    #define MOTOR_FOC_ENABLE
#endif

/* reboot for params to take effect when disabled */
#if     defined(MOTOR_CONFIG_PROPAGATE_SET_DISABLE)
#elif   defined(MOTOR_CONFIG_PROPAGATE_SET_ENABLE)
#else
        #define MOTOR_CONFIG_PROPAGATE_SET_ENABLE
#endif

#if     defined(MOTOR_UNIT_CONVERSION_LOCAL)
    #if defined(MOTOR_SURFACE_SPEED_ENABLE)
    #endif
#elif   defined(MOTOR_UNIT_CONVERSION_HOST)
#else
    #define MOTOR_UNIT_CONVERSION_HOST
#endif

#if     defined(MOTOR_EXTERN_CONTROL_ENABLE)
#elif   defined(MOTOR_EXTERN_CONTROL_DISABLE)
#else
    #define MOTOR_EXTERN_CONTROL_DISABLE
#endif

#endif



