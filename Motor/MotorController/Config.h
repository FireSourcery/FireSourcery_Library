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
    @brief  Motor module preprocessor configuration options and defaults
    @version V0
*/
/******************************************************************************/
#ifndef CONFIG_MOTOR_CONTROLLER_H
#define CONFIG_MOTOR_CONTROLLER_H


#if     defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_EEPROM)
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_FLASH)
#else
    #define CONFIG_MOTOR_CONTROLLER_USER_CONFIG_EEPROM
#endif

/* For Protocol Flash Only */
#if     defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
#elif   defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_DISABLE)
#else
    #define CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_DISABLE
#endif

#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_ONCE)
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_FLASH)
#else
    #define CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_ONCE
#endif

#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_RAM_ENABLE)
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_RAM_DISABLE)
#else
    #define CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_RAM_ENABLE
#endif

#if     defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
#elif   defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_DISABLE)
#else
    #define CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE
#endif

#if     defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
#elif   defined(CONFIG_MOTOR_CONTROLLER_SHELL_DISABLE)
#else
    #define CONFIG_MOTOR_CONTROLLER_SHELL_DISABLE
#endif

#if     defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
#elif   defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_DISABLE)
#else
    #define CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_DISABLE
#endif

#if     defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
#elif   defined(CONFIG_MOTOR_CONTROLLER_DEBUG_DISABLE)
#else
    #define CONFIG_MOTOR_CONTROLLER_DEBUG_DISABLE
#endif

#if     defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE)
    #if     defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    #elif   defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_DISABLE)
    #else
        #define CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_DISABLE
    #endif
#elif   defined(CONFIG_MOTOR_CONTROLLER_SERVO_DISABLE)
#else
    #define CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
#endif

#endif

