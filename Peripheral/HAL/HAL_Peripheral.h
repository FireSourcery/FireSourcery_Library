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
    @file   HAL_Peripheral.h
    @author FireSourcery
    @brief  Define configurable HAL Peripheral Path
    @version V0
*/
/******************************************************************************/
#ifndef HAL_PERIPHERAL_H
#define HAL_PERIPHERAL_H

#define XSTR(String) #String
#define STR(String) XSTR(String)

#if     defined(CONFIG_HAL_PERIPHERAL_PATH)         /* External directory */
    #define HAL_PERIPHERAL_PATH(File) STR(CONFIG_HAL_PERIPHERAL_PATH/File)
#elif   defined(CONFIG_HAL_PERIPHERAL_PLATFORM)     /* Library platform directory */
    #define HAL_PERIPHERAL_PATH(File) STR(Peripheral/HAL/Platform/CONFIG_HAL_PERIPHERAL_PLATFORM/File)
#endif

#endif
