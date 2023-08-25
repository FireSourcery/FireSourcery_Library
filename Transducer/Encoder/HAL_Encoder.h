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
    @file    HAL.h
    @author FireSourcery
    @brief     User must provide HW functions, or configure HAL
    @version V0
 */
/******************************************************************************/
#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#define XSTR(String) #String
#define STR(String) XSTR(String)

#if defined(CONFIG_HAL_ENCODER_PATH)
    #include STR(CONFIG_HAL_ENCODER_PATH/HAL_Encoder.h)
#elif defined(CONFIG_HAL_ENCODER_PLATFORM)
    #include STR(HAL/Platform/CONFIG_HAL_ENCODER_PLATFORM/HAL_Encoder.h)
#elif defined(CONFIG_HAL_PERIPHERAL_PATH) || defined(CONFIG_HAL_PERIPHERAL_PLATFORM)
    #include HAL_PERIPHERAL_PATH(HAL_Encoder.h)
#endif

#endif
