#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   HAL_Encoder.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

#if defined(HAL_ENCODER_PATH)
    #define XSTR(String) #String
    #define STR(String) XSTR(String)
    #include STR(HAL_ENCODER_PATH/HAL_Encoder.h)
#elif defined(HAL_PERIPHERAL_PATH_DIRECTORY) || defined(HAL_PERIPHERAL_PATH_PLATFORM)
    #include "Peripheral/HAL/HAL_Peripheral.h"
    #include HAL_PERIPHERAL_PATH(HAL_Encoder.h)
#endif
