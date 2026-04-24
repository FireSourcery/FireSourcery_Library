#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   HAL_Types.h
    @author FireSourcery
    @brief  Common include type before function signitures.
*/
/******************************************************************************/
#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

/*

*/

typedef FTM_Type HAL_ClockTimer_T;
typedef ADC_Type HAL_ADC_T;
typedef MSCAN_Type HAL_CAN_T;

#if     defined(KE06Z4_SERIES)
typedef FTMRE_Type HAL_Flash_T;
#elif   defined(KE02Z4_SERIES)
typedef FTMRH_Type HAL_Flash_T;
#endif
