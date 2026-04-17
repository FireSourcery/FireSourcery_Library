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
    @file   HAL_ClockTimer.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_ClockTimer.h)

// static inline void HAL_ClockTimer_ClearOverflow(HAL_ClockTimer_T * p_timer);
// static inline bool HAL_ClockTimer_ReadOverflow(const HAL_ClockTimer_T * p_timer);
// static inline uint32_t HAL_ClockTimer_Read(const HAL_ClockTimer_T * p_timer);
// static inline void HAL_ClockTimer_Write(HAL_ClockTimer_T * p_timer, uint32_t count);
// static inline void HAL_ClockTimer_Enable(HAL_ClockTimer_T * p_timer);


static inline uint32_t HAL_ClockTimer_CapturePeriodT(HAL_ClockTimer_T * p_hal)
{
    bool isOverflow = HAL_ClockTimer_ReadOverflow(p_hal);
    if (isOverflow) { HAL_ClockTimer_ClearOverflow(p_hal); }
    HAL_ClockTimer_Write(p_hal, 0U);
    return isOverflow ? HAL_CLOCK_TIMER_MAX : HAL_ClockTimer_Read(p_hal);
}