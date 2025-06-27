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
    @file   HAL_FastPin.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

typedef FGPIO_Type HAL_FastPin_T;

static inline void HAL_FastPin_WriteOutputOn(HAL_FastPin_T * p_hal, uint32_t pinId) { (p_hal->PSOR |= pinId); }
static inline void HAL_FastPin_WriteOutputOff(HAL_FastPin_T * p_hal, uint32_t pinId) { (p_hal->PCOR |= pinId); }
static inline void HAL_FastPin_WriteOutput(HAL_FastPin_T * p_hal, uint32_t pinId, bool isOn) { p_hal->PDOR = isOn ? (p_hal->PDOR | pinId) : (p_hal->PDOR & ~(pinId)); }

static inline void HAL_FastPin_ToggleOutput(HAL_FastPin_T * p_hal, uint32_t pinId) { (p_hal->PTOR |= pinId); }
static inline bool HAL_FastPin_ReadOutput(HAL_FastPin_T * p_hal, uint32_t pinId) { return ((p_hal->PDOR & pinId) == pinId); }

static inline bool HAL_FastPin_ReadInput(const HAL_FastPin_T * p_hal, uint32_t pinId) { return ((p_hal->PDIR & pinId) == pinId); }

static inline void HAL_FastPin_InitInput(HAL_FastPin_T * p_hal, uint32_t pinId)     { p_hal->PDDR &= ~(pinId); p_hal->PIDR &= ~(pinId); }
static inline void HAL_FastPin_InitOutput(HAL_FastPin_T * p_hal, uint32_t pinId)    { p_hal->PDDR |= (pinId); p_hal->PIDR |= (pinId); }
static inline void HAL_FastPin_Deinit(HAL_FastPin_T * p_hal, uint32_t pinId)        { p_hal->PDDR &= ~(pinId); p_hal->PIDR |= (pinId); }
