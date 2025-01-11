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
    @file
    @author
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef HAL_PIN_PLATFORM_H
#define HAL_PIN_PLATFORM_H

#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

typedef GPIO_Type HAL_Pin_T;

static inline void HAL_Pin_WriteOutputOn(HAL_Pin_T * p_hal, uint32_t pinId) { (p_hal->PSOR |= pinId); }
static inline void HAL_Pin_WriteOutputOff(HAL_Pin_T * p_hal, uint32_t pinId) { (p_hal->PCOR |= pinId); }
static inline void HAL_Pin_WriteOutput(HAL_Pin_T * p_hal, uint32_t pinId, bool isOn) { isOn ? (p_hal->PDOR |= pinId) : (p_hal->PDOR &= ~(pinId)); }
static inline void HAL_Pin_ToggleOutput(HAL_Pin_T * p_hal, uint32_t pinId) { (p_hal->PTOR |= pinId); }
// static inline bool HAL_Pin_ReadOutput(HAL_Pin_T * p_hal, uint32_t pinId) { return ((p_hal->PDOR & pinId) != 0U); }
static inline bool HAL_Pin_ReadOutput(HAL_Pin_T * p_hal, uint32_t pinId) { return ((p_hal->PDOR & pinId) == pinId); }

static inline bool HAL_Pin_ReadInput(const HAL_Pin_T * p_hal, uint32_t pinId) { return ((p_hal->PDIR & pinId) == pinId); }

static inline void HAL_Pin_InitInput(HAL_Pin_T * p_hal, uint32_t pinId)     { p_hal->PDDR &= ~(pinId); p_hal->PIDR &= ~(pinId); }
static inline void HAL_Pin_InitOutput(HAL_Pin_T * p_hal, uint32_t pinId)    { p_hal->PDDR |= (pinId); p_hal->PIDR |= (pinId); }
static inline void HAL_Pin_Deinit(HAL_Pin_T * p_hal, uint32_t pinId)        { p_hal->PDDR &= ~(pinId); p_hal->PIDR |= (pinId); }

#endif
