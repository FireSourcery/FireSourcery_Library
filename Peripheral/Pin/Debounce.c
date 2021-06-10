/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
/**************************************************************************/
/**************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#include "Debounce.h"

#include "HAL_Pin.h"

#include <stdint.h>
#include <stdbool.h>

void Debounce_Init
(
	Debounce_T * p_pin,
	const volatile HAL_Pin_T * p_hal_pin,
	const volatile uint32_t * p_Timer,
	uint16_t debounceTime
)
{
	p_pin->p_HAL_Pin = p_hal_pin;

	p_pin->p_Timer = p_Timer;
	p_pin->DebounceTime = debounceTime;

	p_pin->DebouncedState = HAL_Pin_ReadState(p_pin->p_HAL_Pin);
	p_pin->DebouncedStatePrev = p_pin->DebouncedState;
	p_pin->RawStatePrev = p_pin->DebouncedState;
}
