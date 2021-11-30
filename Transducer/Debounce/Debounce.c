/******************************************************************************/
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
/******************************************************************************/
/******************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Debounce.h"

#include "Peripheral/Pin/Pin.h"

#include <stdint.h>
#include <stdbool.h>

void Debounce_Init(Debounce_T * p_debounce, uint16_t debounceTime)
{
	Pin_Input_Init(&p_debounce->CONFIG.PIN);
	p_debounce->DebounceTime = debounceTime;

	p_debounce->DebouncedState 		= Pin_Input_Read(&p_debounce->CONFIG.PIN);
	p_debounce->DebouncedStatePrev 	= p_debounce->DebouncedState;
	p_debounce->RawStatePrev 		= p_debounce->DebouncedState;
}
