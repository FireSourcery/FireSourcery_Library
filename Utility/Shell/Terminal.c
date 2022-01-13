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
	@file  	Terminal.h
	@author FireSourcery
	@brief	Serial text support functions for terminal ui
	@version V0
 */
/******************************************************************************/
#include "Terminal.h"

#include "Peripheral/Serial/Serial.h"

#include <stdio.h>

void Terminal_Init(Terminal_T * p_terminal )
{
	//#ifdef CONFIG_SHELL_USE_SERIAL
//	p_terminal->p_Serial = p_connect;
	//#endif
	p_terminal->CursorIndex = 0;
}

void Terminal_SetXcvr(Terminal_T * p_terminal, void * p_connect)
{
	p_terminal->p_Serial = p_connect;
}





