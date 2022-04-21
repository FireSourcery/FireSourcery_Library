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
	@file  	Cmd.c
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#include "Cmd.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

Cmd_T * Cmd_Search(const Cmd_T * p_cmdTable, uint8_t tableLength, const char * p_cmdName)
{
	Cmd_T * p_cmd = 0U;

	if ((p_cmdName != 0U) && (p_cmdName[0U] != '\0'))
	{
		for (uint8_t idx = 0U; idx < tableLength; idx++)
		{
			if (strcmp(p_cmdName, p_cmdTable[idx].P_NAME) == 0)
			{
				p_cmd = &p_cmdTable[idx];
				break;
			}
		}
	}

	return p_cmd;
}

