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

const char * const INVALID_RETURN_CODE_STRING = "Invalid Return Code";

//static inline int32_t StringCompare(const char * p_str1, const char * p_str2)
//{
//	const char* p_string1 = p_str1;
//	const char* p_string2 = p_str2;
//	int32_t diff = 0;
//
////	for (uint32_t index = 0; index < length; index++)
////	{
////		if (*p_string1 != *p_string2)
////		{
////			diff = (int32_t) *(p_string1) - (int32_t) *(p_string2);
////		}
////		p_string1++;
////		p_string2++;
////	}
//
//	while ((*p_string1 != '\0') && (*p_string2 != '\0')) //set length boundary
//	{
//		if (*p_string1 != *p_string2)
//		{
//			diff = (int32_t) *(p_string1) - (int32_t) *(p_string2);
//			break;
//		}
//		p_string1++;
//		p_string2++;
//	}
//
//	return diff;
//}

Cmd_T * Cmd_Search(const Cmd_T * p_cmdTable, uint8_t tableLength, const char * p_cmdName)
{
	Cmd_T * p_cmd = 0U;

	if (p_cmdName != '\0')
	{
		for (uint8_t idx = 0U; idx < tableLength; idx++)
		{
	//		if (StringCompare(p_cmdName, p_cmdTable[idx].P_NAME) == 0)
			if (strcmp(p_cmdName, p_cmdTable[idx].P_NAME) == 0)
			{
				p_cmd = &p_cmdTable[idx];
				break;
			}
		}
	}
	return p_cmd;
}

const char * Cmd_SearchReturnString(const Cmd_Status_T * p_returnTable, uint8_t tableLength, int returnCodeId)
{
//#ifdef SHELL_OPTION_USE_ARRAY_TABLE

	const char * p_returnCodeString = INVALID_RETURN_CODE_STRING;

//	for(uint8_t idx = 0; idx < MainShell.ReturnCodeCount; idx++)
//	{
//		if (MainShell.p_ReturnCodeTable[idx].ReturnCode == returnCode)
//		{
//			p_returnCodeString = MainShell.p_ReturnCodeTable[idx].p_ReturnCodeString;
//			break;
//		}
//	}
	// if consecutive return code Ids
	if (returnCodeId < tableLength)
	{
		if(&p_returnTable[returnCodeId] != 0U)
		{
			p_returnCodeString = p_returnTable[returnCodeId].P_STRING;
		}
	}
//#endif

	return p_returnCodeString;
}
