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
#ifndef CMD_H
#define CMD_H

#include <stdint.h>

typedef int (*Cmd_Function_T)(void * p_context, int argc, char * argv[]);

typedef int (*Cmd_Loop_T)(void * p_context);

typedef const struct
{
    const char * const P_NAME;
    const char * const P_HELP;
    const Cmd_Function_T FUNCTION;
    const Cmd_Loop_T LOOP;
//    Cmd_Type_T
    // ArgCMax;
}
Cmd_T;


#define CMD_RESERVED_RETURN_CODE_SUCCESS 		(0U)
#define CMD_RESERVED_RETURN_CODE_INVALID_ARGS 	(-1) //Must be implement at cmd function level / per cmd function
//#define CMD_RESERVED_RETURN_CODE_ARGS_ERROR		(-2)
#define CMD_RESERVED_RETURN_CODE_LOOP 			(-3)

//typedef enum
//{
//	CMD_RESERVED_RETURN_CODE_SUCCESS = 0,
//	CMD_RESERVED_RETURN_CODE_INVALID_ARGS = -1,
//}
//Cmd_ReservedReturnCode_T;

typedef const struct
{
	const int CODE_ID;
	const char * const P_STRING;
}
Cmd_Status_T;


//#ifdef SHELL_OPTION_USE_ARRAY_TABLE
#define CMD_ENTRY(CmdName, CmdHelpString, CmdFunction) { (#CmdName), (CmdHelpString), (CmdFunction) }
#define CMD_RETURN_ENTRY(ReturnCode) { (ReturnCode), (#ReturnCode) }
//#endif

#endif
