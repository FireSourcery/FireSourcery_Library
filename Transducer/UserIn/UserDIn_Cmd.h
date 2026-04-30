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
    @file   UserDIn_Cmd.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "UserDIn.h"

typedef void (*UserDIn_Fn_T)(void * p_context, UserDIn_Edge_T edge);
// typedef void (*UserDIn_Fn_T)(void * p_context, bool prevState, bool currentState);

/*  */
typedef const struct UserDIn_Cmd
{
    // UserDIn_Edge_T Edge;
    UserDIn_Fn_T CMD;
    void * P_CONTEXT;
}
UserDIn_Cmd_T;

static void UserDIn_CmdNull(void * p_context, UserDIn_Edge_T edge) { (void)p_context; (void)edge; }
static const UserDIn_Fn_T USER_DIN_FN_NULL = UserDIn_CmdNull;
static const UserDIn_Cmd_T USER_DIN_CMD_NULL = { .CMD = UserDIn_CmdNull, .P_CONTEXT = NULL };

// Your code here
static inline void UserDIn_PollEdgeCmd(UserDIn_T * p_dev, UserDIn_Cmd_T * p_cmd)
{
    // UserDIn_Edge_T edge = UserDIn_Modal_PollEdgeValue(p_dev); with disable
    p_cmd->CMD(p_cmd->P_CONTEXT, UserDIn_PollEdgeValue(p_dev));
}

