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



/*
    Base Module holds pointer or opaque pointer to decouple from service
*/
typedef const struct UserDIn_CmdTable
{
    UserDIn_Fn_T * P_CMD_TABLE;
    uint8_t LENGTH;
    void * P_CONTEXT;
}
UserDIn_CmdTable_T;

// static const UserDIn_Cmd_T USER_DIN_CMD_NULL = { .CMD = UserDIn_CmdNull, .P_CONTEXT = NULL };

// static inline void UserDIn_PollEdgeCmd(UserDIn_T * p_dev, UserDIn_Cmd_T * p_cmd)
// {
//     p_cmd->CMD(p_cmd->P_CONTEXT, UserDIn_Modal_PollEdgeValue(p_dev));
// }

// static inline void _UserDIn_PollEdgeCmd(UserDIn_T * p_dev, void * p_context)
// {
//     UserDIn_Edge_T edge = UserDIn_Modal_PollEdgeValue(p_dev);
//     if (edge != USER_DIN_EDGE_NULL) { p_dev->P_STATE->OptCmd(p_context, edge); }
// }

/*
    Array helper
*/
static inline void UserDIn_Array_ResolveCallbacks(UserDIn_T * p_dins, uint8_t count, const UserDIn_Fn_T * p_cmdTable)
{
    for (uint8_t i = 0; i < count; i++) { p_dins[i].P_STATE->OptCmd = p_cmdTable[p_dins[i].P_NVM_CONFIG->CmdId]; }
}


// static inline bool UserDIn_Array_IsCmdIdPresent(UserDIn_T * p_dins, uint8_t count,  uint8_t cmdid)
// {
//     for (uint8_t i = 0; i < count; i++)
//     {
//         if (p_dins[i].P_NVM_CONFIG->CmdId == cmdid) { return true; }
//     }
//     return false;
// }

