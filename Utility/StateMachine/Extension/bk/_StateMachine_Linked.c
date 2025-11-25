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
    @file   _StateMachine_Linked.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
// Your code here
#include "../StateMachine.h"

/******************************************************************************/
/*
    Linked Menus
*/
/******************************************************************************/
// #ifdef STATE_MACHINE_LINKED_STATES_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, state_input_t input, state_value_t inputValue)
{
    StateMachine_ApplyInput(p_stateMachine, input, inputValue)
}

State_T * StateMachine_Menu_GetPtrActive(StateMachine_T * p_stateMachine)
{
    return p_stateMachine->p_ActiveState;
}

void StateMachine_Menu_SetMenu(StateMachine_T * p_stateMachine, State_T * p_targetMenu)
{
    p_stateMachine->p_ActiveState = p_targetMenu;
}

void StateMachine_Menu_StartMenu(StateMachine_T * p_stateMachine, State_T * p_targetMenu)
{
    _StateMachine_Transition(p_stateMachine, p_targetMenu);
}

// does not run entry function
void StateMachine_Menu_SetNext(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_ActiveState->P_NEXT_MENU != NULL) { StateMachine_Menu_SetMenu(p_stateMachine, p_stateMachine->p_ActiveState->P_NEXT_MENU); }
}

// run entry function
void StateMachine_Menu_StartNext(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_ActiveState->P_NEXT_MENU != NULL) { StateMachine_Menu_StartMenu(p_stateMachine, p_stateMachine->p_ActiveState->P_NEXT_MENU); }
}

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, state_input_t input, state_value_t inputValue)
{
    StateMachine_ApplyInput(p_stateMachine, input, inputValue)
}

void StateMachine_Menu_ProcLoop(StateMachine_T * p_stateMachine)
{
    StateMachine_ProcState(p_stateMachine);
}
// #endif


