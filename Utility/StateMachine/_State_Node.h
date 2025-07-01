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
    @file   _State_Node.h
    @author FireSourcery
    @brief  Private inline for StateMachine include
*/
/******************************************************************************/
#include "State.h"

/******************************************************************************/
/*
    [Hierarchical State Machine]
    [State] as Node/Tree
*/
/******************************************************************************/

/*
    Nth Ancestor
    No NULL check, as this is a private function for compile time defined values
*/
static inline const State_T * _State_IterateUp(const State_T * p_start, int8_t iterations)
{
    const State_T * p_iterator = p_start;
    for (int8_t count = 0; count < iterations; count++) { p_iterator = p_iterator->P_PARENT; }
    return p_iterator;
}

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline bool State_IsSubState(const State_T * p_state) { return (p_state->DEPTH > 0U); }


/******************************************************************************/
/*
    Generic [Parent Node Tree] Relations
*/
/******************************************************************************/
extern bool State_IsAncestor(const State_T * p_reference, const State_T * p_isAncestor);
extern bool State_IsDescendant(const State_T * p_reference, const State_T * p_isDescendant);
extern bool State_IsActiveBranch(const State_T * p_active, const State_T * p_test);
extern bool State_IsDirectBranch(const State_T * p_active, const State_T * p_test);
extern State_T * State_CommonAncestorOf(const State_T * p_state1, const State_T * p_state2);


/******************************************************************************/
/*
    Transition Process
*/
/******************************************************************************/
extern void State_TraverseTransitionThrough(const State_T * p_start, const State_T * p_common, const State_T * p_end, void * p_context);
extern void State_TraverseTransition(const State_T * p_start, const State_T * p_end, void * p_context);
extern State_Input_T State_TraverseAcceptInput(const State_T * p_start, void * p_context, state_input_t inputId);
extern State_T * State_TraverseTransitionOfContext(const State_T * p_start, const State_T * p_end, void * p_context);
extern State_T * State_TraverseTransitionOfInput(const State_T * p_start, void * p_context, state_input_t inputId, state_input_value_t inputValue);