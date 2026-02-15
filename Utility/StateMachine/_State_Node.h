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
// #ifndef NDEBUG
//     assert(_State_IterateUp(p_state, p_state->DEPTH) == p_state->P_TOP);
// #endif
    // assert((p_state->P_TOP == NULL) || (p_state->P_PARENT != NULL));
/* Compile Time define Top State */
/* A top state may define its own P_TOP as NULL or itself */
/* alternatively iterate if not defined */
static inline State_T * _State_GetRoot(State_T * p_start)
{
    return (p_start->P_TOP == NULL) ? p_start : p_start->P_TOP;
}

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline bool State_IsSubState(State_T * p_state) { return (p_state->DEPTH > 0U); }


/******************************************************************************/
/*
    Generic [Parent Node Tree] Relations
*/
/******************************************************************************/
extern bool State_IsAncestor(State_T * p_reference, State_T * p_isAncestor);
extern bool State_IsDescendant(State_T * p_reference, State_T * p_isDescendant);
extern bool State_IsAncestorOrSelf(State_T * p_active, State_T * p_test);
extern bool State_IsDirectLineage(State_T * p_active, State_T * p_test);
extern State_T * State_CommonAncestorOf(State_T * p_state1, State_T * p_state2);


/******************************************************************************/
/*
    Transition Process
*/
/******************************************************************************/
extern void State_TraverseOnTransitionThrough(State_T * p_start, State_T * p_common, State_T * p_end, void * p_context);
extern void State_TraverseOnTransition(State_T * p_start, State_T * p_end, void * p_context);

/*  */
// extern State_Input_T State_TraverseAcceptInput(State_T * p_start, void * p_context, state_input_t inputId);
extern State_T * State_TraverseTransitionOfInput(State_T * p_start, void * p_context, state_input_t inputId, state_value_t inputValue);
extern State_T * State_TraverseTransitionOfInputRange(State_T * p_start, void * p_context, uint8_t stopLevel, state_input_t inputId, state_value_t inputValue);


extern State_T * State_TraverseTransitionOfOutputRange(State_T * p_start, void * p_context, uint8_t stopLevel);
// extern State_T * State_TraverseTransitionOfOutput(State_T * p_start, State_T * p_end, void * p_context);