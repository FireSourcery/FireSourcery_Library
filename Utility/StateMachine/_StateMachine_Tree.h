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
    @file   _StateMachine_Tree.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_StateMachine.h"

/******************************************************************************/
/*!
    HSM Wrap
*/
/******************************************************************************/
static inline State_T * StateMachine_GetRootState(const StateMachine_Active_T * p_active) { return p_active->p_ActiveState; }
static inline State_T * StateMachine_GetLeafState(const StateMachine_Active_T * p_active) { return (p_active->p_ActiveSubState == NULL) ? p_active->p_ActiveState : p_active->p_ActiveSubState; }

static inline bool StateMachine_IsLeafState(const StateMachine_Active_T * p_active, State_T * p_state) { return (p_state == StateMachine_GetLeafState(p_active)); }

/* State is within the active branch. */
static inline bool StateMachine_IsActiveBranch(const StateMachine_Active_T * p_active, State_T * p_state) { return State_IsActiveBranch(StateMachine_GetLeafState(p_active), p_state); }

/* Ancestor or Descendant */
static inline bool StateMachine_IsDirectBranch(const StateMachine_Active_T * p_active, State_T * p_state) { return State_IsDirectBranch(StateMachine_GetLeafState(p_active), p_state); }

/******************************************************************************/
/*!

*/
/******************************************************************************/
/* _StateMachine_Branch */
extern void _StateMachine_TraverseTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);
extern void _StateMachine_TraverseTransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);
extern void _StateMachine_ProcBranchSyncOutput(StateMachine_Active_T * p_active, void * p_context, State_T * p_limit);
extern void _StateMachine_ProcBranchInputTransition(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);
extern void _StateMachine_ProcBranchSyncInput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_ProcBranchAsyncInputTransition(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_ProcBranchAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);


extern void _StateMachine_ProcBranch_Nested(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_ProcBranch(StateMachine_Active_T * p_active, void * p_context);
