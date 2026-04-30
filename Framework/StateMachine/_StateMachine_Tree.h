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
    @brief  Hierarchical State Machine
            via Parent Node Tree, following most UML conventions
*/
/******************************************************************************/
#include "_StateMachine.h"

/******************************************************************************/
/*!
    HSM Active State
*/
/******************************************************************************/
static inline State_T * StateMachine_GetRootState(const StateMachine_Active_T * p_active) { return _State_GetRoot(p_active->p_ActiveState); }
static inline State_T * StateMachine_GetLeafState(const StateMachine_Active_T * p_active) { return p_active->p_ActiveState; }


static inline bool StateMachine_IsRootState(const StateMachine_Active_T * p_active, State_T * p_state) { return (p_state == StateMachine_GetRootState(p_active)); }
static inline bool StateMachine_IsLeafState(const StateMachine_Active_T * p_active, State_T * p_state) { return (p_state == StateMachine_GetLeafState(p_active)); }

/* Match up along the active path */
static inline bool StateMachine_IsActiveBranch(const StateMachine_Active_T * p_active, State_T * p_state) { return State_IsAncestorOrSelf(StateMachine_GetLeafState(p_active), p_state); }

/* Ancestor or Descendant */
static inline bool StateMachine_IsDirectBranch(const StateMachine_Active_T * p_active, State_T * p_state) { return State_IsDirectLineage(StateMachine_GetLeafState(p_active), p_state); }

static inline state_t StateMachine_GetRootStateId(const StateMachine_Active_T * p_active) { return StateMachine_GetRootState(p_active)->ID; }
static inline state_t StateMachine_GetLeafStateId(const StateMachine_Active_T * p_active) { return StateMachine_GetLeafState(p_active)->ID; }

/* ActiveStateOfBranch */
static inline state_t StateMachine_GetActiveSubStateId(const StateMachine_Active_T * p_active, State_T * p_ancestor)
{
    return State_IsAncestor(StateMachine_GetLeafState(p_active), p_ancestor) ? p_active->p_ActiveState->ID : STATE_ID_NULL;
}

/* depreciate for pointer compare */
static inline bool StateMachine_IsRootStateId(const StateMachine_Active_T * p_active, state_t stateId) { return (stateId == StateMachine_GetRootState(p_active)->ID); }


// static inline State_PathId_T StateMachine_GetPathId(const StateMachine_Active_T * p_active)
// {
//     assert(StateMachine_GetLeafState(p_active)->DEPTH < 8); /* Ensure depth fits within 4-bit fields */

//     uint32_t id = 0;
//     for (State_T * p_iterator = StateMachine_GetLeafState(p_active); p_iterator != NULL; p_iterator = p_iterator->P_PARENT)
//     {
//         id |= ((uint32_t)p_iterator->ID << (p_iterator->DEPTH * 4));
//     }
//     return (State_PathId_T){ .BranchId = id };
// }

/*

*/
// static inline state_value_t _StateMachine_Branch_Access(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t valueK, state_value_t valueV)
// {
//     return State_Access(p_active->p_ActiveSubState, p_context, id, valueK, valueV);
// }

// static inline void _StateMachine_Branch_SetValue(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t valueK, state_value_t valueV)
// {
//     State_SetValue(p_active->p_ActiveSubState, p_context, id, valueK, valueV);
// }

// static inline state_value_t _StateMachine_Branch_GetValue(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t valueK)
// {
//     return State_GetValue(p_active->p_ActiveSubState, p_context, id, valueK);
// }

/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
/* _StateMachine_Branch_Action */
extern void _StateMachine_TraverseTransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);
extern void _StateMachine_TraverseTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);

extern void _StateMachine_Branch_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_Branch_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);
extern void _StateMachine_Branch_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context);

extern void _StateMachine_Branch_ProcSyncTransition(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_Branch_ApplyAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_InvokeTraverseTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_start, State_Input_T input, state_value_t value);

extern void _StateMachine_Branch_Proc(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_Branch_Proc_Nested(StateMachine_Active_T * p_active, void * p_context);


/* Root First */
extern void _StateMachine_RootFirst_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);

extern void _StateMachine_RootFirst_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_RootFirst_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_RootOnly_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_RootFirst_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_RootFirst_ProcSyncTransition(StateMachine_Active_T * p_active, void * p_context);

extern void _StateMachine_RootFirst_ApplyAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_RootFirst_Proc(StateMachine_Active_T * p_active, void * p_context);
