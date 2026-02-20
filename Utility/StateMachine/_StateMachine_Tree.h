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
    HSM Wrap
*/
/******************************************************************************/
// static inline State_T * StateMachine_GetRootState(const StateMachine_Active_T * p_active) { return p_active->p_ActiveState; }
// static inline State_T * StateMachine_GetLeafState(const StateMachine_Active_T * p_active) { return (p_active->p_ActiveSubState == NULL) ? p_active->p_ActiveState : p_active->p_ActiveSubState; }
// refactor to single pointer
static inline State_T * StateMachine_GetRootState(const StateMachine_Active_T * p_active) { return _State_GetRoot(p_active->p_ActiveState); }
static inline State_T * StateMachine_GetLeafState(const StateMachine_Active_T * p_active) { return p_active->p_ActiveState; }

static inline state_t StateMachine_GetRootStateId(const StateMachine_Active_T * p_active) { return StateMachine_GetRootState(p_active)->ID; }
static inline state_t StateMachine_GetLeafStateId(const StateMachine_Active_T * p_active) { return StateMachine_GetLeafState(p_active)->ID; }

static inline bool StateMachine_IsRootState(const StateMachine_Active_T * p_active, State_T * p_state) { return (p_state == StateMachine_GetRootState(p_active)); }
static inline bool StateMachine_IsRootStateId(const StateMachine_Active_T * p_active, state_t stateId) { return (stateId == StateMachine_GetRootState(p_active)->ID); }

static inline bool StateMachine_IsLeafState(const StateMachine_Active_T * p_active, State_T * p_state) { return (p_state == StateMachine_GetLeafState(p_active)); }

/*

*/
/* State is within the active branch. */
static inline bool StateMachine_IsActivePath(const StateMachine_Active_T * p_active, State_T * p_state) { return State_IsAncestorOrSelf(StateMachine_GetLeafState(p_active), p_state); }

/* Ancestor or Descendant */
static inline bool StateMachine_IsDirectPath(const StateMachine_Active_T * p_active, State_T * p_state) { return State_IsDirectLineage(StateMachine_GetLeafState(p_active), p_state); }

typedef union State_PathId
{
    uint32_t BranchId;
    struct
    {
        uint32_t Depth0 : 4;
        uint32_t Depth1 : 4;
        uint32_t Depth2 : 4;
        uint32_t Depth3 : 4;
        uint32_t Depth4 : 4;
        uint32_t Depth5 : 4;
        uint32_t Depth6 : 4;
        uint32_t Depth7 : 4;
    };
}
State_PathId_T;


/******************************************************************************/
/*!
    @brief  Encodes the entire active substate path into a 32-bit ID.

    Encoding formats (selectable):
      - 4-bit fields × 8 levels: supports 16 substates per level, 8 depth max
      - 8-bit fields × 4 levels: supports 256 substates per level, 4 depth max

    Root state -> leaf state maps to LSB -> MSB.
    Unused (deeper) fields are 0.

    e.g. for path Root(2) -> Sub(3) -> SubSub(1):
      4-bit: 0x00002310  (reading MSB to LSB: ...0, 2, 3, 1, 0...)
      8-bit: 0x02030100
*/
/******************************************************************************/
static inline State_PathId_T StateMachine_GetPathId(const StateMachine_Active_T * p_active)
{
    assert(StateMachine_GetLeafState(p_active)->DEPTH < 8); /* Ensure depth fits within 4-bit fields */

    uint32_t id = 0;
    for (State_T * p_iterator = StateMachine_GetLeafState(p_active); p_iterator != NULL; p_iterator = p_iterator->P_PARENT)
    {
        id |= ((uint32_t)p_iterator->ID << (p_iterator->DEPTH * 4));
    }
    return (State_PathId_T){ .BranchId = id };
}

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
extern void _StateMachine_Branch_TransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);
extern void _StateMachine_Branch_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);

extern void _StateMachine_Branch_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_Branch_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);
extern void _StateMachine_Branch_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context);

extern void _StateMachine_Branch_ProcSyncTransition(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_Branch_ApplyAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_Branch_InvokeTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_start, State_Input_T input, state_value_t value);

extern void _StateMachine_Branch_Proc(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_Branch_Proc_Nested(StateMachine_Active_T * p_active, void * p_context);


/* Root First */
extern void _StateMachine_RootFirst_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);

extern void _StateMachine_RootFirst_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_RootFirst_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);
extern void _StateMachine_RootFirst_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_RootFirst_ProcSyncTransition(StateMachine_Active_T * p_active, void * p_context);

extern void _StateMachine_RootFirst_ApplyAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_RootFirst_Proc(StateMachine_Active_T * p_active, void * p_context);
