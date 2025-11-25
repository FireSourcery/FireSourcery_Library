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
    @file   _StateMachine.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "_State.h"
#include "_State_Node.h"
#include "_StateMachine.h"
#include "State.h"
#include "Config.h"


#define _STATE_INLINE __attribute__((always_inline))

/******************************************************************************/
/*

*/
/******************************************************************************/
/* Internal use */
/* Private Template Meta Helpers */
// typedef State_T * (*_StateMachine_InputTransitionFunction_T)(State_T * p_state, void * p_context, state_input_t inputId, state_value_t inputValue);
// typedef State_T * (*_StateMachine_OutputTransitionFunction_T)(State_T * p_state, void * p_context);
// typedef void (*_StateMachine_Transition_T)(StateMachine_Active_T * p_active, void * p_context, State_T * p_state);

typedef void (*_StateMachine_ProcInput_T)(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

void _StateMachine_Meta_ProcSyncInput(_StateMachine_ProcInput_T procInput, StateMachine_Active_T * p_active, void * p_context)
{
    // assert(__builtin_clz(p_active->SyncInputMask) < 32 STATE_TRANSITION_TABLE_LENGTH_MAX);
    for (uint32_t inputMask = p_active->SyncInputMask; inputMask != 0UL; inputMask &= (inputMask - 1))
    {
        state_input_t input = __builtin_ctz(inputMask);
        // assert(input < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */
        procInput(p_active, p_context, input, p_active->SyncInputs[input]); /* may update [p_ActiveState] */
    }
    p_active->SyncInputMask = 0UL;
}

// static inline void __StateMachine_ProcSyncInput(
//     StateMachine_Active_T * p_active,
//     State_T * p_state,
//     void * p_context,
//     _StateMachine_TransitionFunction_T transitionFunction,
//     _StateMachine_Transition_T transition
// )
// {
//     // assert(input < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */
//     for (uint32_t inputMask = p_syncInputs->IdsMask; inputMask != 0UL; inputMask &= (inputMask - 1))
//     {
//         state_input_t input = __builtin_ctz(inputMask);
//         transition(p_active, p_context, transitionFunction(p_state, p_context, input, p_syncInputs->Values[input])); /* may update [p_ActiveState] */
//     }
//     p_syncInputs->IdsMask = 0UL;
// }



// static inline void * __StateMachine_TraverseApplyAs(
//     StateMachine_Active_T * p_active,
//     State_T * p_state,
//     void * p_context,
// )
