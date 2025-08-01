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
    @file   _State_Node.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "_State_Node.h"
#include "_State.h"

#include "State.h"


/******************************************************************************/
/*
    Generic [Parent Node Tree] Relations
*/
/******************************************************************************/
// bool State_IsAncestor(State_T * p_reference, State_T * p_testAncestor)
// {
//     return (p_testAncestor == IterateUp(p_reference, p_reference->DEPTH - p_testAncestor->DEPTH));
// }

// bool State_IsDescendant(State_T * p_reference, State_T * p_testDescendant)
// {
//     return (p_testDescendant == IterateUp(p_testDescendant, p_testDescendant->DEPTH - p_reference->DEPTH));
// }

/*
    Is [test] a Ancestor of [ref]
*/
bool State_IsAncestor(State_T * p_reference, State_T * p_testAncestor)
{
    assert(p_reference != NULL);
    assert(p_testAncestor != NULL);

    bool isAncestor = false;
    for (State_T * p_descendant = p_reference; (p_descendant->DEPTH > p_testAncestor->DEPTH); p_descendant = p_descendant->P_PARENT)
    {
        if (p_descendant->P_PARENT == p_testAncestor) { isAncestor = true; break; } /* Compare the parent. p_descendant->DEPTH returns 0 before p_descendant reaches NULL */
    }
    return isAncestor;
}

/*

*/
bool State_IsDescendant(State_T * p_reference, State_T * p_testDescendant)
{
    assert(p_reference != NULL);
    assert(p_testDescendant != NULL);

    bool isDescendant = false;
    for (State_T * p_descendant = p_testDescendant; (p_descendant->DEPTH > p_reference->DEPTH); p_descendant = p_descendant->P_PARENT)
    {
        if (p_descendant->P_PARENT == p_reference) { isDescendant = true; break; }
    }
    return isDescendant;
}

// bool State_IsCousin(State_T * p_reference, State_T * p_common, State_T * p_isCousin)
// {
//     return (State_IsAncestor(p_reference, p_common) && State_IsDescendant(p_common, p_isCousin));
// }

/* State as a Branch from root to itself. */
bool State_IsActiveBranch(State_T * p_active, State_T * p_test)
{
    return ((p_active == p_test) || State_IsAncestor(p_active, p_test));
}

bool State_IsDirectBranch(State_T * p_active, State_T * p_test)
{
    return ((p_active == p_test) || State_IsAncestor(p_active, p_test) || State_IsDescendant(p_active, p_test));
}

// bool State_IsReachableBranch(State_T * p_active, State_T * p_common, State_T * p_test)
// {
//     return ((p_active == p_test) || State_IsCousin(p_active, p_common, p_test));
// }

// bool State_IsInactiveBranch(State_T * p_active, State_T * p_test)
// {
//     return State_IsDescendant(p_active, p_test);
// }

// bool State_IsInactiveBranch(State_T * p_active, State_T * p_test)
// {
//     return State_IsDescendant(p_active, p_test);
// }

/* May include itself */
State_T * State_CommonAncestorOf(State_T * p_state1, State_T * p_state2)
{
    State_T * p_iterator1 = p_state1;
    State_T * p_iterator2 = p_state2;

    if (p_iterator1 != NULL && p_iterator2 != NULL)
    {
        // Bring both nodes to the same level
        while (p_iterator1->DEPTH > p_iterator2->DEPTH) { p_iterator1 = p_iterator1->P_PARENT; }
        while (p_iterator2->DEPTH > p_iterator1->DEPTH) { p_iterator2 = p_iterator2->P_PARENT; }

        // Traverse upwards until both nodes meet
        while (p_iterator1 != p_iterator2)
        {
            p_iterator1 = p_iterator1->P_PARENT;
            p_iterator2 = p_iterator2->P_PARENT;
        }
    }

    return p_iterator1;

    // if (p_state1 == NULL || p_state2 == NULL) { return NULL; }
    // if (p_state1 == p_state2) { return p_state1; }
    // if (p_state1->DEPTH > p_state2->DEPTH) { return State_CommonAncestorOf(p_state1->P_PARENT, p_state2); }
    // if (p_state1->DEPTH < p_state2->DEPTH) { return State_CommonAncestorOf(p_state1, p_state2->P_PARENT); }
    // return State_CommonAncestorOf(p_state1->P_PARENT, p_state2->P_PARENT);
}

/* Capture on Traverse Up, store for iterative traverse down. */
// static inline void State_CaptureTraverseUp(State_T * p_descendant, State_T ** p_buffer, uint8_t * p_count)
// {
//     for (State_T * p_iterator = p_descendant; p_iterator != NULL; p_iterator = p_iterator->P_PARENT)
//     {
//         p_buffer[(*p_count)++] = p_iterator;
//     }
// }



/******************************************************************************/
/*
    State Tree Functions
*/
/******************************************************************************/
/******************************************************************************/
/*
    Traverse
        LOOP
        NEXT
        P_TRANSITION_TABLE
*/
/******************************************************************************/
/******************************************************************************/
/* State Branch Sync/Output */
/******************************************************************************/
/*
    Proc Synchronous State Outputs.

    pass end to skip top levels
*/
/* end works on NULL and known ActiveState */
State_T * State_TraverseTransitionOfOutput(State_T * p_start, State_T * p_end, void * p_context)
{
    State_T * p_next = NULL;
    for (State_T * p_iterator = p_start; (p_iterator != NULL) && (p_iterator != p_end); p_iterator = p_iterator->P_PARENT)
    {
        p_next = State_TransitionOfOutput(p_iterator, p_context);
        if (p_next != NULL) { break; } /* substate takes precedence */
    }
    return p_next;
}

/*
    Optionally
    Take the Top most transision. Higher level determines the target State.
*/
// State_T * State_TraverseTransitionOfOutput(State_T * p_start, State_T * p_end, void * p_context)
// {
//     State_T * p_result = NULL;
//     State_T * p_dest = NULL;
//     for (State_T * p_iterator = p_start; (p_iterator != NULL) && (p_iterator != p_end); p_iterator = p_iterator->P_PARENT)
//     {
//         p_result = State_TransitionOfOutput(p_iterator, p_context);
//         if (p_result != NULL) { p_dest = p_result; } /* not break on first. let upper level overwrites. */
//     }
//     return p_dest;
// }

/******************************************************************************/
/* State Branch Input */
/******************************************************************************/
/*
    Take the first transition that accepts the input
    Inputs using id always include top level
*/
State_Input_T State_TraverseAcceptInput(State_T * p_start, void * p_context, state_input_t inputId)
{
    State_Input_T result = NULL;
    for (State_T * p_iterator = p_start; p_iterator != NULL; p_iterator = p_iterator->P_PARENT)
    {
        result = State_AcceptInput(p_iterator, p_context, inputId);
        if (result != NULL) { break; }
    }
    return result;
}

State_T * State_TraverseTransitionOfInput(State_T * p_start, void * p_context, state_input_t inputId, state_value_t inputValue)
{
    return _State_CallInput(State_TraverseAcceptInput(p_start, p_context, inputId), p_context, inputValue);
}

/******************************************************************************/
/*
exper
*/
/******************************************************************************/
// State_Input_T __State_TraverseAcceptInput(State_T * p_start, uint8_t endDepth, void * p_context, state_input_t inputId)
// {
//     State_Input_T result = NULL;
//     //skip null check when depth are compiel time in sync
//     for (State_T * p_iterator = p_start; p_iterator->DEPTH < endDepth; p_iterator = p_iterator->P_PARENT)
//     {
//         assert(p_iterator != NULL);
//         result = State_AcceptInput(p_iterator, p_context, inputId);
//         if (result != NULL) { break; }
//     }

//     return result;
// }

// static const uintptr_t State_AcceptInputOfMapper1 = (uintptr_t)State_AcceptInputOfMapper;

// const void * _State_TraverseApplyGeneric(const void * const fn, State_T * p_start, State_T * p_end, void * p_context, state_input_t inputId, state_value_t inputValue)
// {
//     State_Input_T result = NULL;
//     for (State_T * p_iterator = p_start; p_iterator != p_end; p_iterator = p_iterator->P_PARENT)
//     {
//         switch ((uintptr_t)fn)
//         {
//             // case ((uintptr_t)State_AcceptInputOfMapper1): result = State_AcceptInputOfMapper1(p_iterator, p_context, inputId); break;
//             case ((uintptr_t)State_AcceptInputOfMapper1): result = State_AcceptInputOfMapper(p_iterator, p_context, inputId); break;
//             default: result = NULL; // unsupported function
//         }
//         if (result != NULL) { break; }
//     }
//     return result;
// }



/******************************************************************************/
/*
    Traverse OnTransitionActions
*/
/******************************************************************************/
/******************************************************************************/
/* State Branch Transition */
/******************************************************************************/
/*!
    Call Exit traversing up the tree
    @param[in] p_start == NULL => no effect
*/
static inline void TraverseExit(State_T * p_start, State_T * p_common, void * p_context)
{
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    for (State_T * p_iterator = p_start; (p_iterator != NULL) && (p_iterator != p_common); p_iterator = p_iterator->P_PARENT)
    {
        if (p_iterator->EXIT != NULL) { p_iterator->EXIT(p_context); }
    }
#endif
}

/*!
    Call Entry traversing down the tree
        recursive climb up the tree for now
        iterative traverse need capture first
    @param[in] p_common == NULL => repeat ROOT entry.
*/
static inline void TraverseEntry(State_T * p_common, State_T * p_end, void * p_context)
{
    if ((p_end != NULL) && (p_end != p_common))
    {
        TraverseEntry(p_common, p_end->P_PARENT, p_context);
        State_Entry(p_end, p_context);
    }
}

/*!
    Traverse with known CA
    Does not proc [p_common] [Entry]/[Exit]
    @param[in] p_start == NULL, start from [p_common]
    @param[in] p_common == NULL, traverse to top
*/
void State_TraverseTransitionThrough(State_T * p_start, State_T * p_common, State_T * p_end, void * p_context)
{
    TraverseExit(p_start, p_common, p_context);
    TraverseEntry(p_common, p_end, p_context);
}


/*!
    Traverse_OnTransition
    @param[in] p_start == NULL => p_common = NULL
*/
void State_TraverseTransition(State_T * p_start, State_T * p_end, void * p_context)
{
    State_TraverseTransitionThrough(p_start, State_CommonAncestorOf(p_start, p_end), p_end, p_context);
}