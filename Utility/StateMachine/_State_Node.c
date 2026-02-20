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
    return State_IsAncestor(p_testDescendant, p_reference);
}

/* State as a Branch from root to itself. */
bool State_IsAncestorOrSelf(State_T * p_active, State_T * p_test)
{
    return ((p_active == p_test) || State_IsAncestor(p_active, p_test));
}

bool State_IsDirectLineage(State_T * p_active, State_T * p_test)
{
    return ((p_active == p_test) || State_IsAncestor(p_active, p_test) || State_IsDescendant(p_active, p_test));
}

// bool State_IsInactiveBranch(State_T * p_active, State_T * p_test)
// {
//     return State_IsDescendant(p_active, p_test);
// }

// bool State_IsCousin(State_T * p_reference, State_T * p_common, State_T * p_isCousin)
// {
//     return (State_IsAncestor(p_reference, p_common) && State_IsDescendant(p_common, p_isCousin));
// }

// bool State_IsReachableBranch(State_T * p_active, State_T * p_common, State_T * p_test)
// {
//     return ((p_active == p_test) || State_IsCousin(p_active, p_common, p_test));
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
/*!
    Build path from ancestor to descendant for efficient downward traversal
    Returns path length, or 0 if not descendant
*/
// static uint8_t State_BuildPath( State_T * p_descendant, State_T * p_ancestor,  State_T ** p_path  )
// {
//     if (p_descendant == NULL || p_ancestor == NULL) return 0;
//     if (p_descendant->DEPTH <= p_ancestor->DEPTH) return 0;

//     uint8_t pathLength = p_descendant->DEPTH - p_ancestor->DEPTH;

//     // Build path backwards
//     State_T * p_current = p_descendant;
//     for (int8_t i = pathLength - 1; i >= 0; i--)
//     {
//         p_path[i] = p_current;
//         p_current = p_current->P_PARENT;
//     }

//     // Verify ancestor relationship
//     return (p_current == p_ancestor) ? pathLength : 0;
// }
// static inline void * _BuildPath(State_T * p_state, void * p_context, State_T *** p_path) { return **p_path = p_state; *p_path += 1; }

typedef void * State_Visitor_T(State_T * p_state, void * p_context, int opt);

/*

*/
static inline void * ApplyUpTo(State_T * p_start, State_T * p_end, void * p_context, State_Visitor_T fn, int opt)
{
    void * p_result = NULL;
    for (State_T * p_iterator = p_start; p_iterator != p_end; p_iterator = p_iterator->P_PARENT)
    {
        p_result = fn(p_iterator, p_context, opt);
        if (p_result != NULL) { break; }
    }
    return p_result;
}

static inline void * ApplyUp(State_T * p_start, void * p_context, State_Visitor_T fn, int opt)
{
    return ApplyUpTo(p_start, NULL, p_context, fn, opt);
}

// end as last - 1, p_start->DEPTH != 0
// static inline void * _ApplyRange(State_T * p_start, void * p_context, uint8_t end, State_Visitor_T fn, int opt)
// {
//     void * p_result = NULL;
//     for (State_T * p_iterator = p_start; p_iterator->DEPTH > end; p_iterator = p_iterator->P_PARENT)
//     // for (State_T * p_iterator = p_start; (p_iterator != NULL) && p_iterator->DEPTH >= end; p_iterator = p_iterator->P_PARENT)
//     {
//         p_result = fn(p_iterator, p_context, opt);
//         if (p_result != NULL) { break; }
//     }
//     return p_result;
// }

/*
    Nth Ancestor
    No NULL check, as this is a private function for compile time defined values
*/
// static inline State_T * _ApplyUpN(State_T * p_start, void * p_context, int8_t iterations, State_Visitor_T fn, int opt)
// {
//     void * p_result = NULL;
//     State_T * p_iterator = p_start;
//     for (int8_t count = 0; count < iterations; count++)
//     {
//         p_result = fn(p_iterator, p_context, opt);
//         if (p_result != NULL) { break; }
//         p_iterator = p_iterator->P_PARENT;
//         assert(p_iterator != NULL); /* known at compile time */
//     }
//     return p_result;
// }
//

// static inline void * _ApplyDown_Recursive(State_T * p_start, State_T * p_end, void * p_context, State_Visitor_T fn, int opt)
// {
//     void * p_result = NULL;
//     if ((p_end != NULL) && (p_end != p_start))
//     {
//         if (p_result == NULL) { p_result = _ApplyDown_Recursive(p_start, p_end->P_PARENT, p_context, fn, opt); }
//         p_result = fn(p_end, p_context, opt);
//     }
//     return p_result;
// }




/******************************************************************************/
/*
    State Tree Functions
*/
/******************************************************************************/
/*
    Visitor Wrap
*/
static inline void * TransitionOfOutputVisitor(State_T * p_state, void * p_context, int nil) { (void)nil; return (void *)State_TransitionOfOutput(p_state, p_context); }
static inline void * AcceptInputVisitor(State_T * p_state, void * p_context, int id) { return (void *)State_AcceptInput(p_state, p_context, id); }

/******************************************************************************/
/*
    State Branch Sync/Output
*/
/******************************************************************************/
/*
    Process outputs until the first state transition.
    SubStates/InnerStates first.
    Take first transition. (RootFirst Proc to account for top level precedence)
        optionally process remaining.
    Alternatively:
        Always proc top down
        full path bottom up, take the Top/Outer most transition. Higher level determines the target State.
*/
State_T * State_TraverseTransitionOfOutput(State_T * p_start, void * p_context)
{
    return (State_T *)ApplyUp(p_start, p_context, TransitionOfOutputVisitor, 0);
}

State_T * State_TraverseTransitionOfOutputUpTo(State_T * p_start, State_T * p_end, void * p_context)
{
    return (State_T *)ApplyUpTo(p_start, p_end, p_context, TransitionOfOutputVisitor, 0);
}


/******************************************************************************/
/*
    State Branch Input
*/
/******************************************************************************/
/*
    Take the first input
    Traversal stops when input is defined, even if no state transition. alternatively
    Inputs using id always include top level
*/
State_Input_T State_TraverseAcceptInput(State_T * p_start, void * p_context, state_input_t id)
{
    return (State_Input_T)ApplyUp(p_start, p_context, AcceptInputVisitor, id);
}

/* Transition of Traverse Input */
State_T * State_TraverseTransitionOfInput(State_T * p_start, void * p_context, state_input_t id, state_value_t value)
{
    return _State_CallInput(State_TraverseAcceptInput(p_start, p_context, id), p_context, value);
}

State_Input_T State_TraverseAcceptInputUpTo(State_T * p_start, State_T * p_end, void * p_context, state_input_t id)
{
    return (State_Input_T)ApplyUpTo(p_start, p_end, p_context, AcceptInputVisitor, id);
}

State_T * State_TraverseTransitionOfInputUpTo(State_T * p_start, State_T * p_end, void * p_context, state_input_t id, state_value_t value)
{
    return _State_CallInput(State_TraverseAcceptInputUpTo(p_start, p_end, p_context, id), p_context, value);
}

/******************************************************************************/
/*
    Traverse OnTransition State_Actions
*/
/******************************************************************************/
/******************************************************************************/
/* State Branch Transition */
/******************************************************************************/
static inline void * ExitVisitor(State_T * p_state, void * p_context, int nil) { (void)nil; State_Exit(p_state, p_context); return NULL; }
static inline void * EntryVisitor(State_T * p_state, void * p_context, int nil) { (void)nil; State_Entry(p_state, p_context); return NULL; }


/*!
    Call Exit traversing up the tree
    @param[in] p_start == NULL => no effect
*/
static inline void ExitUpTo(State_T * p_start, State_T * p_common, void * p_context)
{
#ifdef STATE_MACHINE_EXIT_FUNCTION_ENABLE
    assert(p_start != NULL); /* if p_common is not NULL, p_start must not be NULL */
    ApplyUpTo(p_start, p_common, p_context, ExitVisitor, 0);
#else
    (void)p_start;
    (void)p_common;
    (void)p_context;
#endif
}

/*!
    Call Entry traversing down the tree
        recursive climb up the tree for now
        iterative traverse need capture first
    @param[in] p_common == NULL => repeat ROOT entry.
*/
static inline void EntryDownTo(State_T * p_common, State_T * p_end, void * p_context)
{
    if ((p_end != NULL) && (p_end != p_common))
    {
        EntryDownTo(p_common, p_end->P_PARENT, p_context);
        State_Entry(p_end, p_context);
    }
}

/*!
    Traverse with known CA
    Does not proc [p_common] [Entry]/[Exit]
    @param[in] p_start == NULL, start from [p_common]
    @param[in] p_common == NULL, traverse to top, alternatively pass CA depth
*/
void State_TraverseOnTransitionThrough(State_T * p_start, State_T * p_common, State_T * p_end, void * p_context)
{
    ExitUpTo(p_start, p_common, p_context);
    EntryDownTo(p_common, p_end, p_context);
}


/*!
    Traverse_OnTransition
    @param[in] p_start == NULL => p_common = NULL

    optionally build path on CA, for non recursive Entry traverse
*/
void State_TraverseOnTransition(State_T * p_start, State_T * p_end, void * p_context)
{
    State_TraverseOnTransitionThrough(p_start, State_CommonAncestorOf(p_start, p_end), p_end, p_context);
}