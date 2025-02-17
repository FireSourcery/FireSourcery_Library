/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   StateMachine.c
    @author FireSourcery
    @brief     StateMachine
    @version V0
*/
/******************************************************************************/
#include "StateMachine.h"

/******************************************************************************/
/*!
    Private Functions
*/
/******************************************************************************/
/*!
*/
static inline bool AcquireSignal_ISR(StateMachine_T * p_stateMachine)
{
    return Critical_AcquireSignal(&p_stateMachine->InputSignal); /* Checks if an Input has the signal  */
}

static inline void ReleaseSignal_ISR(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine; /* Single threaded input always overwrite */
#endif
}

static inline bool AcquireSignal_Input(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    /* Multi-threaded do not proceed, if another thread holds the signal */
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#else
    /* Single threaded input always overwrite. ISR will run to completion. does not need to block Input */
    Critical_AcquireSignal(&p_stateMachine->InputSignal);
    return true;
#endif
}

static inline void ReleaseSignal_Input(StateMachine_T * p_stateMachine)
{
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
}

/*
    Selection between DisableISR and Signal
    Multi-threaded may use disable interrupts, set_and_test signal, or spin-wait with thread scheduler
*/
static inline bool AcquireCritical_ISR(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    _Critical_DisableIrq();
    #endif
    return true;
#else
    return AcquireSignal_ISR(p_stateMachine);
#endif
}

static inline void ReleaseCritical_ISR(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    _Critical_EnableIrq();
    #endif
#else
    ReleaseSignal_ISR(p_stateMachine);
#endif
}

static inline bool AcquireCritical_Input(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    _Critical_DisableIrq();
    return true;
#else
    return AcquireSignal_Input(p_stateMachine);
#endif
}

static inline void ReleaseCritical_Input(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    _Critical_EnableIrq();
#else
    ReleaseSignal_Input(p_stateMachine);
#endif
}


/******************************************************************************/
/*! */
/******************************************************************************/
static inline void State_Init(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_newState, void * p_context)
{
    if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_context); }
    *pp_currentState = p_newState;
}

/* Sets null as null */
static inline void State_ProcTransition(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_newState, void * p_context)
{
    if (p_newState != NULL)
    {
    #ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
        if ((*pp_currentState)->EXIT != NULL) { (*pp_currentState)->EXIT(p_context); }
    #endif
        if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_context); }
        *pp_currentState = p_newState;
    }
}

/*
    Top level State Proc
*/
static inline void State_Proc(const StateMachine_State_T ** pp_currentState, void * p_context)
{
    (*pp_currentState)->LOOP(p_context);
    if ((*pp_currentState)->NEXT != NULL) { State_ProcTransition(pp_currentState, (*pp_currentState)->NEXT(p_context), p_context); }
}

/*
    Recursive for SubState
*/
static inline void State_ProcTraversal(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_superState, void * p_context)
{
    if (p_superState != NULL)
    {
        // p_superState->LOOP(p_context);
        if (p_superState->LOOP != NULL) { p_superState->LOOP(p_context); }
        if (p_superState->NEXT != NULL) { State_ProcTransition(pp_currentState, p_superState->NEXT(p_context), p_context); }
        State_ProcTraversal(pp_currentState, p_superState->P_HOST, p_context);
    }
}


/*
    Top Level p_stateMachine->p_StateActive
*/
static inline StateMachine_Input_T AcceptInput(const StateMachine_T * p_stateMachine, state_machine_input_id_t inputId)
{
    // assert(inputId < p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH);
    // return p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId];
    return (inputId < p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH) ? p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId] : NULL;
}

static inline StateMachine_State_T * _TransitionFunction(StateMachine_T * p_stateMachine, StateMachine_Input_T transition, state_machine_input_value_t inputValue)
{
    return (transition != NULL) ? transition(p_stateMachine->CONST.P_CONTEXT, inputValue) : NULL;
}

/*!
    Transistion Function maps current state to new state for each input

    @return NULL for self-transition without processing ENTRY,
            self for self-transition processing ENTRY
            another State for transition to another State
*/
/* Map (inputId, inputValue) => newState */
static inline StateMachine_State_T * TransitionFunction(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    // StateMachine_Input_T transition = AcceptInput(p_stateMachine, inputId);
    // return (transition != NULL) ? transition(p_stateMachine->CONST.P_CONTEXT, inputValue) : NULL;
    return _TransitionFunction(p_stateMachine, AcceptInput(p_stateMachine, inputId), inputValue);
}


// static inline void SetActiveState(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
// {
//     State_ProcTransition(&p_stateMachine->p_StateActive, p_newState, p_stateMachine->CONST.P_CONTEXT);
// }

/*
    ProcInput must lock
    A transition must not occur between [StateMachine_IsAcceptInput] and [ProcTransition]

    Store a local copy first. if a transition does occur, prevent NULL pointer, the previously selected transition will run.
*/
static void ProcInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    // SetActiveState(p_stateMachine, TransitionFunction(p_stateMachine, inputId, inputValue));
    State_ProcTransition(&p_stateMachine->p_StateActive, TransitionFunction(p_stateMachine, inputId, inputValue), p_stateMachine->CONST.P_CONTEXT);
}

/*
    No null pointer check. User ensure LOOP is defined when using this interface. supply empty for no op
*/
static inline void ProcStateOuput(StateMachine_T * p_stateMachine)
{
    // p_stateMachine->p_StateActive->LOOP(p_stateMachine->CONST.P_CONTEXT);
    // SetActiveState(p_stateMachine, p_stateMachine->p_StateActive->NEXT(p_stateMachine->CONST.P_CONTEXT));
    State_Proc(&p_stateMachine->p_StateActive, p_stateMachine->CONST.P_CONTEXT);
}


/* Init */
static inline void Reset(StateMachine_T * p_stateMachine)
{
    p_stateMachine->p_StateActive = p_stateMachine->CONST.P_MACHINE->P_STATE_INITIAL;
    if (p_stateMachine->p_StateActive->ENTRY != NULL) { p_stateMachine->p_StateActive->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
}

/******************************************************************************/
/*!
    Protected Functions
*/
/******************************************************************************/

/*
    Unconditional Transition - Maps active state to new state.
        p_newState assumed to be valid, caller ensure correctness
        call from within [ProcState] high priority thread, or user handle critical
*/
inline void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
{
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    if (p_stateMachine->p_StateActive->EXIT != NULL) { p_stateMachine->p_StateActive->EXIT(p_stateMachine->CONST.P_CONTEXT); }
#endif
    if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
    p_stateMachine->p_StateActive = p_newState;
    /*
        Async may selectively implement critical. If unprotected:
        Set p_StateActive after proc ENTRY.
            This way Async will not proc [p_newState->OUTPUT] until after [p_newState->ENTRY].
            May proc [p_StateActive->OUTPUT] (the prev State) after ENTRY.
            p_newState->OUTPUT after p_newState->ENTRY correctly
            p_StateActive->OUTPUT may overwrite p_newState->ENTRY incorrectly

        Alternatively:
        Set p_StateActive before ENTRY.
            This way Async might proc OUTPUT of p_newState before ENTRY.
            p_newState->ENTRY overwrite p_StateActive->OUTPUT correctly
            p_newState->OUTPUT prior to p_newState->ENTRY incorrectly
    */
}

// void StateMachine_ProcStateTransitionId(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId)

/*
    User ensure same thread
    Selectively implement critical in calling layer, if not require for all inputs
*/
inline void _StateMachine_ProcStateOutput(StateMachine_T * p_stateMachine)
{
    ProcStateOuput(p_stateMachine);
}

/*
    it is possible to return a boolean status indicating if the input was accepted,
    however this status is only dependent on the active State, and not the logic of the State.
    the calling layer only needs to check the active state.
*/
inline void _StateMachine_ProcAsyncInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    ProcInput(p_stateMachine, inputId, inputValue);
}

inline void _StateMachine_ProcSyncInput(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->SyncInput != STATE_MACHINE_INPUT_ID_NULL) /* checked by isAccept input */
    {
        ProcInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
        p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL;
    }
}

inline void _StateMachine_SetSyncInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    p_stateMachine->SyncInputValue = inputValue;
    p_stateMachine->SyncInput = inputId;
}


/******************************************************************************/
/*!
    Public Functions
*/
/******************************************************************************/
/*
    States const strut should be compile time def
*/
void StateMachine_Init(StateMachine_T * p_stateMachine)
{
    _StateMachine_SetSyncInput(p_stateMachine, STATE_MACHINE_INPUT_ID_NULL, 0);
    // Sync_InitSignal(p_stateMachine);
    atomic_flag_clear(&p_stateMachine->InputSignal); /* Special case single threaded, ISR will run null once */
    Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    if (AcquireCritical_Input(p_stateMachine) == true)
    {
        Reset(p_stateMachine);
        ReleaseCritical_Input(p_stateMachine);
    }
}

/******************************************************************************/
/*
    [Synchronous Machine]
    Synchronous Proc State - [ProcState] synchronous to clock/timer
    Synchronous Proc Input - Async user [SetInput] proc synchronously during [ProcState]

    Proc last set input, always single threaded proc, inputs may overwrite

    Does not need EnterCritical/DisableIRQ, given:
        [ProcState] thread is higher priority than [SetInput] thread
        [SetInput] calls are on the same thread
        Check accept on [ProcState]

    Multiple [SetInput] calls in between [ProcState] needs to be handled by synchronization mechanism.
        [ProcState] may occur during [SetInput]
        [SyncInput] will not have cleared when a new [SyncInputValue] is set
    E.g.
        p_stateMachine->SyncInputValue = inputValue;
        p_stateMachine->SyncInput = inputId;
        p_stateMachine->SyncInputValue = inputValue;
        StateMachine_Sync_ProcState
        p_stateMachine->SyncInput = inputId;

    Atomic signal skip [ProcState], or spin-wait in low priority input thread

*/
/******************************************************************************/
void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine)
{
    if (AcquireSignal_ISR(p_stateMachine) == true)
    {
        /*
            Multi-threaded calls to [Sync_SetInput] use additional sentinel value
            [InputSignal] flag is cleared to provide indication to other inputs

            Single threaded calls can use atomic flag as bufferEmpty, bufferHasData when cleared
            [InputSignal] flag does not need to clear, as it is always overwritten by input
            [SyncInput] id does not need to clear
        */
    #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
        _StateMachine_ProcSyncInput(p_stateMachine);
    #else
        /* Singled threaded case handled by signal flag */
        _StateMachine_ProcAsyncInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
    #endif
        ReleaseSignal_ISR(p_stateMachine);
    }
    else
    {
        /* [ProcStateOutput] without Signal/DisableISR is okay, when all Inputs are [Sync_SetInput] */
        _StateMachine_ProcStateOutput(p_stateMachine);
    }
}

/*!
    Multiple calls overwrite the previous, only the last set is processed.

    [StateMachine_IsAcceptInput] may provide an immediate return,
        although may differ when checked again during [ProcState], due to transition or overwrite
*/
void StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    /* Disables [ProcInput] portion of [Sync_ProcState]. */
    if (AcquireSignal_Input(p_stateMachine) == true)
    {
        _StateMachine_SetSyncInput(p_stateMachine, inputId, inputValue);
        ReleaseSignal_Input(p_stateMachine);
    }
}

// void StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)

/******************************************************************************/
/*
    [Asynchronous Machine]
    Async user input may change state immediately,
        optionally implement periodic [ProcStateOuput]

    Input/State on the same thread: same behavior as synchronous

    Input/State on different threads:
        Critical on input, Disable ProcState when input is processing

*/
/******************************************************************************/
/*
    Assuming [ProcState] runs higher priority than [ProcInput], critical is implemented in [ProcInput] only
*/
/* when input can be higher priority this must block or not transition */
void StateMachine_Async_ProcState(StateMachine_T * p_stateMachine)
{
    if (AcquireCritical_ISR(p_stateMachine) == true) /* Disabled when input is processing, ensure any transition is completed */
    {
        _StateMachine_ProcStateOutput(p_stateMachine);
        ReleaseCritical_ISR(p_stateMachine);
    }
}

/*
    [AcquireCritical] blocks [ProcState] and other [ProcInput]

    Signal Case
        may skip the next [ProcState]
        inputs must be on the same thread, or a non polling input maybe missed
*/
void StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    if (AcquireCritical_Input(p_stateMachine) == true)
    {
        _StateMachine_ProcAsyncInput(p_stateMachine, inputId, inputValue);
        ReleaseCritical_Input(p_stateMachine);
    }
}

/******************************************************************************/
/*
    [Combined Semi-Synchronous Machine]
    supports both operation at expense of additional condition check
*/
/******************************************************************************/
inline void StateMachine_ProcState(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL) /* if Async is always disable critical */
    /* [Async_ProcInput] disables ISR, [ProcStateOutput] is ok  */
    StateMachine_Sync_ProcState(p_stateMachine);
#else
    /* [ProcSyncInput] must also be blocked during [Async_ProcInput] in case of transition */
    if (AcquireSignal_ISR(p_stateMachine) == true)
    {
        _StateMachine_ProcSyncInput(p_stateMachine);     /* Proc input must use sentinel */
        _StateMachine_ProcStateOutput(p_stateMachine);
        ReleaseSignal_ISR(p_stateMachine);
    }
#endif
}

inline void StateMachine_SetInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    StateMachine_Sync_SetInput(p_stateMachine, inputId, inputValue);
}

inline void StateMachine_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue)
{
    StateMachine_Async_ProcInput(p_stateMachine, inputId, inputValue);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/* per state inputs, only need to check id. inputs that only map to 1 state, reduce table size */
// inline void StateMachine_SetValue(StateMachine_T * p_stateMachine, state_machine_state_t state, state_machine_input_value_t value)
// {
//     if (p_stateMachine->p_StateActive->ID == state) {
//         if (p_stateMachine->p_StateActive->SET != NULL) { p_stateMachine->p_StateActive->SET(p_stateMachine->CONST.P_CONTEXT, value); }
//     }
// }

// inline void StateMachine_substateInput(StateMachine_T * p_stateMachine, substateid, state_machine_input_value_t value)
// {
//  p_subState->P_TRANSITION_TABLE[ ](p_stateMachine->CONST.P_CONTEXT, value);
// }



/*
    SubState Functions
*/

/*  */
void _StateMachine_ProcBranch(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_SubState != NULL)
    {
        State_ProcTraversal(&p_stateMachine->p_SubState, p_stateMachine->p_SubState, p_stateMachine->CONST.P_CONTEXT);
    }
}

/* SubState Only */
/* Proc already inside Signal */
void _StateMachine_ProcSubState(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_SubState != NULL)
    {
        if (p_stateMachine->p_SubState->LOOP != NULL)
        {
            /* caller must call _StateMachine_EndSubState to end loop or use return code */
            State_Proc(&p_stateMachine->p_SubState, p_stateMachine->CONST.P_CONTEXT);
        }
        // else
        // {
        //     _StateMachine_EndSubState(p_stateMachine);
        // }
    }
}

inline void _StateMachine_EndSubState(StateMachine_T * p_stateMachine)
{
    // p_stateMachine->p_SubState = p_stateMachine->p_SubState->NEXT(p_stateMachine->CONST.P_CONTEXT);
    p_stateMachine->p_SubState = NULL;
}


/*
    Allow setting SubState without defined transitions.
*/

bool State_IsActive(const StateMachine_State_T * p_currentHost, const StateMachine_State_T * p_subState)
{
    if (p_currentHost == p_subState) { return true; }
    else { return State_IsActive(p_currentHost, p_subState->P_HOST); }

    // while(p_subState->P_HOST != NULL) {
    //     p_host = p_subState->P_HOST;
            // if (StateMachine_IsActiveState(p_stateMachine, p_subState->P_HOST) == true)
    // } /* get the top level state */
}

// set the substate if it parent exist in the branch
// inline void _StateMachine_SetSubState(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_subState)
// {
//     if (StateMachine_IsActiveState(p_stateMachine->p_SubState->P_HOST, p_subState->P_HOST) == true)
//     {
//         /* multilevel implement array and level */
//         State_Init(&p_stateMachine->p_SubState, p_subState, p_stateMachine->CONST.P_CONTEXT);
//     }
// }


/*
    Allow any SubState to overwrite any SubState
*/
void StateMachine_SetSubState(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_subState)
{
    if (AcquireSignal_Input(p_stateMachine) == true) /* Disable ProcState or entry maybe overwritten */
    {
        if (StateMachine_IsActiveState(p_stateMachine, p_subState->P_HOST) == true)
        {
            /* multilevel implement array and level */
            State_Init(&p_stateMachine->p_SubState, p_subState, p_stateMachine->CONST.P_CONTEXT);
        }
        ReleaseSignal_Input(p_stateMachine);
    }
}

// void StateMachine_ProcSubStateInput(StateMachine_T * p_stateMachine, state_machine_input_id_t id, state_machine_input_value_t value)
// {
//     p_subState->P_TRANSITION_TABLE[id](p_stateMachine->CONST.P_CONTEXT, value);
// }

/* additionally match to substate */
// void StateMachine_ProcSubStateInput(StateMachine_T * p_stateMachine, StateMachine_State_T * p_subState, state_machine_input_id_t id, state_machine_input_value_t value)
// {
//     p_subState->P_TRANSITION_TABLE[id](p_stateMachine->CONST.P_CONTEXT, value);
//
// }

/*
    Unique per State Inputs
*/

static inline bool IsCmdState(const StateMachine_T * p_stateMachine, const StateMachine_State_T * p_state)
{
    return ((p_state == p_stateMachine->p_StateActive) || (p_state->P_HOST == p_stateMachine->p_StateActive));
}

static inline bool IsCmdStateId(const StateMachine_T * p_stateMachine, const state_machine_state_t stateId)
{
    return true;
}

// void StateMachine_StartCmd(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_initialState, const StateMachine_Cmd_T * p_cmd, state_machine_input_value_t inputValue)
// {

// }

void StateMachine_StartCmd(StateMachine_T * p_stateMachine, const StateMachine_Cmd_T * p_cmd, state_machine_input_value_t inputValue)
{
    if (AcquireSignal_Input(p_stateMachine) == true) /* Disable Proc or results of CMD maybe overwritten */
    {
        // StateMachine_ProcInput(p_stateMachine, p_cmd->P_INITIAL->P_HOST, 0); optionally attempt to transition first
        if (IsCmdState(p_stateMachine, p_cmd->P_INITIAL))
        {
            if (p_cmd->CMD != NULL) { p_cmd->CMD(p_stateMachine->CONST.P_CONTEXT, inputValue); }

            // _StateMachine_SetSubState
            if (p_cmd->P_INITIAL == p_stateMachine->p_StateActive) { _StateMachine_EndSubState(p_stateMachine); } /* pointer to the host state, indicate no periodic loop */
            else { State_Init(&p_stateMachine->p_SubState, p_cmd->P_INITIAL, p_stateMachine->CONST.P_CONTEXT); }
        }
        ReleaseSignal_Input(p_stateMachine);
    }

    // p_stateMachine->CmdChain = cmdChain;
    // cmdChain(p_stateMachine->CONST.P_CONTEXT, NULL)->CMD(p_stateMachine->CONST.P_CONTEXT, inputValue); /* call the initial command  */
}




/******************************************************************************/
/*
    Linked Menus
*/
/******************************************************************************/
#ifdef CONFIG_STATE_MACHINE_LINKED_MENU_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_id_t input, state_machine_input_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, inputValue)
}

StateMachine_State_T * StateMachine_Menu_GetPtrActive(StateMachine_T * p_stateMachine)
{
    return p_stateMachine->p_StateActive;
}

void StateMachine_Menu_SetMenu(StateMachine_T * p_stateMachine, StateMachine_State_T * p_targetMenu)
{
    p_stateMachine->p_StateActive = p_targetMenu;
}

void StateMachine_Menu_StartMenu(StateMachine_T * p_stateMachine, StateMachine_State_T * p_targetMenu)
{
    _StateMachine_ProcStateTransition(p_stateMachine, p_targetMenu);
}

// does not run entry function
void StateMachine_Menu_SetNext(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_StateActive->P_NEXT_MENU != NULL) { StateMachine_Menu_SetMenu(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU); }
}

// run entry function
void StateMachine_Menu_StartNext(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_StateActive->P_NEXT_MENU != NULL) { StateMachine_Menu_StartMenu(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU); }
}

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, state_machine_input_id_t input, state_machine_input_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, inputValue)
}

void StateMachine_Menu_ProcLoop(StateMachine_T * p_stateMachine)
{
    StateMachine_ProcState(p_stateMachine);
}
#endif
