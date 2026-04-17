#include "StateMachine.h"

/* Fault State Template/Generic */

#ifdef STATE_MACHINE_FAULT_STATE_ENABLE
    #define _STATE_MACHINE_FAULT(code, ...) code
#else
    #define _STATE_MACHINE_FAULT(code, ...) (void)(__VA_ARGS__)
#endif

State_T * FaultState(const StateMachine_T * p_stateMachine)
{
#ifdef STATE_MACHINE_FAULT_STATE_ENABLE
    return  p_stateMachine->P_MACHINE->P_STATE_FAULT;
#else
    return NULL;
#endif
}

void FaultState_SetFlags(const StateMachine_T * p_stateMachine, state_value_t faultFlags)
{
#ifdef STATE_MACHINE_FAULT_STATE_ENABLE
    p_stateMachine->P_ACTIVE->FaultFlags = faultFlags;
#endif
}

state_value_t FaultState_GetFlags(const StateMachine_T * p_stateMachine)
{
#ifdef STATE_MACHINE_FAULT_STATE_ENABLE
    return  p_stateMachine->P_ACTIVE->FaultFlags;
#else
    return 0;
#endif
}


state_value_t FaultState_FaultId(const StateMachine_T * p_stateMachine)
{
#ifdef STATE_MACHINE_FAULT_INPUT_ID
    return STATE_MACHINE_FAULT_INPUT_ID;
#else
    return (p_stateMachine->P_MACHINE->TRANSITION_TABLE_LENGTH - 1U);
#endif
}

// static State_T * FaultState_TransitionFault(const StateMachine_T * p_stateMachine, state_value_t faultCmd)
// {
//     StateMachine_FaultCmd_T cmd = { .Value = faultCmd };
//     p_stateMachine->P_ACTIVE->FaultFlags.Value |= cmd.FaultSet;
//     return (p_stateMachine->P_ACTIVE->FaultFlags.Value != 0U) ? FaultState(p_stateMachine) : NULL;
// }

/*
    Fault Interface Functions
*/
bool StateMachine_IsFault(const StateMachine_T * p_stateMachine) { return StateMachine_IsActiveState(p_stateMachine->P_ACTIVE, FaultState(p_stateMachine)); }

typedef union StateMachine_FaultCmd
{
    struct { uint16_t FaultSet; uint16_t FaultClear; };
    uint32_t Value;
}
StateMachine_FaultCmd_T;

/* Pass only delta flags — handler applies OR FaultSet / AND-NOT FaultClear atomically */
void  StateMachine_SetFault(const StateMachine_T * p_stateMachine, state_value_t faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_stateMachine, FaultState_FaultId(p_stateMachine), (StateMachine_FaultCmd_T) { .FaultSet = faultFlags }.Value);
    // if (StateMachine_IsFault(p_stateMachine) == false) { StateMachine_ForceTransition(p_stateMachine, FaultState(p_stateMachine)); }
}

void  StateMachine_ClearFault(const StateMachine_T * p_stateMachine, state_value_t faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_stateMachine, FaultState_FaultId(p_stateMachine), (StateMachine_FaultCmd_T) { .FaultClear = faultFlags }.Value);
    // if (StateMachine_IsFault(p_stateMachine) == true) { StateMachine_ForceTransition(p_stateMachine, p_stateMachine->P_MACHINE->P_STATE_INITIAL); }
}

/*! @return true if no fault remains */
bool  StateMachine_ExitFault(const StateMachine_T * p_stateMachine)
{
    StateMachine_ClearFault(p_stateMachine, UINT16_MAX);
    return !StateMachine_IsFault(p_stateMachine);
}

void StateMachine_EnterFault(StateMachine_T * p_stateMachine)
{
    StateMachine_SetFault(p_stateMachine, FaultState_GetFlags(p_stateMachine));
}

