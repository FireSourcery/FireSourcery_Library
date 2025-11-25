#include "StateMachine.h"


/// Fault State Template/Generic
bool StateMachine_IsFault(const StateMachine_T * p_stateMachine) { return StateMachine_IsActiveState(p_stateMachine->P_ACTIVE, p_stateMachine->P_MACHINE->P_STATE_FAULT); }

void StateMachine_EnterFault(StateMachine_T * p_stateMachine)
{
    // if (StateMachine_IsFault(p_stateMachine) == false) { StateMachine_ApplyInput(p_stateMachine, (p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH - 1U), 0U); }

    if (StateMachine_IsFault(p_stateMachine) == false) { StateMachine_ForceTransition(p_stateMachine, p_stateMachine->P_MACHINE->P_STATE_FAULT); }
}

/*! @return true if no fault remains */
bool StateMachine_ExitFault(StateMachine_T * p_stateMachine)
{
    if (StateMachine_IsFault(p_stateMachine) == true) { StateMachine_ForceTransition(p_stateMachine, p_stateMachine->P_MACHINE->P_STATE_INITIAL); }
    return !StateMachine_IsFault(p_stateMachine);
}

void StateMachine_SetFault(StateMachine_T * p_stateMachine, state_value_t faultFlags)
{
    p_stateMachine->P_ACTIVE->FaultFlags |= faultFlags;
    // if (StateMachine_IsFault(p_stateMachine) == false) { StateMachine_ApplyInput(p_stateMachine, (p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH - 1U), faultFlags); }
}

/*! @return true if cleared applies, fault to non fault */
void StateMachine_ClearFault(StateMachine_T * p_stateMachine, state_value_t faultFlags)
{
    // bool isFault = StateMachine_IsFault(p_stateMachine);
    p_stateMachine->P_ACTIVE->FaultFlags &= ~faultFlags;
    // if (StateMachine_IsFault(p_stateMachine) == true) { StateMachine_ApplyInput(p_stateMachine, , faultFlags); }
    // return (StateMachine_IsFault(p_stateMachine) != isFault);
}