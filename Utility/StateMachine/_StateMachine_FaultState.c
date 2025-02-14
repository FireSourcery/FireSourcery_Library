#include "StateMachine.h"


/// Fault State Template/Generic
bool StateMachine_IsFault(const StateMachine_T * p_stateMachine) { return (StateMachine_GetActiveStateId(p_stateMachine) == (p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH - 1U)); }

void StateMachine_EnterFault(StateMachine_T * p_stateMachine)
{
    if (StateMachine_IsFault(p_stateMachine) == false) { StateMachine_ProcInput(p_stateMachine, (p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH - 1U), 0U); }
}

/*! @return true if no fault remains */
bool StateMachine_ExitFault(StateMachine_T * p_stateMachine)
{
    if (StateMachine_IsFault(p_stateMachine) == true) { StateMachine_ProcInput(p_stateMachine, (p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH - 1U), 0U); }
    return StateMachine_IsFault(p_stateMachine);
}

void StateMachine_SetFault(StateMachine_T * p_stateMachine, state_machine_input_value_t faultFlags)
{
    // *(state_machine_input_value_t *)p_stateMachine->P_STATE_CONTEXT |= faultFlags;
    if (StateMachine_IsFault(p_stateMachine) == false) { StateMachine_ProcInput(p_stateMachine, (p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH - 1U), faultFlags); }
}

/*! @return true if cleared applies, fault to non fault */
bool StateMachine_ClearFault(StateMachine_T * p_stateMachine, state_machine_input_value_t faultFlags)
{
    bool isFault = StateMachine_IsFault(p_stateMachine);
    // p_stateMachine->FaultFlags &= ~faultFlags;
    if (StateMachine_IsFault(p_stateMachine) == true) { StateMachine_ProcInput(p_stateMachine, (p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH - 1U), faultFlags); }
    return (StateMachine_IsFault(p_stateMachine) != isFault);
}