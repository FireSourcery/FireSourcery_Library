/******************************************************************************/
/*
    Linked Menus
*/
/******************************************************************************/
#ifdef CONFIG_STATE_MACHINE_LINKED_STATES_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, state_input_t input, state_input_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, inputValue)
}

State_T * StateMachine_Menu_GetPtrActive(StateMachine_T * p_stateMachine)
{
    return p_stateMachine->p_ActiveState;
}

void StateMachine_Menu_SetMenu(StateMachine_T * p_stateMachine, State_T * p_targetMenu)
{
    p_stateMachine->p_ActiveState = p_targetMenu;
}

void StateMachine_Menu_StartMenu(StateMachine_T * p_stateMachine, State_T * p_targetMenu)
{
    _StateMachine_Transition(p_stateMachine, p_targetMenu);
}

// does not run entry function
void StateMachine_Menu_SetNext(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_ActiveState->P_NEXT_MENU != NULL) { StateMachine_Menu_SetMenu(p_stateMachine, p_stateMachine->p_ActiveState->P_NEXT_MENU); }
}

// run entry function
void StateMachine_Menu_StartNext(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_ActiveState->P_NEXT_MENU != NULL) { StateMachine_Menu_StartMenu(p_stateMachine, p_stateMachine->p_ActiveState->P_NEXT_MENU); }
}

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, state_input_t input, state_input_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, inputValue)
}

void StateMachine_Menu_ProcLoop(StateMachine_T * p_stateMachine)
{
    StateMachine_ProcState(p_stateMachine);
}
#endif