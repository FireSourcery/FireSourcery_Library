// static State_T * Lock_CalibrateAdc(const MotorController_T * p_context, state_value_t lockId)
// {
//     return &MC_STATE_LOCK_CALIBRATE_ADC;
// }

// void MotorController_Lock_CalibrateAdc(const MotorController_T * p_context)
// {
//     static const StateMachine_TransitionInput_T CMD = { .P_START = &MC_STATE_LOCK, .TRANSITION = (State_Input_T)Lock_CalibrateAdc, };
//     StateMachine_Branch_InvokeTransition(&p_context->STATE_MACHINE, &CMD, 0);
// }