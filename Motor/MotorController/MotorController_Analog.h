// static State_T * Lock_CalibrateAdc(const MotorController_T * p_dev, state_value_t lockId)
// {
//     return &MC_STATE_LOCK_CALIBRATE_ADC;
// }

// void MotorController_Lock_CalibrateAdc(const MotorController_T * p_dev)
// {
//     static const StateMachine_TransitionCmd_T CMD = { .P_START = &MC_STATE_LOCK, .NEXT = (State_Input_T)Lock_CalibrateAdc, };
//     StateMachine_Tree_InvokeTransition(&p_dev->STATE_MACHINE, &CMD, 0);
// }