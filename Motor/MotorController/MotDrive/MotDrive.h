#include <stdint.h>




typedef struct MotDrive_Input
{
    int8_t Direction;
    uint16_t ThrottleValue;
    uint16_t BrakeValue;
    // uint16_t CmdValue;
    void (*Capture)(struct MotDrive_Input * p_input);
}
MotDrive_Input_T;

// typedef struct MotorController_DriveState
// {
//     // MotAnalogUser_AIn_T ThrottleAIn;

//     // AnalogValueIn
//     // bool IsEnable;
//     // uint16_t Value;
//     // uint16_t ValuePrev;

//     // MotorController_Direction_T Direction; /* Previous state */
// }
// MotorController_DriveState_T;

// static void MotorN_Fn(Motor_T * p_motors, size_t motorCount){}

// static const StateMachine_State_T STATE_DRIVE =
// {
//     .ID                 = MCSM_STATE_ID_DRIVE,
//     .ENTRY              = (StateMachine_Function_T)Drive_Entry,
//     .LOOP               = (StateMachine_Function_T)Drive_Proc,
//     .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
// };

/* update cmd using edge */
/* other 0 modes use wrapper state */
// void Motor_User_StartCmd(Motor_T * p_motor, int16_t speed_fract16)
// {
//     /* always pass to state machine 100ms-10ms */
//     // Motor_User_SetActiveCmdValue(p_motor, speed_fract16);
//     // Motor_User_ActivateControl(p_motor);
//     // StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_, speed_fract16);

//     /* invoke state machine only on edge */
//     // if      ((p_mc->UserCmdValue != 0) && (cmdValue != 0))  { Motor_User_SetActiveCmdValue(p_mc, cmdValue); }
//     // else if ((p_mc->UserCmdValue == 0) && (cmdValue != 0))  { Motor_User_ActivateControl(p_mc); } /* SetFeedbackMode, Transition */
//     // else if ((p_mc->UserCmdValue != 0) && (cmdValue == 0))  { Motor_User_Release(p_mc); }
//     // else if ((p_mc->UserCmdValue == 0) && (cmdValue == 0))  { }
// }
