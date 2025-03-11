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


// static void MotorN_Fn(Motor_T * p_motors, size_t motorCount){}

// static const StateMachine_State_T STATE_DRIVE =
// {
//     .ID                 = MCSM_STATE_ID_DRIVE,
//     .ENTRY              = (StateMachine_Function_T)Drive_Entry,
//     .LOOP               = (StateMachine_Function_T)Drive_Proc,
//     .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
// };