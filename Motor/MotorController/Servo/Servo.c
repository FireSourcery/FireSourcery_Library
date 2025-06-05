#include "Servo.h"

// #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
// static const State_T STATE_SERVO;
// #endif
/******************************************************************************/
/*!
    @brief Servo State
    User State, Motor_User interface
*/
/******************************************************************************/
/*
    Handle per motor functions
    multi parameter input into StateMachine. No transition
*/
/*! @param[in] cmdValue int16  */
// void MotorController_ServoState_InputCmd(MotorController_T * p_mc, uint8_t motorId, int16_t cmdValue)
// {
//     if (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_SERVO)
//     {
//     #if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
//         MotorController_ServoExtern_SetMotorCmd(p_mc, motorId, cmdValue);
//     #else
//         Motor_User_SetActiveCmdValue(MotorController_MotorAt(p_mc, motorId), cmdValue);
//     #endif
//     }
// }

// #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static void Servo_Entry(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Start(p_mc);
#else
    // MotorController_Servo_Start(p_mc);
    // MotorController_StartCmdMode(p_mc, p_mc->Config.DefaultCmdMode);
    // MotorController_ActivateAll(p_mc);
#endif
}

static void Servo_Proc(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Proc(p_mc);
#else
    // MotorController_Servo_Proc(p_mc);
#endif
}

static State_T * Servo_InputExit(MotorController_T * p_mc, state_input_value_t _void)
{
    (void)_void;
    MotMotors_ForEach(&p_mc->CONST.MOTORS, Motor_User_Release);
    return &STATE_PARK;
}

/*! @param[in] cmdValue int16  */
static State_T * Servo_InputCmd(MotorController_T * p_mc, state_input_value_t cmdValue)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_SetCmd(p_mc, cmdValue);
#else
    // MotorController_Servo_SetCmd(p_mc, userCmd);
    MotMotors_SetCmdValue(&p_mc->CONST.MOTORS, cmdValue);
#endif

    return NULL;
}

// static State_T * Servo_InputDirection(MotorController_T * p_mc, state_input_value_t direction)
// {
//     State_T * p_nextState = NULL;
//     switch((MotDrive_Direction_T)direction)
//     {
//         case MOT_DRIVE_DIRECTION_PARK:       p_nextState = MotorController_IsEveryMotorStopState(p_mc) ? &STATE_PARK : NULL; break;
//         case MOT_DRIVE_DIRECTION_NEUTRAL:    p_nextState = &STATE_NEUTRAL; break;
//         case MOT_DRIVE_DIRECTION_FORWARD:    p_nextState = MotorController_SetDirectionForwardAll(p_mc) ? &STATE_DRIVE : NULL; break;
//         case MOT_DRIVE_DIRECTION_REVERSE:    p_nextState = MotorController_SetDirectionReverseAll(p_mc) ? &STATE_DRIVE : NULL; break;
//         default: break;
//     }
//     return p_nextState;
// }

static State_T * Servo_InputServo(MotorController_T * p_mc, state_input_value_t servoMode)
{
    State_T * p_nextState = NULL;
    switch ((MotorController_ServoMode_T)servoMode)
    {
        case MOTOR_CONTROLLER_SERVO_MODE_EXIT:
            if (MotMotors_IsEveryValue(&p_mc->CONST.MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) == true) { p_nextState = &STATE_PARK; }
            break;
        default: break;
    }
    return p_nextState;
}

static const State_Input_T SERVO_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    // [MOT_DRIVE_STATE_INPUT_DIRECTION]   = (State_Input_T)Servo_InputDirection,
    [MOT_DRIVE_STATE_INPUT_CMD]            = (State_Input_T)Servo_InputCmd,
    [MCSM_INPUT_SERVO]          = (State_Input_T)Servo_InputServo,
};

static const State_T STATE_SERVO =
{
    .ID                 = MCSM_STATE_ID_SERVO,
    .ENTRY              = (State_Action_T)Servo_Entry,
    .LOOP               = (State_Action_T)Servo_Proc,
    .P_TRANSITION_TABLE = &SERVO_TRANSITION_TABLE[0U],
};
// #endif