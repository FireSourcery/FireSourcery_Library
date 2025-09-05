
/******************************************************************************/
/*!
    @brief Drive Set Common
    if motor proc transition on call, let container state can depend on motor state
*/
/******************************************************************************/
/* Motors to PASSIVE, keep direction */
// static State_T * Common_InputNeutral(const MotDrive_T * p_motDrive)
// {
//     return &STATE_NEUTRAL; /* apply v float on entry */
// }

// // Motor_User_Direction_T Direction == MOTOR_DIRECTION_FORWARD
// // Phase_Output_T PhaseState = PHASE_OUTPUT_VPWM/PHASE_OUTPUT_FLOAT;
// static State_T * Common_InputForward(const MotDrive_T * p_motDrive)
// {
//     MotMotors_ApplyUserDirection(&p_motDrive->MOTORS, MOTOR_DIRECTION_FORWARD);
//     return (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) == true) ? &STATE_DRIVE : NULL;
//     return &STATE_DRIVE;
// }

// static State_T * Common_InputReverse(const MotDrive_T * p_motDrive)
// {
//     MotMotors_ApplyUserDirection(&p_motDrive->MOTORS, MOTOR_DIRECTION_REVERSE);
//     return (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionReverse) == true) ? &STATE_DRIVE : NULL;
//     return &STATE_DRIVE;
// }

// if motor proc transition on call
// static State_T * Common_InputDirection(const MotDrive_T * p_motDrive, state_value_t direction)
// {
//     State_T * p_nextState = NULL;

//     switch ((MotDrive_Direction_T)direction)
//     {
//         case MOT_DRIVE_DIRECTION_PARK:    p_nextState = Common_InputPark(p_motDrive); break;
//         case MOT_DRIVE_DIRECTION_FORWARD: p_nextState = Common_InputForward(p_motDrive); break;
//         case MOTOR_DIRECTION_REVERSE: p_nextState = Common_InputReverse(p_motDrive); break;
//         case MOT_DRIVE_DIRECTION_NEUTRAL: p_nextState = Common_InputNeutral(p_motDrive); break;
//         case MOT_DRIVE_DIRECTION_ERROR: p_nextState = NULL; break;
//         default: break;
//     }

//     return p_nextState;
// }


// with hsm as orthogonal region
// static State_T * Drive_SyncTransition(const MotDrive_T * p_motDrive)
// {
//     State_T * p_nextState = NULL;
//
//     if (p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd != p_motDrive->P_MOT_DRIVE_STATE->Input.CmdPrev)
//     {
//         /* handle edge */
//         switch (p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd)
//         {
//             case MOT_DRIVE_CMD_BRAKE: p_nextState = &DRIVE_STATE_BRAKE; break;
//             case MOT_DRIVE_CMD_THROTTLE: p_nextState = &DRIVE_STATE_THROTTLE; break;
//             case MOT_DRIVE_CMD_RELEASE: p_nextState = &DRIVE_STATE_RELEASE; break;
//             default: break;
//         }
//     }

//     return p_nextState;
// }
// static void Throttle_Entry(const MotDrive_T * p_motDrive)
// {
//     p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd = MOT_DRIVE_CMD_RELEASE;
//     MotMotors_ActivateControlState(&p_motDrive->MOTORS, PHASE_OUTPUT_VPWM);
// }

// static void Throttle_Proc(const MotDrive_T * p_motDrive)
// {
//     // MotMotors_SetCmdWith(&p_motDrive->MOTORS, p_motDrive->P_MOT_DRIVE_STATE.p_ThrottleFunction, p_motDrive->P_MOT_DRIVE_STATE->Input.ThrottleValue);

//     // switch (id)
//     // {
//     //     case MOT_DRIVE_CMD_BRAKE:
//     //         if (value != 0U) /* ignore brake if simultaneous input for 0, async input only */
//     //         {
//     //             MotDrive_StartBrakeMode(p_this);
//     //             // MotDrive_SetBrakeValue(p_this, value); overwritten unless Start Mode is Async ProcInput
//     //             p_this->DriveSubState = MOT_DRIVE_CMD_BRAKE;
//     //             // MotDrive_StartDriveZero(p_this);
//     //             // p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//     //         }
//     //         break;
//     //     case MOT_DRIVE_CMD_THROTTLE:
//     //         MotDrive_SetThrottleValue(p_this, value);
//     //         if (value == 0U)
//     //         {
//     //             MotDrive_StartDriveZero(p_this);
//     //             p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//     //         }
//     //         break;
//     //     case MOT_DRIVE_CMD_RELEASE:
//     //         MotDrive_StartDriveZero(p_this);
//     //         p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//     //         break;
//     // }
// }

// /* SubState handles entry */
// static const State_T DRIVE_STATE_THROTTLE =
// {
//     .P_PARENT   = &STATE_DRIVE,
//     .P_TOP      = &STATE_DRIVE,
//     .DEPTH      = 1U,
//     .ENTRY      = (State_Action_T)Throttle_Entry,
//     .LOOP       = (State_Action_T)Throttle_Proc,
//     // .NEXT       = (State_InputVoid_T) ,
// };