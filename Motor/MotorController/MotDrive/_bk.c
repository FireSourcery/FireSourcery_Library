


/* either set drive 0 first, Motor set control twice. or motor directly transition */
// static State_T * Drive_InputThrottle(const MotDrive_T * p_motDrive, state_value_t cmdValue)
// {
//     // if (cmdValue == 0U) { SetDriveZero(p_this, MOT_DRIVE_CMD_THROTTLE); } else { SetThrottle(p_this, cmdValue); }
//     // return NULL;
//     return _Drive_InputDrive(p_this, MOT_DRIVE_CMD_THROTTLE, cmdValue);
// }

// static State_T * Drive_InputBrake(const MotDrive_T * p_motDrive, state_value_t cmdValue)
// {
//     // if (cmdValue == 0U) { SetDriveZero(p_this, MOT_DRIVE_CMD_BRAKE); } else { SetBrake(p_this, cmdValue); }
//     // return NULL;
//     return _Drive_InputDrive(p_this, MOT_DRIVE_CMD_BRAKE, cmdValue);
// }




// static void SetBrake(const MotDrive_T * p_motDrive, uint32_t cmdValue)
// {
//     if (p_this->DriveSubState == MOT_DRIVE_CMD_BRAKE)
//     {
//         MotDrive_SetBrakeValue(p_this, cmdValue);
//     }
//     else /* overwrite throttle */
//     {
//         // Brake has been released, and reapplied. Drive case need check for 0 speed for transition to park
//         if (p_this->DriveSubState == MOT_DRIVE_CMD_ZERO) && (MotDrive_IsEveryMotorStopState(p_this) == true)
//         {
//            MotDrive_TryHoldAll(p_this);
//             // p_this->StateFlags.IsStopped = 1U;
//         }

//         MotDrive_StartBrakeMode(p_this);
//         MotDrive_SetBrakeValue(p_this, cmdValue);
//         p_this->DriveSubState = MOT_DRIVE_CMD_BRAKE;
//     }
// }

// static void SetThrottle(const MotDrive_T * p_motDrive, uint32_t cmdValue)
// {
//     if (p_this->DriveSubState == MOT_DRIVE_CMD_THROTTLE)
//     {
//         MotDrive_SetThrottleValue(p_this, cmdValue);
//     }
//     else if (p_this->DriveSubState == MOT_DRIVE_CMD_ZERO || p_this->DriveSubState == MOT_DRIVE_CMD_CMD) /* do not overwrite brake */
//     {
//         p_this->DriveSubState = MOT_DRIVE_CMD_THROTTLE;
//         MotDrive_StartThrottleMode(p_this);
//         MotDrive_SetThrottleValue(p_this, cmdValue);
//     }
// }

// if (cmdValue != 0U)
// {
//     if (p_this->DriveSubState == MOT_DRIVE_CMD_CMD)
//     {
//         MotDrive_SetCmdModeValue(p_this, cmdValue);
//     }
//     else if (p_this->DriveSubState == MOT_DRIVE_CMD_RELEASE)
//     {
//         MotDrive_StartCmdModeDefault(p_this);
//         MotDrive_SetCmdModeValue(p_this, cmdValue);
//         p_this->DriveSubState = MOT_DRIVE_CMD_CMD;
//     }
// }
// else
// {
//     p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
// }

// static void SetDriveZero(const MotDrive_T * p_motDrive, MotDrive_Cmd_T driveSubState)
// {
//     if (p_this->DriveSubState == MOT_DRIVE_CMD_ZERO)
//     {
//         MotDrive_ProcDriveZero(p_this);
//     }
//     else if (p_this->DriveSubState == driveSubState) /* Only override is prev input mode matches. Do not overwrite other mode on 0 */
//     {
//         p_this->DriveSubState = MOT_DRIVE_CMD_ZERO;
//         MotDrive_StartDriveZero(p_this);
//     }
// }



// /*! @param[in] driveCmd MotDrive_Cmd_T */
// static State_T * _Drive_InputDrive(const MotDrive_T * p_motDrive, MotDrive_Cmd_T id, uint32_t value)
// {
//     // if ((value == 0U) && (id == p_this->DriveSubState)) { id = MOT_DRIVE_CMD_RELEASE; }

//     switch (p_this->DriveSubState) // switch on current state
//     {
//         case MOT_DRIVE_CMD_BRAKE:
//             switch (id)
//             {
//                 case MOT_DRIVE_CMD_BRAKE:
//                     MotDrive_SetBrakeValue(p_this, value);
//                     if (value == 0U)
//                     {
//                         MotDrive_StartDriveZero(p_this);
//                         p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//                     }
//                     break;
//                 case MOT_DRIVE_CMD_THROTTLE:
//                     break;
//                 case MOT_DRIVE_CMD_RELEASE: /* UI detected release */
//                     MotDrive_StartDriveZero(p_this);
//                     p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//                     break;
//                 case MOT_DRIVE_CMD_CMD: break;
//             }
//             break;
//         case MOT_DRIVE_CMD_THROTTLE:
//             switch (id)
//             {
//                 case MOT_DRIVE_CMD_BRAKE:
//                     if (value != 0U) /* ignore brake if simultaneous input for 0, async input only */
//                     {
//                         MotDrive_StartBrakeMode(p_this);
//                         // MotDrive_SetBrakeValue(p_this, value); overwritten unless Start Mode is Async ProcInput
//                         p_this->DriveSubState = MOT_DRIVE_CMD_BRAKE;
//                         // MotDrive_StartDriveZero(p_this);
//                         // p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//                     }
//                     break;
//                 case MOT_DRIVE_CMD_THROTTLE:
//                     MotDrive_SetThrottleValue(p_this, value);
//                     if (value == 0U)
//                     {
//                         MotDrive_StartDriveZero(p_this);
//                         p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//                     }
//                     break;
//                 case MOT_DRIVE_CMD_RELEASE:
//                     MotDrive_StartDriveZero(p_this);
//                     p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//                     break;
//                 case MOT_DRIVE_CMD_CMD: break;
//             }
//             break;
//         case MOT_DRIVE_CMD_RELEASE:
//             switch (id)
//             {
//                 case MOT_DRIVE_CMD_BRAKE:
//                     if (value != 0U)
//                     {
//                         // todo 0 speed or passive substate
//                         if (MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) == true) { MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Hold); }
//                         else
//                         {
//                             MotDrive_StartBrakeMode(p_this);
//                             // MotDrive_SetBrakeValue(p_this, value);
//                             p_this->DriveSubState = MOT_DRIVE_CMD_BRAKE;
//                         }
//                     }
//                     else
//                     {
//                         MotDrive_ProcDriveZero(p_this);
//                     }
//                     break;
//                 case MOT_DRIVE_CMD_THROTTLE:
//                     if (value != 0U)
//                     {
//                         MotDrive_StartThrottleMode(p_this);
//                         p_this->DriveSubState = MOT_DRIVE_CMD_THROTTLE;
//                     }
//                     else
//                     {
//                         MotDrive_ProcDriveZero(p_this);
//                     }
//                     break;
//                 case MOT_DRIVE_CMD_RELEASE:
//                     // MotDrive_ProcDriveZero(p_this);
//                     // MotDrive_StartcDriveZero(p_this);
//                     break;
//                 case MOT_DRIVE_CMD_CMD: break;
//             }
//             break;
//         case MOT_DRIVE_CMD_CMD:
//             switch (id)
//             {
//                 case MOT_DRIVE_CMD_BRAKE:
//                     if (value != 0U)
//                     {
//                         MotDrive_StartBrakeMode(p_this);
//                         p_this->DriveSubState = MOT_DRIVE_CMD_BRAKE;
//                     }
//                     break;
//                 case MOT_DRIVE_CMD_RELEASE: break;
//                 case MOT_DRIVE_CMD_THROTTLE: break;
//                 case MOT_DRIVE_CMD_CMD: break;
//             }
//             break;
//     }
//     return NULL;
// }

// //if config.EnableBrakeInNeutral == true
// switch (p_this->DriveSubState) // switch on current state
// {
//     case MOT_DRIVE_CMD_BRAKE:
//         MotDrive_SetBrakeValue(p_this, cmdValue);
//         if (cmdValue == 0U)
//         {
//             MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Release);
//             p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//         }
//         break;
//     case MOT_DRIVE_CMD_RELEASE:
//         MotDrive_StartBrakeMode(p_this);
//         p_this->DriveSubState = MOT_DRIVE_CMD_BRAKE;
//         break;
//     default: /* MOT_DRIVE_CMD_THROTTLE */
//         MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Release);
//         p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE;
//         break;
// }



// with hsm
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