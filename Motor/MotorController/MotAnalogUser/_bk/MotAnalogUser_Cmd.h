// typedef enum MotAnalogUser_Cmd
// {
//     MOT_ANALOG_USER_CMD_SET_BRAKE,
//     MOT_ANALOG_USER_CMD_SET_THROTTLE,
//     MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE,
//     MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE,
//     MOT_ANALOG_USER_CMD_PROC_ZERO,    /* No Brake/Throttle input */
// }
// MotAnalogUser_Cmd_T;
// /* MOT_ANALOG_USER_CMD_SET_NEUTRAL,
// MOT_ANALOG_USER_CMD_PROC_NEUTRAL,
// MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD,
// MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE, */
// static inline MotAnalogUser_Direction_T MotAnalogUser_GetDirectionEdge(const MotAnalogUser_T * p_user)
// {
//     MotAnalogUser_Direction_T direction;

//     if (_MotAnalogUser_IsNeutralOn(p_user) == true)
//     {
//         direction = UserDIn_IsRisingEdge(&p_user->NEUTRAL_DIN) ? MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE : MOT_ANALOG_USER_DIRECTION_NEUTRAL;
//     }
//     else if (MotAnalogUser_IsReverseOn(p_user) == true)
//     {
//         direction = UserDIn_IsRisingEdge(&p_user->REVERSE_DIN) ? MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE : MOT_ANALOG_USER_DIRECTION_REVERSE;
//     }
//     else if (MotAnalogUser_IsForwardOn(p_user) == true)
//     {
//         direction = UserDIn_IsRisingEdge(&p_user->FORWARD_DIN) ? MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE : MOT_ANALOG_USER_DIRECTION_FORWARD;
//     }
//     else
//     {
//         direction = ((UserDIn_IsFallingEdge(&p_user->FORWARD_DIN) == true) || (UserDIn_IsFallingEdge(&p_user->REVERSE_DIN) == true)) ?
//             MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE : MOT_ANALOG_USER_DIRECTION_NEUTRAL;
//     }

//     return direction;
// }

/*
    Get Command from current state
    Directly call Query functions to get current state. Poll to clear edge is not required since update.
*/
// static inline MotAnalogUser_Cmd_T MotAnalogUser_GetCmd(const MotAnalogUser_T * p_user)
// {
//     MotAnalogUser_Direction_T direction = MotAnalogUser_GetDirection(p_user);
//     MotAnalogUser_Cmd_T cmd = MOT_ANALOG_USER_CMD_PROC_ZERO;

//     switch (direction)
//     {
//         case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD; break;
//         case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE: cmd = MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE; break;
//         default:
//             /* Check Brake first */
//             if (UserAIn_IsOn(&p_user->BRAKE_AIN) || MotAnalogUser_IsSwitchBrakeOn(p_user)) { cmd = MOT_ANALOG_USER_CMD_SET_BRAKE; }
//             /* Both Brakes are off - check for release */
//             else if (UserAIn_IsFallingEdge(&p_user->BRAKE_AIN) || MotAnalogUser_IsSwitchBrakeFallingEdge(p_user)) { cmd = MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE; }
//             /* Check Direction */
//             else if (direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE)   { cmd = MOT_ANALOG_USER_CMD_SET_NEUTRAL; }
//             else if (direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL)        { cmd = MOT_ANALOG_USER_CMD_PROC_NEUTRAL; }
//             /* Check Throttle */
//             else if (UserAIn_IsOn(&p_user->THROTTLE_AIN) == true)           { cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE; }
//             else if (UserAIn_IsFallingEdge(&p_user->THROTTLE_AIN) == true)  { cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE; }
//             /* Direction is Forward or Reverse, no throttle or brake value */
//             else { cmd = MOT_ANALOG_USER_CMD_PROC_ZERO; }
//             break;
//     }

//     p_user->P_STATE->Cmd = cmd;
//     return cmd;
// }

// static inline MotAnalogUser_Cmd_T MotAnalogUser_GetCmd(const MotAnalogUser_T * p_user)
// {
//     MotAnalogUser_Direction_T direction = MotAnalogUser_GetDirection(p_user);
//     MotAnalogUser_Cmd_T cmd = MOT_ANALOG_USER_CMD_PROC_ZERO;

//     if (UserAIn_IsOn(&p_user->BRAKE_AIN) || MotAnalogUser_IsSwitchBrakeOn(p_user)) { cmd = MOT_ANALOG_USER_CMD_SET_BRAKE; }
//     else if (UserAIn_IsOn(&p_user->THROTTLE_AIN) == true)           { cmd = MOT_ANALOG_USER_CMD_SET_THROTTLE; }
//     /* Direction is Forward or Reverse, no throttle or brake value */
//     else { cmd = MOT_ANALOG_USER_CMD_PROC_ZERO; }
//     return cmd;
// }