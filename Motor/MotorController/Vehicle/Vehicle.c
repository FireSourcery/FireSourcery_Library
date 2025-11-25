/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Vehicle.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "Vehicle.h"


void _Vehicle_InitFrom(Vehicle_State_T * p_fields, const Vehicle_Config_T * p_config)
{
    if (p_config != NULL) { p_fields->Config = *p_config; }
}

void Vehicle_Init(const Vehicle_T * p_handle)
{
    _Vehicle_InitFrom(p_handle->P_VEHICLE_STATE, p_handle->P_NVM_CONFIG);
    StateMachine_Init(&p_handle->STATE_MACHINE);
}



// static inline void Input_FromThrottle(Motor_User_Input_T * p_user, Vehicle_ThrottleMode_T mode, uint16_t throttle)
// {
//     p_user->CmdValue = (int32_t)throttle / 2; // [0:32767]

//     switch (mode)
//     {
//         case VEHICLE_THROTTLE_MODE_SPEED:   p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT;      break;
//         case VEHICLE_THROTTLE_MODE_TORQUE:  p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT;            break;
//         default: break;
//     }
// }

/******************************************************************************/
/*
    Called from StateMachine
    State Mode use full context
*/
/******************************************************************************/
/* Alternatively as substates */
void Vehicle_StartThrottleMode(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.ThrottleMode)
    {
        case VEHICLE_THROTTLE_MODE_SPEED:  MotMotors_SetFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);     break;
        case VEHICLE_THROTTLE_MODE_TORQUE: MotMotors_SetFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);           break;
        default: break;
    }

    /* alternatively from Release only */
    MotMotors_ActivateControlState(&p_vehicle->MOTORS, PHASE_OUTPUT_VPWM);
}

void Vehicle_SetThrottleValue(const Vehicle_T * p_vehicle, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;

    switch (p_vehicle->P_VEHICLE_STATE->Config.ThrottleMode)
    {
        case VEHICLE_THROTTLE_MODE_SPEED:  MotMotors_SetCmdWith(&p_vehicle->MOTORS, Motor_User_SetSpeedCmd_Scalar, (int32_t)cmdValue);     break;
        case VEHICLE_THROTTLE_MODE_TORQUE: MotMotors_SetCmdWith(&p_vehicle->MOTORS, Motor_User_SetICmd_Scalar, (int32_t)cmdValue);         break;
        default: break;
    }
}

// apply hold on low speed
void Vehicle_StartBrakeMode(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.BrakeMode)
    {
        case VEHICLE_BRAKE_MODE_TORQUE:  MotMotors_SetFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);  break;
        // case VEHICLE_BRAKE_MODE_VOLTAGE: MotMotors_SetFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_VOLTAGE);  break;
        default: break;
    }

    /* alternatively from Release only */
    MotMotors_ActivateControlState(&p_vehicle->MOTORS, PHASE_OUTPUT_VPWM);
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
void Vehicle_SetBrakeValue(const Vehicle_T * p_vehicle, uint16_t userCmdBrake)
{
    int16_t cmdValue = 0 - ((int32_t)userCmdBrake / 2); // 32767 max

    switch (p_vehicle->P_VEHICLE_STATE->Config.BrakeMode)
    {
        case VEHICLE_BRAKE_MODE_TORQUE: MotMotors_SetCmdWith(&p_vehicle->MOTORS, Motor_User_SetICmd_Scalar, cmdValue); break;
        // case VEHICLE_BRAKE_MODE_VOLTAGE: MotMotors_SetCmdWith(&p_vehicle->MOTORS, Motor_User_SetRegenCmd, 0); break;
        default: break;
    }
}

/* an alternate cmd for float is required */
void Vehicle_StartDriveZero(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.ZeroMode)
    {
        case VEHICLE_ZERO_MODE_FLOAT:     MotMotors_ActivateControlState(&p_vehicle->MOTORS, PHASE_OUTPUT_FLOAT);        break;
        case VEHICLE_ZERO_MODE_CRUISE:    MotMotors_SetFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);    break;
        case VEHICLE_ZERO_MODE_REGEN:       /* Vehicle_SetRegenMotorAll(p_this); */ break;
        default: break;
    }
}

/*
    Check Stop / Zero Throttle
    Eventually release for stop transition
*/
void Vehicle_ProcDriveZero(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.ZeroMode)
    {
        case VEHICLE_ZERO_MODE_FLOAT: break;
        case VEHICLE_ZERO_MODE_CRUISE: MotMotors_SetCmdWith(&p_vehicle->MOTORS, Motor_User_SetICmd_Scalar, 0); break;
        case VEHICLE_ZERO_MODE_REGEN: break;
        default: break;
    }
}


